"""Replay an existing host command trace in simulation; no policy, training or serial IO."""
from __future__ import annotations
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
from evaluate_policy import (make_recorder_type, snapshot_environment, snapshot_state,
                             snapshot_initial_conditions, write_report)


def load_trace(path):
    data = json.loads(path.read_text(encoding='utf-8-sig'))
    rows = data['samples']
    if data.get('diagnostic_timing') or not rows or not data.get('motion_command_sent'):
        raise ValueError('requires actual motion report, not virtual diagnostic')
    # Absolute host write-completion timestamp, not late report append time.
    origin = rows[0]['grid_due_clock_s'] - rows[0]['grid_slot']*.002
    times = [r['observation_received_clock_s']+r['observation_host_age_s']-origin for r in rows]
    if any(not math.isfinite(t) or t<0 for t in times) or any(b<=a for a,b in zip(times,times[1:])):
        raise ValueError('invalid command clock ordering')
    velocities = [r['velocity_mm_s']/1000. for r in rows]
    if any(not math.isfinite(v) or abs(v)>1.25 for v in velocities):
        raise ValueError('command exceeds fixed replay authority')
    sign = data['contract']['angle_sign']
    if sign not in (-1,1) or data['contract']['x_sign'] != 1 or data['contract']['action_sign'] != 1:
        raise ValueError('unsupported coordinate mapping')
    initial = list(data['initial_state_check']['state_si'])
    initial[1] *= sign; initial[3] *= sign
    actual = []
    for r in rows:
        state = list(r['state_si']);state[1] *= sign;state[3] *= sign
        actual.append({'time_s':r['observation_received_clock_s']-origin,'state_si':state})
    return data, times, velocities, initial, actual


def initialize_replay_state(env, initial):
    """Simulation-only explicit state write after reset; do not trust reset clamping."""
    robot=env.scene['robot']
    cart,_=robot.find_joints('slider_to_cart');pole,_=robot.find_joints('cart_to_pole')
    if len(cart)!=1 or len(pole)!=1:raise ValueError('expected unique cart and pole joints')
    if len(initial)!=4 or not all(math.isfinite(v) for v in initial):
        raise ValueError('invalid initial state')
    before=snapshot_state(env)
    soft_limits=robot.data.soft_joint_vel_limits.torch.detach().cpu().tolist()
    q=robot.data.joint_pos.torch.clone();v=robot.data.joint_vel.torch.clone()
    q[:,cart[0]]=initial[0];q[:,pole[0]]=initial[1]
    v[:,cart[0]]=initial[2];v[:,pole[0]]=initial[3]
    robot.write_joint_state_to_sim_index(position=q,velocity=v)
    after=snapshot_state(env)
    # Independent PhysX backend readback, not ArticulationData's write-through cache.
    names=list(robot.backend_joint_names)
    c=names.index('slider_to_cart');p=names.index('cart_to_pole')
    backend_q=robot.root_view.get_dof_positions().numpy()
    backend_v=robot.root_view.get_dof_velocities().numpy()
    backend=[float(backend_q[0,c]),float(backend_q[0,p]),float(backend_v[0,c]),float(backend_v[0,p])]
    if not all(math.isfinite(v) for v in backend) or any(abs(actual-wanted)>1e-5 for actual,wanted in zip(backend,initial)):
        raise RuntimeError(f'backend initial-state mismatch: {backend} vs {initial}')
    if not all(math.isfinite(v) for v in after[0]) or any(abs(actual-wanted)>1e-5 for actual,wanted in zip(after[0],initial)):
        raise RuntimeError(f'explicit initial-state readback mismatch: {after[0]} vs {initial}')
    return {'reset_state_before_explicit_write':before,'soft_joint_velocity_limits':soft_limits,
            'explicit_write_readback':after,'backend_readback':backend,
            'scope':'PhysX root_view independent readback plus state interface, before first step'}


def comparison(samples, actual):
    # Never extrapolate beyond simulated trace; separately expose early termination.
    samples = [s for s in samples if s['velocity_window16_m_s'] is not None]
    times = [s['time_s'] for s in samples]
    result = {}
    for lo,hi,name in [(0.016,1.,'startup'),(1.,3.,'middle'),(3.,5.,'late')]:
        pairs = []
        for row in actual:
            t=row['time_s']
            if not lo<=t<hi or not times or not times[0]<=t<=times[-1]:continue
            j=bisect.bisect_left(times,t)
            if j==0:pred=samples[0]['velocity_window16_m_s']
            else:
                f=(t-times[j-1])/(times[j]-times[j-1])
                pred=samples[j-1]['velocity_window16_m_s']*(1-f)+samples[j]['velocity_window16_m_s']*f
            if pred is not None:pairs.append((pred,row['state_si'][2]))
        result[name]={'matched_samples':len(pairs),'cart_velocity_rmse_m_s':
                      math.sqrt(sum((a-b)**2 for a,b in pairs)/len(pairs)) if pairs else None}
    return result


def main():
    import warp as wp
    wp.config.enable_backward = False
    import cartpole.tasks
    import gymnasium as gym
    import torch
    from isaaclab.app import add_launcher_args, launch_simulation
    from isaaclab.managers import RecorderTerm, RecorderTermCfg, RecorderManagerBaseCfg, DatasetExportMode
    from isaaclab_tasks.utils import resolve_task_config
    from isaaclab.utils.io import dump_yaml
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trace',type=Path,required=True)
    p.add_argument('--tau-s',type=float,required=True)
    p.add_argument('--delay-ms',type=int,choices=[0,2,4,6,8,10,12,14,16],default=6)
    p.add_argument('--output-dir',type=Path,required=True)
    add_launcher_args(p);a=p.parse_args();a.physics='isaacsim_physx'
    if not math.isfinite(a.tau_s) or a.tau_s<0:p.error('tau must be finite and nonnegative')
    trace,times,velocities,initial,actual=load_trace(a.trace)
    duration=min(5.,float(trace['requested_seconds']),times[-1])
    a.output_dir.mkdir(parents=True,exist_ok=False)
    report={'status':'starting','scope':'open-loop recorded command replay; not hardware or closed-loop validation',
            'trace_sha256':hashlib.sha256(a.trace.read_bytes()).hexdigest(),
            'replay_revision':2,'tau_s':a.tau_s,'delay_ms':a.delay_ms,'initial_requested':initial,'samples':[],
            'limitations':['host completion/parse times, not device timestamps',
                'initial state was observed before grid origin; time alignment is approximate',
                'replay ends before final recorded command; its subsequent response is not covered',
                'commands held on next 2ms control boundary; up to2ms quantization',
                '16ms ideal endpoint velocity approximation',
                'fixed nominal physics, not fitted real mechanics; no policy inference',
                'tau=0 removes only simulation first-order reference filter; does not reproduce stepper ISR exactly']}
    out=a.output_dir/'report.json';write_report(out,report)
    cfg,_=resolve_task_config('Hexchip-Cartpole-Real-V2-v0','rsl_rl_cfg_entry_point',play_mode=False,overrides=[])
    cfg.scene.num_envs=1;cfg.sim.device=a.device;cfg.seed=42;cfg.log_dir=None
    cfg.episode_length_s=duration+.004
    cfg.observations.policy.enable_corruption=False
    # Keep reset setters: the USD cart mass is not the calibrated nominal mass.
    # Collapse randomization ranges, preserving exactly the same fixed physics in A/B.
    masses = cfg.events.randomize_body_mass.params['mass_range']
    for key,(lo,hi) in list(masses.items()):
        masses[key] = ((lo+hi)/2, (lo+hi)/2)
    pole = cfg.events.randomize_pole_properties.params
    for key in ('mass_range','com_offset_range'):
        lo,hi=pole[key];pole[key]=((lo+hi)/2,(lo+hi)/2)
    pole['inertia_scale_range']=(1.,1.)
    damping=cfg.events.randomize_joint_damping.params['damping_range']
    for key in damping: damping[key]=(0.,0.)
    for name,pos,vel in [('reset_cart_position',initial[0],initial[2]),('reset_pole_position',initial[1],initial[3])]:
        term=getattr(cfg.events,name);term.params['position_range']=(pos,pos);term.params['velocity_range']=(vel,vel)
    cfg.events.randomize_tracking.params.update(vel_max_range=(1.25,1.25),tau_range=(a.tau_s,a.tau_s),acc_max_range=(10.,10.))
    cfg.events.randomize_motor_delay.params['delay_range']=(a.delay_ms//2,a.delay_ms//2)
    cfg.actions.joint_effort.tau=a.tau_s
    dt=float(cfg.sim.dt*cfg.decimation)
    if not math.isclose(dt,.002,abs_tol=1e-9):raise ValueError('replay requires2ms control timestep')
    terminated=[False];position_history=[initial[0]];step_index=[0]
    def consume(snapshot):
        step_index[0]+=1
        state=snapshot['states'][0];position_history.append(state[0])
        n=step_index[0]
        report['samples'].append({'time_s':n*dt,'state_si':state,
            'action':snapshot['actions'][0],
            'velocity_window16_m_s':(state[0]-position_history[n-8])/.016 if n>=8 else None})
        terminated[0]=bool(snapshot['terminated'][0] or snapshot['truncated'][0])
        if terminated[0]:report['termination_reasons']=[k for k,v in snapshot['reasons'].items() if v[0]]
    rec=RecorderManagerBaseCfg();rec.dataset_export_mode=DatasetExportMode.EXPORT_NONE
    rec.export_in_record_pre_reset=False;rec.export_in_close=False
    rec.replay=RecorderTermCfg(class_type=make_recorder_type(RecorderTerm,snapshot=snapshot_environment,consume=consume))
        # Save the physical config without a runtime-local recorder class.
    try:
        with launch_simulation(cfg,a):
            dump_yaml(str(a.output_dir/'env.yaml'),cfg)
            cfg.recorders=rec
            env=gym.make('Hexchip-Cartpole-Real-V2-v0',cfg=cfg)
            try:
                with torch.inference_mode():
                    env.reset(seed=42)
                    report['initialization']=initialize_replay_state(env.unwrapped,initial)
                    report['initial_actual']=snapshot_initial_conditions(env.unwrapped)
                    report['initial_actual']['states']=[report['initialization']['backend_readback']]
                    report['initial_actual']['state_source']='PhysX root_view independent readback'
                    position_history[0]=snapshot_state(env.unwrapped)[0][0]
                    for k in range(math.floor(duration/dt)):
                        j=bisect.bisect_right(times,k*dt)-1
                        velocity=velocities[j] if j>=0 else 0.
                        env.step(torch.tensor([[velocity/1.25]],device=env.unwrapped.device))
                        if terminated[0]:break
                    report['status']='terminated_early' if terminated[0] else 'complete'
                    report['comparison']=comparison(report['samples'],actual)
                    write_report(out,report)
            finally:env.close()
    except BaseException as exc:
        if report['status']=='starting':report.update(status='error',error=str(exc));write_report(out,report)
        raise
    print('REPLAY_REPORT '+str(out))
    return 0 if report['status']=='complete' else 2


if __name__=='__main__':raise SystemExit(main())
