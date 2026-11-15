"""启动参数和生命周期适配边界；通过公开接口注入替身。"""
import unittest
from types import SimpleNamespace
from evaluate_policy import parse_args, make_recorder_type

class ArgumentTests(unittest.TestCase):
    def test_explicit_seeds_and_horizon(self):
        args = parse_args(['--checkpoint','model.pt','--seeds','10','20','--num_envs','8',
                           '--horizon-s','60'], add_launcher_args=lambda p: None)
        self.assertEqual(args.seeds,[10,20])
        self.assertEqual(args.num_envs,8)
        self.assertEqual(args.horizon_s,60)

    def test_duplicate_seeds_rejected(self):
        from contextlib import redirect_stderr
        from io import StringIO
        with redirect_stderr(StringIO()), self.assertRaises(SystemExit):
            parse_args(['--checkpoint','model.pt','--seeds','10','10'], add_launcher_args=lambda p: None)

    def test_recorder_consumes_snapshot_at_post_step(self):
        calls=[]
        env = object()
        class Base:
            def __init__(self): self._env=env
        def snapshot(actual):
            self.assertIs(actual,env)
            calls.append('snapshot')
            return {'states': [[1,2,3,4]]}
        def consume(value): calls.append(value)
        recorder=make_recorder_type(Base,snapshot=snapshot,consume=consume)()
        self.assertEqual(recorder.record_post_step(),(None,None))
        self.assertEqual(calls,['snapshot',{'states':[[1,2,3,4]]}])



class SnapshotTests(unittest.TestCase):
    def test_snapshot_copies_terminal_state_and_all_causes(self):
        import torch
        from evaluate_policy import snapshot_environment
        positions=torch.tensor([[.4,1.2]],dtype=torch.float64)
        class Robot:
            data=SimpleNamespace(joint_pos=SimpleNamespace(torch=positions),
                                 joint_vel=SimpleNamespace(torch=torch.tensor([[.1,.2]])))
            def find_joints(self,name):
                return ([0],['slider_to_cart']) if name=='slider_to_cart' else ([1],['cart_to_pole'])
        manager=SimpleNamespace(terminated=torch.tensor([True]),time_outs=torch.tensor([True]),
                 active_terms=['time_out','cart_out_of_bounds'],get_term=lambda name:torch.tensor([True]))
        env=SimpleNamespace(scene={'robot':Robot()},termination_manager=manager,
                            action_manager=SimpleNamespace(action=torch.tensor([[.8]])))
        result=snapshot_environment(env)
        positions.zero_()  # 模拟下一阶段自动reset，不应篡改已取出的终态
        self.assertAlmostEqual(result['states'][0][0],.4)
        self.assertAlmostEqual(result['states'][0][1],1.2)
        self.assertEqual(result['reasons'],{'time_out':[True],'cart_out_of_bounds':[True]})

class ResetHookTests(unittest.TestCase):
    def test_pre_reset_passes_ids_to_guard(self):
        calls=[]
        recorder=make_recorder_type(object,snapshot=lambda env:None,consume=lambda data:None,
                                    before_reset=calls.append)()
        self.assertEqual(recorder.record_pre_reset([2]),(None,None))
        self.assertEqual(calls,[[2]])

if __name__ == '__main__': unittest.main()
