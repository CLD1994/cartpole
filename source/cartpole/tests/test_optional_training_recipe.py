"""执行配置类的行为契约；不导入 Isaac/PhysX，不代表真实环境构建验证。"""
import ast
import copy
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).parents[1] / 'cartpole/tasks/manager_based/cartpole'


def load_classes(path, namespace):
    tree = ast.parse(path.read_text())
    classes = ast.Module(body=[node for node in tree.body if isinstance(node, ast.ClassDef)], type_ignores=[])
    exec(compile(classes, str(path), 'exec'), namespace)
    return namespace


class RecipeTests(unittest.TestCase):
    def test_window_recipe_does_not_mutate_base(self):
        class Base:
            def __post_init__(self):
                self.rewards = NS(cart_pos='center', cart_vel=NS(weight=-1))
                self.terminations = NS(cart_out_of_bounds=NS(params={'bounds':(-.3,.3)}))
                self.observations = NS(policy=NS(joint_vel_rel=NS()))
                self.actions = NS(joint_effort=NS(tau=.02, acc_max=10))
                self.events = NS(randomize_tracking=NS(params={'tau_range':(.01,.04), 'acc_max_range':(8,10)}))
                self.sim = NS(dt=.0005,render_interval=4)
                self.decimation=4
        ns=load_classes(ROOT/'cartpole_env_cfg_velocity_window.py', dict(
            configclass=lambda cls:cls, RealCartpoleV2EnvCfg=Base,
            JointVelocityWindow=object(), joint_boundary_l2=object(),
            RewTerm=lambda **kw:NS(**kw), SceneEntityCfg=lambda *a,**kw:NS(**kw)))
        recipe=ns['RealCartpoleV2Window16Tau0Acc9Hz400EnvCfg']()
        recipe.__post_init__()
        self.assertEqual(recipe.sim.dt*recipe.decimation,.0025)
        self.assertEqual(recipe.actions.joint_effort.acc_max,9)
        self.assertEqual(recipe.actions.joint_effort.tau,0)
        self.assertEqual(recipe.events.randomize_tracking.params,{'tau_range':(0.,0.),'acc_max_range':(8.,9.)})
        self.assertIsNone(recipe.rewards.cart_pos)
        self.assertEqual(recipe.rewards.cart_vel.weight,-.1)
        self.assertAlmostEqual(recipe.rewards.cart_boundary.params['free_radius'],.21)
        baseline=Base(); baseline.__post_init__()
        self.assertEqual(baseline.rewards.cart_pos,'center')
        self.assertEqual(baseline.actions.joint_effort.acc_max,10)

    def test_base_and_optional_discounts_are_distinct(self):
        class Cfg(NS):
            def copy(self): return copy.deepcopy(self)
        class Model(Cfg):
            GaussianDistributionCfg=Cfg
        ns=load_classes(ROOT/'agents/rsl_rl_ppo_cfg.py', dict(
            configclass=lambda cls:cls, math=math, V2_DT=.002,
            RslRlOnPolicyRunnerCfg=object, RslRlMLPModelCfg=Model,
            RslRlPpoAlgorithmCfg=Cfg, RslRlSymmetryCfg=Cfg,
            symmetry=NS(compute_symmetric_states=object())))
        old=ns['RealCartpoleV2PPORunnerCfg'](); old.__post_init__()
        new=ns['RealCartpoleV2WindowPPORunnerCfg'](); new.__post_init__()
        self.assertAlmostEqual(old.algorithm.gamma,.99**.4)
        self.assertAlmostEqual(new.algorithm.gamma,math.exp(-.001))
        self.assertAlmostEqual(old.algorithm.lam,new.algorithm.lam)
        self.assertEqual(new.num_steps_per_env,120)
        self.assertEqual(old.obs_groups,{'actor':['policy'],'critic':['policy']})

if __name__=='__main__': unittest.main()
