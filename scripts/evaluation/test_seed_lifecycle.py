"""真实torch张量验证跨seed reset生命周期，不启动仿真器。"""
import unittest
import torch
from evaluate_policy import run_seed_cohort
from episode_metrics import EpisodeMetrics

class SeedLifecycleTests(unittest.TestCase):
    def test_second_seed_reset_can_modify_inference_cache(self):
        cache={'history':torch.zeros(1)}
        resets=[]
        def reset(seed):
            resets.append(seed)
            cache['history'].zero_()  # 第二seed修改首回合创建的inference tensor
        results=[]
        for seed in [101,202]:
            metrics=EpisodeMetrics(num_envs=1,seed=seed,dt=.002)
            def step_factory():
                def step():
                    cache['history']=torch.cat([cache['history'],torch.ones(1)])
                    metrics.record(states=[[0,0,0,0]],actions=[0],terminated=[False],
                                   truncated=[True],reasons={'time_out':[True]})
                return step
            results.append(run_seed_cohort(metrics,reset=reset,step_factory=step_factory,
                                           inference_context=torch.inference_mode,max_steps=2))
        self.assertTrue(torch.is_inference(cache['history']))
        self.assertEqual(resets,[101,202])
        self.assertEqual([r['completed'] for r in results],[1,1])
        self.assertEqual([r['seed'] for r in results],[101,202])

    def test_failure_slots_remain_counted_once_across_repeated_steps(self):
        metrics=EpisodeMetrics(num_envs=2,seed=101,dt=.002)
        count=[0]
        def factory():
            def step():
                count[0]+=1
                metrics.record(states=[[.3,0,0,0],[0,0,0,0]],actions=[0,0],
                               terminated=[True,False],truncated=[False,count[0]==2],
                               reasons={'cart_out_of_bounds':[True,False],'time_out':[False,count[0]==2]})
            return step
        result=run_seed_cohort(metrics,reset=lambda seed:None,step_factory=factory,
                               inference_context=torch.inference_mode,max_steps=3)
        self.assertEqual(result['completed'],2)
        self.assertEqual(result['successes'],1)
        self.assertEqual(result['success_rate'],.5)
        self.assertEqual([e['steps'] for e in result['episodes']],[1,2])

if __name__=='__main__': unittest.main()
