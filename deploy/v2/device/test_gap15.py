import ast
from pathlib import Path
import unittest
import run_policy as r
class Gap15Tests(unittest.TestCase):
    def test_explicit15_keeps_observation_and_default(self):
        a=r.parse_args(['--start-confirmed','--port','FAKE','--model','unused','--command-gap-hard-ms','15'])
        self.assertEqual(a.command_gap_hard_ms,15)
        self.assertEqual(r.timing_limits(a.timing_profile)[1],.004)
        default=r.parse_args(['--start-confirmed','--port','FAKE','--model','unused'])
        self.assertIsNone(default.command_gap_hard_ms)
        self.assertEqual(r.timing_limits(default.timing_profile)[0],.006)
    def test_nine_ms_count_independent_of_actual_hard_and_pause(self):
        tree=ast.parse(Path(r.__file__).read_text(encoding='utf-8'))
        nodes=[n for n in ast.walk(tree) if isinstance(n,ast.AugAssign) and isinstance(n.target,ast.Subscript) and isinstance(n.target.slice,ast.Constant) and n.target.slice.value in ('command_gaps_at_or_over_9ms','command_gaps_at_or_over_hard_limit')]
        self.assertEqual(len(nodes),2)
        for gap,pause,expected in [(.010,False,(1,0)),(.015,False,(1,1)),(.020,True,(0,0))]:
            totals={'command_gaps_at_or_over_9ms':0,'command_gaps_at_or_over_hard_limit':0}
            exec(compile(ast.Module(body=nodes,type_ignores=[]),'<actual counters>','exec'),dict(totals=totals,command_gap=gap,intentional_gap=pause,hard_limit_s=.015))
            self.assertEqual(tuple(totals.values()),expected)
