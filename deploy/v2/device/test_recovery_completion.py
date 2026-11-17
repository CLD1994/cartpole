import unittest
from run_policy import completion_violation, parse_args
class CompletionTests(unittest.TestCase):
    def test_age_only_is_warning(self):
        self.assertEqual(completion_violation(.0041,.004,.005,.025), 'observation_age')
    def test_hard_gap_still_fatal(self):
        self.assertEqual(completion_violation(.0041,.004,.025,.025), 'command_gap')
    def test_valid(self):
        self.assertIsNone(completion_violation(.003,.004,.005,.025))

    def test_recovery_write_uses_active_budget_not_intentional_gap_exemption(self):
        import ast
        from pathlib import Path
        tree=ast.parse(Path(__file__).with_name('run_policy.py').read_text(encoding='utf8'))
        call=next(n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='completion_violation')
        self.assertEqual(ast.unparse(call.args[2]),'active_command_gap')
        assigns=[n for n in ast.walk(tree) if isinstance(n,ast.Assign)]
        gap=next(n for n in assigns if any(isinstance(t,ast.Name) and t.id=='active_command_gap' for t in n.targets))
        reset=next(n for n in assigns if len(n.targets)==2 and all(isinstance(t,ast.Name) for t in n.targets) and [t.id for t in n.targets]==['last_command_at','budget_at'] and ast.unparse(n.value)=='completed_at')
        self.assertLess(gap.lineno,reset.lineno)
        self.assertEqual(completion_violation(.030,.004,.030,.025),'command_gap')
