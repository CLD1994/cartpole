"""测试命令参数及退出状态的公开转发契约，不启动仿真器。"""
import unittest
import train
import play


class EntrypointsTest(unittest.TestCase):
    def test_preserves_arguments_and_exit_status(self):
        for entrypoint in (train.main, play.main):
            with self.subTest(entrypoint=entrypoint.__module__):
                calls = []
                def dispatch(args):
                    calls.append(args)
                    return 7
                args = ["--task", "Hexchip-Cartpole-Real-V2-v0", "--headless", "env.seed=42"]
                original = args.copy()
                self.assertEqual(entrypoint(args, run_cli=dispatch), 7)
                self.assertEqual(calls, [["--rl_library", "rsl_rl", *original]])
                self.assertEqual(args, original)

    def test_help_is_forwarded_without_training(self):
        calls = []
        def dispatch(args):
            calls.append(args)
            return 0
        self.assertEqual(train.main(["--help"], run_cli=dispatch), 0)
        self.assertEqual(calls, [["--rl_library", "rsl_rl", "--help"]])


if __name__ == "__main__":
    unittest.main()
