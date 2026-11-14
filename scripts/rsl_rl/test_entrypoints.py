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
                args = ["--task", "Hexchip-Cartpole-Real-V2-v0", "--viz", "none", "env.seed=42"]
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

    def test_play_forwards_checkpoint_and_visualizer(self):
        calls = []
        def dispatch(args):
            calls.append(args)
            return 0
        args = ["--task", "Hexchip-Cartpole-Real-V2-v0", "--checkpoint", "model.pt", "--viz", "kit"]
        self.assertEqual(play.main(args, run_cli=dispatch), 0)
        self.assertEqual(calls, [["--rl_library", "rsl_rl", *args]])

    def test_unknown_options_are_not_silently_translated(self):
        calls = []
        def dispatch(args):
            calls.append(args)
            return 2
        self.assertEqual(train.main(["--headless"], run_cli=dispatch), 2)
        self.assertEqual(calls, [["--rl_library", "rsl_rl", "--headless"]])


if __name__ == "__main__":
    unittest.main()
