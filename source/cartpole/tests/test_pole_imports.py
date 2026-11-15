"""配置和诊断脚本可在 Kit 启动前加载，不触碰仿真或 USD 执行依赖。"""
import builtins
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch


class PoleImportTests(unittest.TestCase):
    def test_configuration_spawner_import_does_not_load_runtime(self):
        source = Path(__file__).parents[1] / "cartpole/tasks/manager_based/cartpole/pole_spawner.py"
        self.check_import(source)

    def test_diagnostic_script_import_does_not_load_runtime(self):
        self.check_import(Path(__file__).parents[3] / "scripts/check_pole_physics.py")

    def check_import(self, source):
        original = builtins.__import__
        def guarded(name, *args, **kwargs):
            if name.split(".")[0] in ("pxr", "isaaclab", "isaaclab_physx", "cartpole", "omni"):
                raise AssertionError(f"配置/脚本加载提前导入运行时模块：{name}")
            return original(name, *args, **kwargs)
        spec = importlib.util.spec_from_file_location("pole_import_contract", source)
        module = importlib.util.module_from_spec(spec)
        with patch("builtins.__import__", guarded):
            spec.loader.exec_module(module)


if __name__ == "__main__":
    unittest.main()
