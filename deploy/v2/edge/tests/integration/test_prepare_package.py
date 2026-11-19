"""模型文件、ONNX读取与包输出的集成测试；不需要NPU。"""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import sys
import unittest
from unittest.mock import patch
import onnx
EDGE = Path(__file__).parents[2]
sys.path.insert(0, str(EDGE / 'tests' / 'support'))
from onnx_fixture import policy_graph
spec = importlib.util.spec_from_file_location('prepare_package_integration', EDGE/'prepare_package.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

class PrepareTest(unittest.TestCase):
    def fixture(self, root, shape=(1,4)):
        model, _ = policy_graph(shape)
        onnx.save(model, root/'policy.onnx')
        manifest = dict(schema_version=1, observation_order=['x','theta','xdot','thetadot'],
            observation_units=['m','rad','m/s','rad/s'],action_type='cart_target_velocity',action_dim=1,
            action_scale_m_s=.9,training_control_hz=400,training_acceleration_m_s2=10,
            training_source=dict(association='operator_selected_checkpoint_and_onnx',checkpoint_name='candidate.pt',
                                 env_yaml_sha256='a'*64,checkpoint_sha256='b'*64),
            onnx_main_file_sha256=hashlib.sha256((root/'policy.onnx').read_bytes()).hexdigest(),external_data_sha256={})
        (root/'model.json').write_text(json.dumps(manifest))
        (root/'source.om').write_bytes(b'om-fixture')

    def test_prepare_copies_om_and_binds_facts(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); self.fixture(root)
            result=m.prepare(root,root/'source.om',root/'edge')
            self.assertEqual((result.parent/'policy.om').read_bytes(),b'om-fixture')
            self.assertIn(hashlib.sha256(b'om-fixture').hexdigest(),result.read_text())
            self.assertEqual(json.loads(result.read_text())['control_hz'],400)
            self.assertEqual(result.name,'edge-package.json')
            self.assertFalse((result.parent/'edge-package.txt').exists())
            self.assertTrue((result.parent/'source-model.json').is_file())
            with self.assertRaises(FileExistsError): m.prepare(root,root/'source.om',root/'edge')

    def test_manifest_json_survives_windows_text_translation(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d); self.fixture(root)
            original = Path.write_text
            def windows_write_text(path, data, *args, **kwargs):
                # Windows text-mode newline conversion, isolated at filesystem boundary.
                return original(path, data.replace('\n', '\r\n'), *args, **kwargs)
            with patch.object(Path, 'write_text', windows_write_text):
                result = m.prepare(root, root/'source.om', root/'edge')
            raw = result.read_bytes()
            self.assertEqual(json.loads(raw)['schema_version'],2)
            self.assertEqual(json.loads(raw.replace(b'\n', b'\r\n'))['control_hz'],400)

    def test_cpu_only_package(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); self.fixture(root)
            result=m.prepare(root,None,root/'cpu')
            self.assertIsNone(json.loads(result.read_text())['model_npu'])
            self.assertIsNone(json.loads(result.read_text())['sha256_npu'])
            self.assertTrue((result.parent/'policy.fp32').read_bytes().startswith(b'CPFP32V1'))
            self.assertFalse((result.parent/'policy.om').exists())

    def test_bad_onnx_hash_creates_no_output(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); self.fixture(root)
            (root/'policy.onnx').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'onnx_hash_mismatch'): m.prepare(root,root/'source.om',root/'edge')
            self.assertFalse((root/'edge').exists())

    def test_wrong_observation_shape_creates_no_output(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); self.fixture(root,(1,64))
            with self.assertRaises(ValueError) as caught:
                m.prepare(root,root/'source.om',root/'edge')
            self.assertEqual(str(caught.exception), m.ERROR_MODEL_SHAPE_REQUIRES_FLOAT32_1X4_TO_1X1)
            self.assertFalse((root/'edge').exists())

if __name__ == '__main__': unittest.main()
