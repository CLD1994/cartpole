"""CPU 后端真实权重文件加载及推理集成测试。"""
import ctypes, pathlib, struct, subprocess, tempfile, unittest
import importlib.util
import sys
import numpy as np
EDGE=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(EDGE / 'tests' / 'support'))
from onnx_fixture import policy_graph
class CPUModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory(); cls.root=pathlib.Path(cls.tmp.name)
        subprocess.run(['cc','-shared','-fPIC','-std=c11','-O2','-ffp-contract=off','-I'+str(EDGE),str(EDGE/'cpu_policy.c'),str(EDGE/'policy.c'),str(EDGE/'tests/support/policy_enum_bridge.c'),'-lm','-o',str(cls.root/'cpu.so')],check=True)
        cls.lib=ctypes.CDLL(str(cls.root/'cpu.so'))
        cls.cpu_backend = cls.lib.test_policy_cpu_backend()
        cls.npu_backend = cls.lib.test_policy_npu_backend()
        cls.lib.policy_init.argtypes=[ctypes.c_int,ctypes.c_char_p]
        cls.lib.policy_forward.argtypes=[ctypes.POINTER(ctypes.c_float),ctypes.POINTER(ctypes.c_float)]
        cls.lib.cpu_policy_init.argtypes=[ctypes.c_char_p]
        cls.lib.cpu_policy_forward.argtypes=[ctypes.POINTER(ctypes.c_float),ctypes.POINTER(ctypes.c_float)]
    @classmethod
    def tearDownClass(cls): cls.tmp.cleanup()
    def tearDown(self): self.lib.policy_deinit(); self.lib.cpu_policy_deinit()
    def blob(self):
        return b'CPFP32V1'+struct.pack('<4I',4,1,1,1)+struct.pack('<9f',1,0,0,0,0,1,0,1,0)
    def load(self,b):
        p=self.root/'model'; p.write_bytes(b); return self.lib.cpu_policy_init(str(p).encode())
    def test_exported_onnx_parameters_execute_in_real_cpu_backend(self):
        # 真实 Python 导出器 → 参数文件 → C 调度器/CPU 后端；不替换组件。
        spec = importlib.util.spec_from_file_location('cpu_export_integration', EDGE/'cpu_export.py')
        exporter = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(exporter)
        model, arrays = policy_graph()
        path = self.root/'exported.fp32'
        path.write_bytes(exporter.export_cpu(model))
        self.assertEqual(self.lib.policy_init(self.cpu_backend, str(path).encode()), 0)
        for values in [(1, 2, 3, 4), (-1, -2, -3, -4), (.2, -.3, .4, -.5)]:
            with self.subTest(observation=values):
                expected = np.array(values, dtype=np.float32)
                for layer in range(3):
                    expected = arrays[layer*2] @ expected + arrays[layer*2+1]
                    if layer < 2:
                        expected = np.where(expected < 0, np.expm1(expected), expected)
                result = ctypes.c_float()
                self.assertEqual(self.lib.policy_forward((ctypes.c_float*4)(*values), ctypes.byref(result)), 0)
                self.assertAlmostEqual(result.value, float(expected[0]), places=5)

    def test_positive_forward(self):
        self.assertEqual(self.load(self.blob()),0)
        out=ctypes.c_float(); self.assertEqual(self.lib.cpu_policy_forward((ctypes.c_float*4)(2,0,0,0),ctypes.byref(out)),0); self.assertEqual(out.value,2)
    def test_elu_layers(self):
        import math
        self.assertEqual(self.load(self.blob()),0)
        out=ctypes.c_float(); self.lib.cpu_policy_forward((ctypes.c_float*4)(-1,0,0,0),ctypes.byref(out)); self.assertAlmostEqual(out.value,math.expm1(math.expm1(-1)),places=6)
    def test_invalid_files(self):
        for data in [b'',self.blob()[:-1],self.blob()+b'x',b'INVALID!'+self.blob()[8:],self.blob()[:12]+struct.pack('<I',257)+self.blob()[16:],self.blob()[:-4]+struct.pack('<f',float('nan'))]:
            with self.subTest(data=len(data)): self.assertNotEqual(self.load(data),0)
    def test_nonfinite_input_rejected(self):
        self.assertEqual(self.load(self.blob()),0); out=ctypes.c_float(3)
        self.assertNotEqual(self.lib.cpu_policy_forward((ctypes.c_float*4)(float('nan'),0,0,0),ctypes.byref(out)),0); self.assertEqual(out.value,0)
    def test_cpu_dispatch_without_sdk(self):
        p=self.root/'dispatch'; p.write_bytes(self.blob())
        self.assertEqual(self.lib.policy_init(self.cpu_backend,str(p).encode()),0)
        out=ctypes.c_float(); self.assertEqual(self.lib.policy_forward((ctypes.c_float*4)(2,0,0,0),ctypes.byref(out)),0)
        self.assertEqual(out.value,2)
        self.assertNotEqual(self.lib.policy_init(self.npu_backend,b'absent.om'),0)
    def test_npu_unavailable_no_fallback(self):
        self.assertNotEqual(self.lib.policy_init(self.npu_backend,b'absent.om'),0)
        out=ctypes.c_float(7); self.assertNotEqual(self.lib.policy_forward((ctypes.c_float*4)(2,0,0,0),ctypes.byref(out)),0)
        self.assertEqual(out.value,0)
    def test_arbitrary_hidden_dimensions(self):
        import math, random
        rng=random.Random(713)
        dims=[4,3,2,1]; data=b'CPFP32V1'+struct.pack('<4I',*dims); layers=[]
        for ni,no in zip(dims,dims[1:]):
            w=[[rng.uniform(-.5,.5) for _ in range(ni)] for _ in range(no)]; b=[rng.uniform(-.5,.5) for _ in range(no)]
            data+=struct.pack('<'+'f'*(ni*no+no),*[v for row in w for v in row],*b); layers.append((w,b))
        self.assertEqual(self.load(data),0)
        for _ in range(20):
            x=[rng.uniform(-1,1) for _ in range(4)]; expected=x
            for i,(w,b) in enumerate(layers):
                expected=[sum(a*v for a,v in zip(row,expected))+bias for row,bias in zip(w,b)]
                if i<2: expected=[math.expm1(v) if v<0 else v for v in expected]
            out=ctypes.c_float(); self.assertEqual(self.lib.cpu_policy_forward((ctypes.c_float*4)(*x),ctypes.byref(out)),0)
            self.assertAlmostEqual(out.value,expected[0],places=6)
if __name__=='__main__': unittest.main()
