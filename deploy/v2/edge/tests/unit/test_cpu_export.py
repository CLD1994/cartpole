"""纯内存 ONNX 图到 CPU 参数协议的契约测试。"""
import importlib.util
from pathlib import Path
import struct
import sys
import unittest
import numpy as np
from onnx import helper, numpy_helper, TensorProto
spec=importlib.util.spec_from_file_location('cpu_export',Path(__file__).parents[2]/'cpu_export.py')
m=importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

sys.path.insert(0, str(Path(__file__).parents[1] / 'support'))
from onnx_fixture import policy_graph as graph

class ExportTest(unittest.TestCase):
    def export(self,g):
        return m.export_cpu(g)
    def test_binary_preserves_parameters_and_variable_widths(self):
        g,a=graph(); data=self.export(g)
        self.assertEqual(data[:24],b'CPFP32V1'+struct.pack('<4I',4,3,2,1))
        np.testing.assert_array_equal(np.frombuffer(data[24:],dtype='<f4'),np.concatenate([v.ravel() for v in a]))
    def test_reject_wrong_transpose(self):
        g,_=graph();g.graph.node[0].attribute[0].i=0
        with self.assertRaises(ValueError) as caught:
            self.export(g)
        self.assertEqual(str(caught.exception), m.ERROR_UNSUPPORTED_CPU_GEMM_ATTRIBUTES)
    def test_reject_changed_activation(self):
        g,_=graph();g.graph.node[1].op_type='Relu'
        with self.assertRaises(ValueError) as caught:
            self.export(g)
        self.assertEqual(str(caught.exception), m.ERROR_UNSUPPORTED_CPU_ARCHITECTURE)
    def test_reject_rewired_graph(self):
        g,_=graph();g.graph.node[2].input[0]='obs'
        with self.assertRaises(ValueError) as caught:
            self.export(g)
        self.assertEqual(str(caught.exception), m.ERROR_UNSUPPORTED_CPU_CHAIN)
    def test_reject_nonfinite(self):
        g,_=graph();g.graph.initializer[1].CopyFrom(numpy_helper.from_array(np.full(3,np.nan,np.float32),'b0'))
        with self.assertRaises(ValueError) as caught:
            self.export(g)
        self.assertEqual(str(caught.exception), m.ERROR_CPU_WEIGHTS_NOT_FINITE)

    def test_reject_changed_elu_alpha(self):
        g,_=graph();g.graph.node[1].attribute.append(helper.make_attribute('alpha',2.0))
        with self.assertRaises(ValueError) as caught:
            self.export(g)
        self.assertEqual(str(caught.exception), m.ERROR_UNSUPPORTED_CPU_ELU_ATTRIBUTES)
    def test_reject_extra_graph_operation(self):
        g,_=graph();g.graph.node.append(helper.make_node('Identity',['action'],['unused']))
        with self.assertRaises(ValueError) as caught:
            self.export(g)
        self.assertEqual(str(caught.exception), m.ERROR_UNSUPPORTED_CPU_ARCHITECTURE)
    def test_reject_unsupported_hidden_width(self):
        g,_=graph();g.graph.initializer[0].CopyFrom(numpy_helper.from_array(np.zeros((257,4),np.float32),'w0'));g.graph.initializer[1].CopyFrom(numpy_helper.from_array(np.zeros(257,np.float32),'b0'))
        with self.assertRaises(ValueError) as caught:
            self.export(g)
        self.assertEqual(str(caught.exception), m.ERROR_CPU_HIDDEN_WIDTH)
    def test_reject_non_float32_weights(self):
        g,_=graph();g.graph.initializer[0].CopyFrom(numpy_helper.from_array(np.zeros((3,4),np.float64),'w0'))
        with self.assertRaises(ValueError) as caught:
            self.export(g)
        self.assertEqual(str(caught.exception), m.ERROR_CPU_WEIGHT_SHAPE)
    def test_reject_nonunit_gemm_scale(self):
        g,_=graph();g.graph.node[0].attribute.append(helper.make_attribute('alpha',2.0))
        with self.assertRaises(ValueError) as caught:
            self.export(g)
        self.assertEqual(str(caught.exception), m.ERROR_UNSUPPORTED_CPU_GEMM_ATTRIBUTES)

if __name__ == "__main__": unittest.main()
