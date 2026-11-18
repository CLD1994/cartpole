"""纯内存 ONNX 图到 CPU 参数协议的契约测试。"""
import importlib.util
from pathlib import Path
import struct
import unittest
import numpy as np
from onnx import helper, numpy_helper, TensorProto
spec=importlib.util.spec_from_file_location('cpu_export',Path(__file__).parents[2]/'cpu_export.py')

def graph():
    arrays=[np.arange(12,dtype=np.float32).reshape(3,4)/10,np.zeros(3,np.float32),np.ones((2,3),np.float32),np.zeros(2,np.float32),np.ones((1,2),np.float32),np.ones(1,np.float32)]
    names=['w0','b0','w1','b1','w2','b2']
    nodes=[helper.make_node('Gemm',['obs','w0','b0'],['a'],transB=1),helper.make_node('Elu',['a'],['b']),helper.make_node('Gemm',['b','w1','b1'],['c'],transB=1),helper.make_node('Elu',['c'],['d']),helper.make_node('Gemm',['d','w2','b2'],['action'],transB=1)]
    g=helper.make_graph(nodes,'test',[helper.make_tensor_value_info('obs',TensorProto.FLOAT,[1,4])],[helper.make_tensor_value_info('action',TensorProto.FLOAT,[1,1])],[numpy_helper.from_array(a,n) for n,a in zip(names,arrays)])
    return helper.make_model(g),arrays

class ExportTest(unittest.TestCase):
    def export(self,g):
        m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
        return m.export_cpu(g)
    def test_binary_preserves_parameters_and_variable_widths(self):
        g,a=graph(); data=self.export(g)
        self.assertEqual(data[:24],b'CPFP32V1'+struct.pack('<4I',4,3,2,1))
        np.testing.assert_array_equal(np.frombuffer(data[24:],dtype='<f4'),np.concatenate([v.ravel() for v in a]))
    def test_reject_wrong_transpose(self):
        g,_=graph();g.graph.node[0].attribute[0].i=0
        with self.assertRaisesRegex(ValueError,'gemm'):self.export(g)
    def test_reject_changed_activation(self):
        g,_=graph();g.graph.node[1].op_type='Relu'
        with self.assertRaisesRegex(ValueError,'architecture'):self.export(g)
    def test_reject_rewired_graph(self):
        g,_=graph();g.graph.node[2].input[0]='obs'
        with self.assertRaisesRegex(ValueError,'chain'):self.export(g)
    def test_reject_nonfinite(self):
        g,_=graph();g.graph.initializer[1].CopyFrom(numpy_helper.from_array(np.full(3,np.nan,np.float32),'b0'))
        with self.assertRaisesRegex(ValueError,'finite'):self.export(g)

    def test_reject_changed_elu_alpha(self):
        g,_=graph();g.graph.node[1].attribute.append(helper.make_attribute('alpha',2.0))
        with self.assertRaisesRegex(ValueError,'elu'):self.export(g)
    def test_reject_extra_graph_operation(self):
        g,_=graph();g.graph.node.append(helper.make_node('Identity',['action'],['unused']))
        with self.assertRaisesRegex(ValueError,'architecture'):self.export(g)
    def test_reject_unsupported_hidden_width(self):
        g,_=graph();g.graph.initializer[0].CopyFrom(numpy_helper.from_array(np.zeros((257,4),np.float32),'w0'));g.graph.initializer[1].CopyFrom(numpy_helper.from_array(np.zeros(257,np.float32),'b0'))
        with self.assertRaisesRegex(ValueError,'width'):self.export(g)
    def test_reject_non_float32_weights(self):
        g,_=graph();g.graph.initializer[0].CopyFrom(numpy_helper.from_array(np.zeros((3,4),np.float64),'w0'))
        with self.assertRaisesRegex(ValueError,'shape'):self.export(g)
    def test_reject_nonunit_gemm_scale(self):
        g,_=graph();g.graph.node[0].attribute.append(helper.make_attribute('alpha',2.0))
        with self.assertRaisesRegex(ValueError,'gemm'):self.export(g)

if __name__ == "__main__": unittest.main()
