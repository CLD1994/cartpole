"""导出与包测试共用的三层 Gemm/ELU 模型；每次返回独立图和参数。"""
import numpy as np
from onnx import helper, numpy_helper, TensorProto


def policy_graph(input_shape=(1, 4)):
    arrays = [np.arange(12, dtype=np.float32).reshape(3, 4) / 10,
              np.zeros(3, np.float32), np.ones((2, 3), np.float32),
              np.zeros(2, np.float32), np.ones((1, 2), np.float32),
              np.ones(1, np.float32)]
    names = ['w0', 'b0', 'w1', 'b1', 'w2', 'b2']
    nodes = [helper.make_node('Gemm', ['obs', 'w0', 'b0'], ['a'], transB=1),
             helper.make_node('Elu', ['a'], ['b']),
             helper.make_node('Gemm', ['b', 'w1', 'b1'], ['c'], transB=1),
             helper.make_node('Elu', ['c'], ['d']),
             helper.make_node('Gemm', ['d', 'w2', 'b2'], ['action'], transB=1)]
    graph = helper.make_graph(nodes, 'policy',
        [helper.make_tensor_value_info('obs', TensorProto.FLOAT, input_shape)],
        [helper.make_tensor_value_info('action', TensorProto.FLOAT, [1, 1])],
        [numpy_helper.from_array(a, n) for n, a in zip(names, arrays)])
    return helper.make_model(graph), arrays
