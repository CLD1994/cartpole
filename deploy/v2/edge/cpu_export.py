"""将已验证的三层 Gemm/ELU 策略导出为 CPFP32V1 小端参数。"""
import struct
import numpy as np
from onnx import helper, numpy_helper, TensorProto, checker


def export_cpu(model):
    graph = model.graph
    nodes = list(graph.node)
    if [n.op_type for n in nodes] != ['Gemm', 'Elu', 'Gemm', 'Elu', 'Gemm'] or any(n.domain not in ('', 'ai.onnx') for n in nodes):
        raise ValueError('unsupported_cpu_architecture')
    if len(graph.input) != 1 or len(graph.output) != 1:
        raise ValueError('model_shape')
    for value, expected in ((graph.input[0], [1, 4]), (graph.output[0], [1, 1])):
        if value.type.tensor_type.elem_type != TensorProto.FLOAT or [d.dim_value for d in value.type.tensor_type.shape.dim] != expected:
            raise ValueError('model_shape')
    tensors = {t.name: numpy_helper.to_array(t) for t in graph.initializer}
    if len(tensors) != len(graph.initializer):
        raise ValueError('duplicate_initializer')
    previous = graph.input[0].name
    arrays, widths, used = [], [4], set()
    for index, node in enumerate(nodes):
        if not node.input or node.input[0] != previous or len(node.output) != 1:
            raise ValueError('unsupported_cpu_chain')
        attrs = {a.name: helper.get_attribute_value(a) for a in node.attribute}
        if node.op_type == 'Gemm':
            defaults = dict(transA=0, transB=1, alpha=1.0, beta=1.0)
            # ONNX transB defaults to zero, not the supported transpose.
            if len(node.input) != 3 or attrs.get('transB', 0) != 1 or any(k not in defaults or v != defaults[k] for k, v in attrs.items()):
                raise ValueError('unsupported_cpu_gemm_attributes')
            if any(name not in tensors for name in node.input[1:]):
                raise ValueError('cpu_constant_weights_required')
            w, b = (tensors[name] for name in node.input[1:])
            if w.dtype != np.float32 or b.dtype != np.float32 or w.ndim != 2 or b.shape != (w.shape[0],) or w.shape[1] != widths[-1]:
                raise ValueError('cpu_weight_shape')
            if not 1 <= w.shape[0] <= 256 or (index == 4 and w.shape[0] != 1):
                raise ValueError('cpu_hidden_width')
            if not np.isfinite(w).all() or not np.isfinite(b).all():
                raise ValueError('cpu_weights_not_finite')
            widths.append(w.shape[0]); arrays.extend((w, b)); used.update(node.input[1:])
        elif len(node.input) != 1 or any(k != 'alpha' or v != 1.0 for k, v in attrs.items()):
            raise ValueError('unsupported_cpu_elu_attributes')
        previous = node.output[0]
    if previous != graph.output[0].name or used != set(tensors):
        raise ValueError('unsupported_cpu_chain')
    checker.check_model(model)
    return b'CPFP32V1' + struct.pack('<4I', *widths) + b''.join(a.astype('<f4').tobytes(order='C') for a in arrays)
