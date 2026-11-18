#!/usr/bin/env python3
"""从正式 ONNX 模型目录准备 CPU FP32 包，可选加入经厂商转换的 NPU OM。

校验 model.json 与 ONNX 来源，严格导出支持的三层 Gemm/ELU CPU 参数。
OM 与 ONNX 的数值等价性仍须板端独立验证；输出目录必须不存在。
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import sys


def render_manifest(manifest, digest, npu_digest=None):
    def milli(key):
        value = manifest[key] * 1000
        if not math.isfinite(value) or value <= 0 or not math.isclose(value, round(value), rel_tol=0, abs_tol=1e-8):
            raise ValueError('firmware_unit_precision:' + key)
        if value > 2147483647:
            raise ValueError('firmware_unit_range:' + key)
        return round(value)
    speed = milli('action_scale_m_s')
    acceleration = milli('training_acceleration_m_s2')
    hz = manifest['training_control_hz']
    if type(hz) is not int or hz not in (400, 500):
        raise ValueError('unsupported_frequency')
    if not isinstance(digest, str) or re.fullmatch('[0-9a-f]{64}', digest) is None:
        raise ValueError('invalid_sha256')
    if npu_digest is not None and re.fullmatch('[0-9a-f]{64}', npu_digest) is None:
        raise ValueError('invalid_npu_sha256')
    return (f'EDGE_PACKAGE_V2\nmodel_cpu=policy.fp32\nmodel_npu={"policy.om" if npu_digest else "none"}\n'
            f'sha256_cpu={digest}\nsha256_npu={npu_digest or "none"}\ncontrol_hz={hz}\n'
            f'action_scale_mm_s={speed}\nvelocity_limit_mm_s={speed}\n'
            f'acceleration_mm_s2={acceleration}\nangle_sign=-1\npause_deg=20\nresume_deg=18\n')


def prepare(model_directory, om, destination):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'device'))
    from model_manifest import load_manifest
    model_directory, destination = map(Path, (model_directory, destination))
    om = Path(om) if om is not None else None
    manifest = load_manifest(model_directory / 'model.json', model_directory / 'policy.onnx')
    # 对应运行器真实输入是4项SI值；窗口在测量端，不是外部16帧输入。
    import onnx
    graph = onnx.load(str(model_directory / 'policy.onnx'), load_external_data=True)
    inputs = [i for i in graph.graph.input if i.name not in {v.name for v in graph.graph.initializer}]
    def shape(value):
        return [d.dim_value for d in value.type.tensor_type.shape.dim]
    if (len(inputs) != 1 or len(graph.graph.output) != 1 or shape(inputs[0]) != [1, 4]
            or shape(graph.graph.output[0]) != [1, 1]
            or inputs[0].type.tensor_type.elem_type != onnx.TensorProto.FLOAT
            or graph.graph.output[0].type.tensor_type.elem_type != onnx.TensorProto.FLOAT):
        raise ValueError('model_shape_requires_float32_1x4_to_1x1')
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from cpu_export import export_cpu
    cpu_data = export_cpu(graph)
    if om is not None and (not om.is_file() or om.stat().st_size == 0):
        raise ValueError('om_missing_or_empty')
    destination.mkdir(parents=False, exist_ok=False)
    try:
        (destination / 'policy.fp32').write_bytes(cpu_data)
        digest = hashlib.sha256(cpu_data).hexdigest()
        npu_digest = None
        if om is not None:
            copied = destination / 'policy.om'
            shutil.copyfile(om, copied)
            with copied.open('rb') as stream:
                npu_digest = hashlib.file_digest(stream, 'sha256').hexdigest()
        (destination / 'edge-package.txt').write_bytes(render_manifest(manifest, digest, npu_digest).encode('ascii'))
        # 来源仅供追踪，不作为运行器额外配置。
        (destination / 'source-model.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    except BaseException:
        shutil.rmtree(destination)
        raise
    return destination / 'edge-package.txt'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True, type=Path, help='正式 policy.onnx/model.json 目录')
    parser.add_argument('--om', type=Path, help='由该 ONNX 经目标 SDK 转换得到的 OM')
    parser.add_argument('--output', required=True, type=Path, help='新建的端侧模型包目录')
    args = parser.parse_args()
    print(prepare(args.model, args.om, args.output))
    print('模型包已生成；正式运行前需目标板验证所选后端的数值与推理耗时。')

if __name__ == '__main__': main()
