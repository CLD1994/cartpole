#!/usr/bin/env python3
"""从正式 ONNX 模型目录和厂商工具生成的 OM 准备端侧模型包。

输入 model.json 为训练事实的唯一来源。先校验 ONNX 及外部权重散列，
然后复制 OM 并绑定其 SHA256。不会声称 OM 与 ONNX 数值等价；板端推理
对照仍须独立执行。输出目录必须不存在，失败清理本次创建的目录。
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


def render_manifest(manifest, digest):
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
    return (f'EDGE_PACKAGE_V1\nmodel=policy.om\nsha256={digest}\ncontrol_hz={hz}\n'
            f'action_scale_mm_s={speed}\nvelocity_limit_mm_s={speed}\n'
            f'acceleration_mm_s2={acceleration}\nangle_sign=-1\npause_deg=20\nresume_deg=18\n')


def prepare(model_directory, om, destination):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'device'))
    from model_manifest import load_manifest
    model_directory, om, destination = map(Path, (model_directory, om, destination))
    manifest = load_manifest(model_directory / 'model.json', model_directory / 'policy.onnx')
    # 对应运行器真实输入是4项SI值；窗口在测量端，不是外部16帧输入。
    import onnx
    graph = onnx.load(str(model_directory / 'policy.onnx'), load_external_data=False)
    inputs = [i for i in graph.graph.input if i.name not in {v.name for v in graph.graph.initializer}]
    def shape(value):
        return [d.dim_value for d in value.type.tensor_type.shape.dim]
    if (len(inputs) != 1 or len(graph.graph.output) != 1 or shape(inputs[0]) != [1, 4]
            or shape(graph.graph.output[0]) != [1, 1]
            or inputs[0].type.tensor_type.elem_type != onnx.TensorProto.FLOAT
            or graph.graph.output[0].type.tensor_type.elem_type != onnx.TensorProto.FLOAT):
        raise ValueError('model_shape_requires_float32_1x4_to_1x1')
    if not om.is_file() or om.stat().st_size == 0:
        raise ValueError('om_missing_or_empty')
    destination.mkdir(parents=False, exist_ok=False)
    try:
        copied = destination / 'policy.om'
        shutil.copyfile(om, copied)
        with copied.open('rb') as stream:
            digest = hashlib.file_digest(stream, 'sha256').hexdigest()
        (destination / 'edge-package.txt').write_bytes(render_manifest(manifest, digest).encode('ascii'))
        # 来源仅供追踪，不作为运行器额外配置。
        (destination / 'source-model.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    except BaseException:
        shutil.rmtree(destination)
        raise
    return destination / 'edge-package.txt'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True, type=Path, help='正式 policy.onnx/model.json 目录')
    parser.add_argument('--om', required=True, type=Path, help='由该 ONNX 经目标 SDK 转换得到的 OM')
    parser.add_argument('--output', required=True, type=Path, help='新建的端侧模型包目录')
    args = parser.parse_args()
    print(prepare(args.model, args.om, args.output))
    print('模型包已生成；仍需目标板验证 OM 数值、形状与推理耗时。')

if __name__ == '__main__': main()
