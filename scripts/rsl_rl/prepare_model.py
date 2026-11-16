"""将已导出的 ONNX 与保存的训练配置整理为可运行模型目录，不启动仿真。"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath, PureWindowsPath
import sys

_DEVICE = Path(__file__).resolve().parents[2] / 'deploy' / 'v2' / 'device'
sys.path.insert(0, str(_DEVICE))
from model_integrity import _messages, file_sha256, verify_model_files
from model_manifest import training_contract, validate_manifest


def read_training_yaml(text):
    """只解析数据；允许 Isaac 保存的 tuple，不实例化 Python 对象。"""
    import yaml
    class Loader(yaml.SafeLoader):
        pass
    Loader.add_constructor('tag:yaml.org,2002:python/tuple',
                           lambda loader, node: loader.construct_sequence(node))
    Loader.add_constructor('tag:yaml.org,2002:python/object/apply:builtins.slice',
                           lambda loader, node: {'slice': loader.construct_sequence(node)})
    try:
        result = yaml.load(text, Loader=Loader)
    except yaml.YAMLError as exc:
        raise ValueError('training_yaml_invalid:' + str(exc)) from exc
    if not isinstance(result, dict):
        raise ValueError('training_yaml_mapping_required')
    return result


def validate_policy_shape(inputs, outputs):
    if (len(inputs) != 1 or len(outputs) != 1 or len(inputs[0]) != 2
            or len(outputs[0]) != 2 or inputs[0][1] != 4 or outputs[0][1] != 1
            or inputs[0][0] not in (None, 1) or outputs[0][0] not in (None, 1)):
        raise ValueError('onnx_single_action_shape_required')


def inspect_model(path):
    import onnx
    path = path.resolve(strict=True)
    graph = onnx.load_model(str(path), load_external_data=False)
    def shape(value):
        tensor = value.type.tensor_type
        if tensor.elem_type != onnx.TensorProto.FLOAT:
            raise ValueError('onnx_float32_required')
        return [dim.dim_value if dim.HasField('dim_value') else None for dim in tensor.shape.dim]
    validate_policy_shape([shape(v) for v in graph.graph.input], [shape(v) for v in graph.graph.output])
    external = {}
    for tensor in _messages(graph):
        if not isinstance(tensor, onnx.TensorProto) or not tensor.external_data:
            continue
        locations = [entry.value for entry in tensor.external_data if entry.key == 'location']
        if len(locations) != 1:
            raise ValueError('invalid_external_data_location')
        location = locations[0]
        relative = PurePosixPath(location)
        if (not location or '\\' in location or PureWindowsPath(location).drive
                or relative.is_absolute() or '..' in relative.parts or relative.as_posix() != location):
            raise ValueError('unsafe_external_data_location')
        target = (path.parent / location).resolve(strict=True)
        if not target.is_relative_to(path.parent):
            raise ValueError('external_data_outside_model_directory')
        external[location] = file_sha256(target)
    result = {'onnx_main_file_sha256': file_sha256(path), 'external_data_sha256': external}
    verify_model_files(path, result)
    return result


def prepare(onnx_path, checkpoint, env_path):
    """不推断 ONNX 与 checkpoint 同源；记录操作者明确指定的关联。"""
    raw = env_path.read_bytes()
    contract = training_contract(read_training_yaml(raw.decode('utf-8-sig')))
    result = {'schema_version': 1, **contract, **inspect_model(onnx_path),
              'training_source': {'env_yaml_sha256': hashlib.sha256(raw).hexdigest(),
                                  'checkpoint_sha256': file_sha256(checkpoint),
                                  'checkpoint_name': checkpoint.name,
                                  'association': 'operator_selected_checkpoint_and_onnx'}}
    return validate_manifest(result)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--onnx', required=True, type=Path, help='已导出的 policy.onnx')
    parser.add_argument('--checkpoint', required=True, type=Path, help='操作者确认该 ONNX 对应的检查点')
    parser.add_argument('--training-config', type=Path, help='默认从检查点训练目录读取 params/env.yaml')
    args = parser.parse_args(argv)
    try:
        checkpoint = args.checkpoint.resolve(strict=True)
        model = args.onnx.resolve(strict=True)
        if model.name != 'policy.onnx':
            raise ValueError('model_filename_must_be_policy.onnx')
        root = checkpoint.parent.parent if checkpoint.parent.name == 'training-candidates' else checkpoint.parent
        env = args.training_config or root / 'params' / 'env.yaml'
        manifest = prepare(model, checkpoint, env)
        target = model.parent / 'model.json'
        # A failed validation leaves any previous manifest untouched.
        temporary = target.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        temporary.replace(target)
        print(f"模型准备完成：{model.parent}")
        print(f"单动作 · {manifest['training_control_hz']} Hz · 加速度 {manifest['training_acceleration_m_s2']:g} m/s²")
        print('来源关联：使用指定检查点和 ONNX；哈希记录不独立证明二者同源。')
        return 0
    except (ValueError, OSError, ImportError) as exc:
        print(f'模型准备失败：{exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
