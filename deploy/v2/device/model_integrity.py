"""检查ONNX主文件及图中所有外部张量文件；必须在打开设备前调用。"""
import hashlib
from pathlib import Path, PurePosixPath, PureWindowsPath
import re


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _messages(message):
    """遍历公开protobuf字段，覆盖子图、属性张量、稀疏张量等嵌套。"""
    yield message
    for field, value in message.ListFields():
        if field.type == field.TYPE_MESSAGE:
            repeated = field.is_repeated if hasattr(field, 'is_repeated') else field.label == field.LABEL_REPEATED
            for child in value if repeated else (value,):
                yield from _messages(child)


def verify_model_files(model, contract):
    import onnx
    model = Path(model).resolve(strict=True)
    expected_main = contract.get('onnx_main_file_sha256')
    expected = contract.get('external_data_sha256')
    valid_hash = lambda value: isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None
    if not valid_hash(expected_main) or not isinstance(expected, dict) or any(
            not isinstance(k, str) or not valid_hash(v) for k, v in expected.items()):
        raise ValueError('model_hash_manifest_required')
    if file_sha256(model) != expected_main:
        raise ValueError('onnx_hash_mismatch')
    graph = onnx.load_model(str(model), load_external_data=False)
    locations = set()
    for tensor in _messages(graph):
        if not isinstance(tensor, onnx.TensorProto):
            continue
        if tensor.data_location != onnx.TensorProto.EXTERNAL and not tensor.external_data:
            continue
        found = [entry.value for entry in tensor.external_data if entry.key == 'location']
        if len(found) != 1 or not found[0]:
            raise ValueError('invalid_external_data_location')
        location = found[0]
        path = PurePosixPath(location)
        # ONNX外部文件必须在模型目录内；拒绝平台歧义、绝对路径和目录越界。
        if ('\\' in location or PureWindowsPath(location).drive or path.is_absolute()
                or '..' in path.parts or path.as_posix() != location):
            raise ValueError('unsafe_external_data_location')
        locations.add(location)
    if locations != set(expected):
        raise ValueError('external_data_manifest_mismatch')
    result = {'onnx_main_file_sha256': expected_main, 'external_data_sha256': {}}
    for location in sorted(locations):
        path = (model.parent / location).resolve(strict=True)
        if not path.is_relative_to(model.parent) or not path.is_file():
            raise ValueError('external_data_outside_model_directory')
        actual = file_sha256(path)
        if actual != expected[location]:
            raise ValueError('external_data_hash_mismatch:' + location)
        result['external_data_sha256'][location] = actual
    return result

