"""可复制到 IsaacLab 主机的原生策略客户端，仅依赖 NumPy 和标准库。"""
from enum import Enum
import hashlib
from datetime import datetime, timezone
from http.client import HTTPConnection
from io import BytesIO
import time
from urllib.error import HTTPError
import json
import math
from pathlib import Path
from urllib.parse import urlsplit
import uuid

import numpy as np


PROTOCOL_VERSION = 1
MAX_BATCH = 32
MAX_BODY = 65536
IDENTITY_KEYS = ('protocol_version', 'candidate_id', 'artifact_hashes', 'checkpoint_sha256')
RESPONSE_KEYS = ('protocol_version', 'candidate_id', 'instance_id', 'request_id', 'mode')
HOST_FILES = ('native_policy_client.py', 'verify_native_policy.py',
              'play_native_policy.py', 'native_policy_rollout.py')


class Mode(str, Enum):
    FLOAT = 'float'
    REAL_QUANT = 'real_quant'


def validate_matrix(value, *, columns):
    """拒绝隐式字符串/布尔转换，返回有限 float32 矩阵。"""
    if isinstance(value, np.ndarray):
        if value.dtype.kind not in 'iuf':
            raise ValueError('矩阵必须只包含数值')
        raw = value
    else:
        if not isinstance(value, list) or not value or any(
            not isinstance(row, list) or len(row) != columns or
            any(type(item) not in (int, float) for item in row) for row in value
        ):
            raise ValueError('矩阵行列或数值类型非法')
        try:
            raw = np.asarray(value)
        except (ValueError, OverflowError) as error:
            raise ValueError('矩阵数值非法') from error
    if raw.ndim != 2 or raw.shape[1] != columns or not 1 <= len(raw) <= MAX_BATCH:
        raise ValueError(f'要求 [N,{columns}]，1 <= N <= {MAX_BATCH}')
    try:
        with np.errstate(over='ignore', invalid='ignore'):
            result = raw.astype(np.float32)
    except (ValueError, OverflowError) as error:
        raise ValueError('无法转换为 float32') from error
    if not np.isfinite(result).all():
        raise ValueError('矩阵包含非有限 float32')
    return result


def encode_json(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode('utf-8')


def decode_json(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f'JSON 字段重复: {key}')
            result[key] = value
        return result
    def invalid_constant(value):
        raise ValueError(f'非法 JSON 常量: {value}')
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=invalid_constant)


def file_digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest() if hasattr(hashlib, 'file_digest') else _digest_stream(stream)


def _digest_stream(stream):
    digest = hashlib.sha256()
    for block in iter(lambda: stream.read(1024 * 1024), b''):
        digest.update(block)
    return digest.hexdigest()


class ExchangePhase(str, Enum):
    CONNECT = 'connect'
    SEND = 'send'
    RESPONSE_HEADERS = 'response_headers'
    RESPONSE_BODY = 'response_body'
    VALIDATION = 'validation'


class HttpTransport:
    """一次运行拥有的持久连接；失败后关闭，永不自动重发或重连。"""

    def __init__(self):
        self.connection = None
        self.origin = None
        self.closed = False
        self.last_exchange = None
        self.server_connection_id = None

    def close(self):
        self.closed = True
        if self.connection is not None:
            self.connection.close()

    def __call__(self, method, url, body, timeout):
        if self.closed:
            raise RuntimeError('HTTP 连接已关闭，必须开始新的运行')
        parsed = urlsplit(url)
        if (parsed.scheme != 'http' or parsed.hostname not in ('localhost', '127.0.0.1', '::1')
                or parsed.username is not None or parsed.password is not None
                or parsed.query or parsed.fragment):
            raise ValueError('要求本机 HTTP 地址')
        origin = (parsed.hostname, parsed.port or 80)
        if self.origin is not None and self.origin != origin:
            raise ValueError('持久连接不能切换服务地址')
        data = None if body is None else encode_json(body)
        if data is not None and len(data) > MAX_BODY:
            raise ValueError('请求体超过限制')
        started = time.perf_counter()
        trace = {'started_at_utc': datetime.now(timezone.utc).isoformat(),
                 'method': method, 'path': parsed.path or '/',
                 'reused_connection': self.connection is not None, 'status': 'running',
                 'server_connection_id': self.server_connection_id}
        self.last_exchange = trace

        def phase(name, action):
            trace['phase'] = name.value
            before = time.perf_counter()
            try:
                return action()
            finally:
                trace[name.value + '_ms'] = (time.perf_counter() - before) * 1000

        try:
            if self.connection is None:
                self.origin = origin
                self.connection = HTTPConnection(*origin, timeout=timeout)
                phase(ExchangePhase.CONNECT, self.connection.connect)
            trace['phase'] = ExchangePhase.SEND.value
            # http.client 可隐式重连；这里明确禁止在既有连接消失后重连。
            if self.connection.sock is None:
                raise ConnectionError('持久连接已经断开')
            phase(ExchangePhase.SEND, lambda: self.connection.request(
                method, parsed.path or '/', body=data, headers={'Content-Type': 'application/json'}))
            response = phase(ExchangePhase.RESPONSE_HEADERS, self.connection.getresponse)
            trace['http_status'] = response.status
            lengths = response.headers.get_all('Content-Length', [])
            if (response.headers.get('Transfer-Encoding') is not None or len(lengths) != 1
                    or not (lengths[0].isascii() and lengths[0].isdecimal()) or int(lengths[0]) > MAX_BODY):
                raise ValueError('响应缺少合法且有界的 Content-Length')
            raw = phase(ExchangePhase.RESPONSE_BODY, lambda: response.read(MAX_BODY + 1))
            if len(raw) != int(lengths[0]):
                raise ValueError('响应体不完整')
            trace['phase'] = ExchangePhase.VALIDATION.value
            if response.status != 200:
                raise HTTPError(url, response.status, response.reason, response.headers, BytesIO(raw))
            if response.version != 11 or response.will_close:
                raise ValueError('服务未启用 HTTP/1.1 持久连接，请更新并重启原生推理服务')
            connection_id = response.headers.get('X-Connection-ID')
            if (not isinstance(connection_id, str) or not 1 <= len(connection_id) <= 128
                    or (self.server_connection_id is not None and connection_id != self.server_connection_id)):
                raise ValueError('服务端连接标识缺失或变化，请确认两端已更新且没有重连')
            self.server_connection_id = connection_id
            trace['server_connection_id'] = connection_id
            value = decode_json(raw)
            trace['status'] = 'complete'
            return value
        except BaseException as error:
            trace.update(status='failed', error_type=type(error).__name__, message=str(error))
            self.close()
            setattr(error, 'transport_exchange', trace)
            raise
        finally:
            trace['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
            trace['elapsed_ms'] = (time.perf_counter() - started) * 1000


def http_transport(method, url, body, timeout):
    """供一次性调用者使用；PolicyClient 拥有并复用自己的 HttpTransport。"""
    transport = HttpTransport()
    try:
        return transport(method, url, body, timeout)
    finally:
        transport.close()


class PolicyClient:
    def __init__(self, url, expected_identity, timeout=10, *, transport=None):
        parsed = urlsplit(url)
        if (parsed.scheme != 'http' or parsed.hostname not in ('localhost', '127.0.0.1', '::1')
                or parsed.username is not None or parsed.password is not None
                or parsed.path not in ('', '/') or parsed.query or parsed.fragment):
            raise ValueError('要求本机 HTTP 服务根地址')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout 必须为有限正数')
        self.url, self.timeout = url.rstrip('/'), timeout
        self._owned_transport = HttpTransport() if transport is None else None
        self._transport = self._owned_transport if transport is None else transport
        self._closed = False
        self.last_request = None
        try:
            self._handshake(expected_identity)
            self.handshake_exchange = (None if self._owned_transport is None
                                       else self._owned_transport.last_exchange)
        except BaseException as error:
            if self._owned_transport is not None:
                setattr(error, 'transport_exchange', self._owned_transport.last_exchange)
            self.close()
            raise

    def _handshake(self, expected_identity):
        health = self._transport('GET', self.url + '/health', None, self.timeout)
        if not isinstance(health, dict) or type(health.get('protocol_version')) is not int:
            raise ValueError('服务握手格式非法')
        if (health['protocol_version'] != PROTOCOL_VERSION or
                any(key not in expected_identity or health.get(key) != expected_identity[key] for key in IDENTITY_KEYS)
                or not isinstance(health.get('instance_id'), str) or not health['instance_id']
                or health.get('modes') != [mode.value for mode in Mode]
                or health.get('input') != {'dtype': 'float32', 'columns': 4, 'max_batch': MAX_BATCH}
                or health.get('output') != {'dtype': 'float32', 'columns': 1}):
            raise ValueError('服务身份或输入输出协议与核验包不一致')
        self.health = health

    def close(self):
        self._closed = True
        if self._owned_transport is not None:
            self._owned_transport.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def infer(self, observations, mode):
        if self._closed:
            raise RuntimeError('策略客户端已关闭，必须开始新的运行')
        values = validate_matrix(observations, columns=4)
        mode = Mode(mode)
        request = {key: self.health[key] for key in ('protocol_version', 'candidate_id', 'instance_id')}
        request.update(request_id=uuid.uuid4().hex, mode=mode.value, observations=values.tolist())
        started = time.perf_counter()
        self.last_request = {**{key: request[key] for key in RESPONSE_KEYS},
            'started_at_utc': datetime.now(timezone.utc).isoformat(), 'status': 'running'}
        try:
            response = self._transport('POST', self.url + '/infer', request, self.timeout)
            if (not isinstance(response, dict) or type(response.get('protocol_version')) is not int or
                    any(response.get(key) != request[key] for key in RESPONSE_KEYS)):
                raise ValueError('响应不属于当前请求、服务实例或模式')
            actions = validate_matrix(response.get('actions'), columns=1)
            if len(actions) != len(values):
                raise ValueError('动作批次数与观测不一致')
            if 'server_timing' in response or self._owned_transport is not None:
                timing = response.get('server_timing')
                if (not isinstance(timing, dict) or any(
                    type(timing.get(key)) not in (int, float) or not math.isfinite(timing[key]) or timing[key] < 0
                    for key in ('queue_ms', 'inference_ms'))):
                    raise ValueError('服务端耗时字段非法或缺失')
                self.last_request['server_timing'] = timing
            self.last_request['status'] = 'complete'
            return actions
        except BaseException as error:
            self.last_request.update(status='failed', error_type=type(error).__name__, message=str(error))
            self.close()
            raise
        finally:
            self.last_request['elapsed_ms'] = (time.perf_counter() - started) * 1000
            self.last_request['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
            if self._owned_transport is not None:
                self.last_request['transport'] = self._owned_transport.last_exchange



def load_bundle(path):
    root = Path(path).resolve()
    manifest = decode_json((root / 'manifest.json').read_bytes())
    if not isinstance(manifest, dict) or manifest.get('bundle_version') != 1:
        raise ValueError('不支持的核验包')
    files = manifest.get('files')
    if not isinstance(files, dict) or not files:
        raise ValueError('核验包缺少文件索引')
    if not {*HOST_FILES, 'validation_report.json'} <= set(files):
        raise ValueError('核验包缺少主机依赖或尾部来源报告')
    for name, digest in files.items():
        target = (root / name).resolve()
        if target.parent != root or file_digest(target) != digest:
            raise ValueError(f'核验包文件不匹配: {name}')
    if not isinstance(manifest.get('cases'), list) or not manifest['cases']:
        raise ValueError('核验包没有测试批次')
    if any(name not in files for name in manifest['cases']):
        raise ValueError('测试批次未登记哈希')
    if not isinstance(manifest.get('identity'), dict):
        raise ValueError('核验包缺少预期模型身份')
    cases = {name: decode_json((root / name).read_bytes()) for name in manifest['cases']}
    report = decode_json((root / 'validation_report.json').read_bytes())
    validate_case_coverage(manifest, cases, report)
    manifest['_root'] = root
    return manifest


def validation_tails(report, identity):
    """从与当前模型绑定的已完成报告取得前 20 条尾部记录。"""
    try:
        inputs = report['context']['inputs']
        binding = report['model_binding']
        if (report['status'] != 'complete' or type(report['sample_count']) is not int
                or report['sample_count'] < MAX_BATCH or binding['status'] != 'verified'
                or binding['checkpoint_sha256'] != identity['checkpoint_sha256']
                or any(inputs[key]['sha256'] != digest for key, digest in identity['artifact_hashes'].items())):
            raise ValueError('独立误差报告与模型身份不符')
        tails = report['worst_samples']['real_quant'][:20]
        if not tails:
            raise ValueError('误差报告缺少尾部样本')
        seen = set()
        for row in tails:
            index = row['sample_index']
            if type(index) is not int or not 0 <= index < report['sample_count'] or index in seen:
                raise ValueError('尾部索引非法或重复')
            seen.add(index)
            validate_matrix([row['observation']], columns=4)
        return tails
    except (KeyError, TypeError) as error:
        raise ValueError('误差报告缺少来源或模型绑定字段') from error


def validate_case_coverage(manifest, cases, report):
    tails = validation_tails(report, manifest['identity'])
    batches = {}
    for case in cases.values():
        observations = validate_matrix(case['observations'], columns=4)
        size = len(observations)
        if size in batches:
            raise ValueError('核验批次重复')
        for mode in Mode:
            if len(validate_matrix(case[mode.value], columns=1)) != size:
                raise ValueError('参考动作行数不匹配')
        batches[size] = observations
    if set(batches) != {1, MAX_BATCH} or len(manifest['cases']) != 2:
        raise ValueError('核验包必须同时包含 B1 与 B32')
    source = manifest.get('source', {})
    indices = source.get('indices')
    if (source.get('sha256') != report['context']['inputs']['data']['sha256']
            or source.get('validation_report_sha256') != manifest['files']['validation_report.json']
            or not isinstance(indices, list) or len(indices) != MAX_BATCH
            or any(type(index) is not int or not 0 <= index < report['sample_count'] for index in indices)
            or len(set(indices)) != MAX_BATCH):
        raise ValueError('尾部来源或采样索引不完整')
    if not np.array_equal(batches[1], batches[MAX_BATCH][:1]) or indices[0] != tails[0]['sample_index']:
        raise ValueError('B1 未覆盖报告首个尾部样本')
    for row in tails:
        index = row['sample_index']
        if index not in indices or not np.array_equal(
                batches[MAX_BATCH][indices.index(index)], np.asarray(row['observation'], dtype=np.float32)):
            raise ValueError('核验包缺少来源已确认的尾部观测')


def verify_bundle(client, bundle):
    checked = []
    for name in bundle['cases']:
        path = bundle['_root'] / name
        if file_digest(path) != bundle['files'][name]:
            raise ValueError(f'核验数据发生变化: {name}')
        case = decode_json(path.read_bytes())
        observations = validate_matrix(case['observations'], columns=4)
        for mode in Mode:
            client.verification_context = {'case': name, 'mode': mode.value}
            expected = validate_matrix(case[mode.value], columns=1)
            actual = client.infer(observations, mode.value)
            if not np.array_equal(actual, expected):
                raise ValueError(f'原生数值不一致: {name}, {mode.value}')
            checked.append({'case': name, 'mode': mode.value, 'rows': len(observations), 'exact_match': True,
                            'request': client.last_request})
    client.verification_context = None
    return {'status': 'passed', 'health': client.health, 'checks': checked}
