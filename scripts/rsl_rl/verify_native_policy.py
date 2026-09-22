"""主机固定观测核验入口：只需 NumPy，无需 AMCT 或 IsaacLab。"""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import sys
import uuid

if __package__:
    from .native_policy_client import PolicyClient, encode_json, load_bundle, verify_bundle
else:
    from native_policy_client import PolicyClient, encode_json, load_bundle, verify_bundle


def _write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_bytes(encode_json(value) + b'\n')
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description='核验本机原生策略服务与直接推理结果一致')
    parser.add_argument('--url', default='http://127.0.0.1:8765')
    parser.add_argument('--bundle', type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument('--output-dir', type=Path, default=Path('native-policy-verification'))
    parser.add_argument('--timeout', type=float, default=10)
    args = parser.parse_args()
    root = args.output_dir.resolve() / ('verification-' + uuid.uuid4().hex)
    root.mkdir(parents=True)
    stage, client, bundle = 'startup', None, None
    try:
        _write_json(root / 'startup.json', {'started_at': datetime.now(timezone.utc).isoformat(),
            'url': args.url, 'bundle': str(args.bundle.resolve()), 'timeout': args.timeout,
            'arguments': sys.argv[1:]})
        stage = 'bundle_verification'
        bundle = load_bundle(args.bundle)
        stage = 'handshake'
        client = PolicyClient(args.url, bundle['identity'], timeout=args.timeout)
        stage = 'verification'
        report = verify_bundle(client, bundle)
        stage = 'connection_close'
        client.close()
        stage = 'report_write'
        _write_json(root / 'report.json', report)
        print('[PASS] 两种模式的固定批次均与原生直接推理逐元素相同')
        print(f'[OUTPUT] {root}')
    except BaseException as error:
        failure = {'status': 'failed', 'stage': stage,
            'error_type': type(error).__name__, 'message': str(error),
            'identity': None if bundle is None else bundle['identity'],
            'health': None if client is None else client.health,
            'request': None if client is None else client.last_request,
            'verification_context': getattr(client, 'verification_context', None),
            'transport_exchange': getattr(error, 'transport_exchange', None)}
        if client is not None:
            try:
                client.close()
            except BaseException as cleanup_error:
                failure['connection_cleanup_error'] = {'error_type': type(cleanup_error).__name__,
                                                       'message': str(cleanup_error)}
        print(encode_json(failure).decode(), file=sys.stderr, flush=True)
        try:
            _write_json(root / 'failure.json', failure)
        except Exception as persistence_error:
            print(f'无法保存 failure.json: {persistence_error}', file=sys.stderr, flush=True)
        print(f'[FAILED] {root}')
        raise


if __name__ == '__main__':
    main()
