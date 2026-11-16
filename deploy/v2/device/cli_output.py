"""只读诊断终端摘要；完整证据保留在报告中。"""
import json
import sys


def readonly_entrypoint(main, argv=None):
    """只读命令的外层失败出口；不将未完成检查解释为验证通过。"""
    args = sys.argv[1:] if argv is None else argv
    try:
        return main()
    except Exception as exc:
        report = {'status': 'error', 'verification_complete': False,
                  'motion_sent': False, 'error': f'{type(exc).__name__}: {exc}'}
        print_readonly_summary(report, json_output='--json' in args)
        return 1


def print_readonly_summary(report, path=None, *, json_output=False):
    if json_output:
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return
    print('只读检查：' + str(report.get('status', 'unknown')) + '；未发送运动指令。')
    mode = report.get('firmware_mode', report.get('mode'))
    if mode is not None:
        print('模式：' + (' '.join(mode) if isinstance(mode, (list, tuple)) else str(mode)))
    reasons = list(dict.fromkeys(str(report[key]) for key in
        ('error', 'receive_error', 'session_failure', 'preflight_reason') if report.get(key)))
    for reason in reasons:
        print('原因：' + reason)
    if 'samples' in report:
        print(f'有效观测：{len(report["samples"])}')
    if path is not None:
        print(f'报告：{path}')
