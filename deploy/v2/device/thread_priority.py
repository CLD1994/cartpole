"""本次运行内可选调整当前 Windows 线程；不改变进程级别或系统设置。"""
from contextlib import contextmanager
import ctypes
import sys

ABOVE_NORMAL = 1
ERROR_RETURN = 0x7fffffff


class _WindowsPriority:
    def __init__(self):
        from ctypes import wintypes
        self.api = ctypes.WinDLL('kernel32', use_last_error=True)
        self.api.GetCurrentThread.argtypes = []
        self.api.GetCurrentThread.restype = wintypes.HANDLE
        self.api.GetThreadPriority.argtypes = [wintypes.HANDLE]
        self.api.GetThreadPriority.restype = ctypes.c_int
        self.api.SetThreadPriority.argtypes = [wintypes.HANDLE, ctypes.c_int]
        self.api.SetThreadPriority.restype = wintypes.BOOL
        # 当前线程伪句柄只能在创建此对象的线程使用，无需 CloseHandle。
        self.handle = self.api.GetCurrentThread()

    def get(self):
        value = self.api.GetThreadPriority(self.handle)
        if value == ERROR_RETURN:
            raise ctypes.WinError(ctypes.get_last_error())
        return value

    def set(self, value):
        if not self.api.SetThreadPriority(self.handle, value):
            raise ctypes.WinError(ctypes.get_last_error())


def _load_api():
    return _WindowsPriority()


@contextmanager
def current_thread_priority(requested='unchanged'):
    """在调用线程进入和退出；applied 仅代表设置成功且读回符合请求。

    优先级不是实时保证。默认不调用系统 API。退出时尽力恢复原级别，
    恢复失败写入报告，不覆盖运行主体原本的异常。
    """
    if requested not in ('unchanged', 'above-normal'):
        raise ValueError('unknown_thread_priority')
    report = dict(requested=requested, applied=False, actual=None, previous=None,
                  set_succeeded=None, error=None, platform=sys.platform,
                  scope='current_thread_only; process_priority_unchanged; no_realtime_guarantee',
                  restore=dict(attempted=False, confirmed=False, actual=None, error=None))
    api = None
    changed = False
    if requested != 'unchanged':
        if sys.platform != 'win32':
            report['error'] = 'unsupported_platform'
        else:
            try:
                api = _load_api()
                report['previous'] = api.get()
                report['set_succeeded'] = False
                api.set(ABOVE_NORMAL)
                changed = True
                report['set_succeeded'] = True
                report['actual'] = api.get()
                report['applied'] = report['actual'] == ABOVE_NORMAL
                if not report['applied']:
                    report['error'] = 'priority_readback_mismatch'
            except (OSError, AttributeError) as exc:
                report['error'] = f'{type(exc).__name__}: {exc}'
    try:
        yield report
    finally:
        if changed:
            restoration = report['restore']
            restoration['attempted'] = True
            try:
                api.set(report['previous'])
                restoration['actual'] = api.get()
                restoration['confirmed'] = restoration['actual'] == report['previous']
                if not restoration['confirmed']:
                    restoration['error'] = 'priority_readback_mismatch'
            except (OSError, AttributeError) as exc:
                restoration['error'] = f'{type(exc).__name__}: {exc}'
