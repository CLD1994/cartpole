import unittest
from unittest.mock import patch

import thread_priority as tp

class FakeApi:
    def __init__(self, values=(0, 1, 0), fail_set=False):
        self.values=iter(values); self.sets=[]; self.fail_set=fail_set
    def get(self):
        value=next(self.values)
        if isinstance(value, Exception): raise value
        return value
    def set(self, value):
        self.sets.append(value)
        if self.fail_set: raise OSError(5, 'denied')

class PriorityTests(unittest.TestCase):
    def test_default_does_not_load_or_change_os(self):
        with patch.object(tp, '_load_api') as load:
            with tp.current_thread_priority() as report:
                self.assertFalse(report['applied'])
                self.assertIsNone(report['actual'])
            load.assert_not_called()
    def test_above_normal_is_verified_and_restored(self):
        api=FakeApi()
        with patch.object(tp.sys, 'platform', 'win32'), patch.object(tp, '_load_api', return_value=api):
            with tp.current_thread_priority('above-normal') as report:
                self.assertTrue(report['applied']); self.assertEqual(report['actual'], 1)
                self.assertEqual(report['previous'], 0)
            self.assertTrue(report['restore']['confirmed']); self.assertEqual(api.sets, [1, 0])
    def test_unsupported_is_not_success(self):
        with patch.object(tp.sys, 'platform', 'linux'), patch.object(tp, '_load_api') as load:
            with tp.current_thread_priority('above-normal') as report:
                self.assertFalse(report['applied']); self.assertEqual(report['error'], 'unsupported_platform')
            load.assert_not_called()
    def test_set_error_is_recorded(self):
        api=FakeApi(values=(0,), fail_set=True)
        with patch.object(tp.sys, 'platform', 'win32'), patch.object(tp, '_load_api', return_value=api):
            with tp.current_thread_priority('above-normal') as report:
                self.assertFalse(report['applied']); self.assertIn('denied', report['error'])
                self.assertFalse(report['set_succeeded'])
    def test_failed_readback_not_claimed_success_but_restored(self):
        api=FakeApi(values=(0, OSError('readback'), 0))
        with patch.object(tp.sys, 'platform', 'win32'), patch.object(tp, '_load_api', return_value=api):
            with tp.current_thread_priority('above-normal') as report:
                self.assertFalse(report['applied']); self.assertIsNone(report['actual'])
                self.assertTrue(report['set_succeeded'])
            self.assertTrue(report['restore']['confirmed']); self.assertEqual(api.sets, [1,0])
    def test_body_exception_still_restores(self):
        api=FakeApi()
        with patch.object(tp.sys, 'platform', 'win32'), patch.object(tp, '_load_api', return_value=api):
            with self.assertRaisesRegex(RuntimeError, 'body'):
                with tp.current_thread_priority('above-normal') as report:
                    raise RuntimeError('body')
            self.assertTrue(report['restore']['confirmed'])
    def test_restore_mismatch_reported(self):
        api=FakeApi(values=(0, 1, 1))
        with patch.object(tp.sys, 'platform', 'win32'), patch.object(tp, '_load_api', return_value=api):
            with tp.current_thread_priority('above-normal') as report: pass
            self.assertFalse(report['restore']['confirmed'])
            self.assertEqual(report['restore']['error'], 'priority_readback_mismatch')



class FakeFunction:
    def __init__(self, result): self.result=result
    def __call__(self, *args): return self.result

class WindowsBindingTests(unittest.TestCase):
    def api(self, get_result=1, set_result=1):
        from types import SimpleNamespace
        dll=SimpleNamespace(GetCurrentThread=FakeFunction(123),
            GetThreadPriority=FakeFunction(get_result), SetThreadPriority=FakeFunction(set_result))
        with patch.object(tp.ctypes, 'WinDLL', return_value=dll, create=True):
            api=tp._WindowsPriority()
        return api, dll
    def test_signatures_and_actual_level(self):
        from ctypes import wintypes
        api,dll=self.api()
        self.assertEqual(api.get(),1)
        self.assertEqual(dll.GetCurrentThread.argtypes, [])
        self.assertIs(dll.GetCurrentThread.restype,wintypes.HANDLE)
        self.assertEqual(dll.GetThreadPriority.argtypes,[wintypes.HANDLE])
        self.assertIs(dll.GetThreadPriority.restype,tp.ctypes.c_int)
        self.assertEqual(dll.SetThreadPriority.argtypes,[wintypes.HANDLE,tp.ctypes.c_int])
        self.assertIs(dll.SetThreadPriority.restype,wintypes.BOOL)
    def test_get_failure_sentinel_raises(self):
        api,_=self.api(get_result=0x7fffffff)
        with patch.object(tp.ctypes,'get_last_error',return_value=5,create=True), patch.object(tp.ctypes,'WinError',return_value=OSError(5,'denied'),create=True):
            with self.assertRaises(OSError): api.get()
    def test_set_false_raises(self):
        api,_=self.api(set_result=0)
        with patch.object(tp.ctypes,'get_last_error',return_value=5,create=True), patch.object(tp.ctypes,'WinError',return_value=OSError(5,'denied'),create=True):
            with self.assertRaises(OSError): api.set(1)

if __name__ == '__main__': unittest.main()
