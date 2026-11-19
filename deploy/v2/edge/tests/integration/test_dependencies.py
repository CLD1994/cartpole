"""Compile the business modules' actual include graphs and unresolved references.

This checks a dependency rule, not runtime module loading: cartpole business code
must depend on the injected device contract, never the serial/protocol adapter.
"""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile
import unittest

EDGE = Path(__file__).resolve().parents[2]
MODULES = ('cartpole.c', 'cartpole_controller.c', 'cartpole_schedule.c', 'cartpole_motion.c', 'cartpole_travel.c', 'cartpole_pole_calibration.c', 'console.c')


class BusinessDependencies(unittest.TestCase):
    def test_business_modules_do_not_reference_transport_or_protocol(self):
        compiler = shlex.split(os.environ.get('CC', 'cc'))
        with tempfile.TemporaryDirectory() as directory:
            for module in MODULES:
                with self.subTest(module=module):
                    dependencies = subprocess.check_output(
                        compiler + ['-MM', '-I.', module], cwd=EDGE, text=True)
                    headers = {Path(header).name for header in re.findall(r'[\w/.-]+\.h', dependencies)}
                    self.assertFalse(headers & {'serial_cartpole.h', 'uart.h', 'proto.h'}, dependencies)
                    if module != 'cartpole.c':
                        self.assertFalse(headers & {'cartpole_internal.h', 'cartpole_device.h'}, dependencies)
                    obj = Path(directory) / (module + '.o')
                    subprocess.run(compiler + ['-std=c11', '-D_DEFAULT_SOURCE', '-I.', '-c', module,
                                               '-o', str(obj)], cwd=EDGE, check=True)
                    references = subprocess.check_output(['nm', '-u', str(obj)], text=True)
                    symbols = [line.split()[-1] for line in references.splitlines() if line.split()]
                    self.assertFalse([s for s in symbols if s.startswith(('uart_', 'proto_', 'serial_cartpole_'))], references)


if __name__ == '__main__':
    unittest.main()
