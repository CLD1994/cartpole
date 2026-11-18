"""真实文件/编译器边界的集成测试，独立于纯单元测试运行。"""
from pathlib import Path
import subprocess
import tempfile
import unittest
EDGE = Path(__file__).parents[2]
BASE = ('EDGE_PACKAGE_V1\nmodel=policy.om\nsha256=' + 'a'*64 + '\ncontrol_hz=400\n'
        'action_scale_mm_s=900\nvelocity_limit_mm_s=900\nacceleration_mm_s2=10000\n'
        'angle_sign=-1\npause_deg=20\nresume_deg=18\n')
class PackageLoadTest(unittest.TestCase):
    def test_load_and_reject_malformed_packages(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory)
            (p/'probe.c').write_text('#include "package.h"\n#include <stdio.h>\nint main(int n,char**v){edge_package p;char e[200];(void)n;int r=edge_package_load(v[1],&p,e,sizeof(e));if(r){puts(e);return 1;}printf("%.0f %d %d %s\\n",p.control_hz,p.action_scale_mm_s,p.acceleration_mm_s2,p.model_path);return 0;}')
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I'+str(EDGE),str(p/'probe.c'),str(EDGE/'package.c'),'-lm','-o',str(p/'probe')],check=True)
            (p/'policy.om').write_bytes(b'test-model')
            def run(text):
                (p/'edge-package.txt').write_text(text)
                return subprocess.run([str(p/'probe'),str(p/'edge-package.txt')],capture_output=True,text=True)
            good = run(BASE)
            self.assertEqual(good.returncode,0,good.stdout)
            self.assertIn('400 900 10000',good.stdout)
            for text in (BASE+'extra\n', BASE.replace('400','nan'), BASE.replace('900','0'),
                         BASE.replace('policy.om','../policy.om'),BASE.replace('angle_sign=-1','angle_sign=0'),
                         BASE.replace('resume_deg=18','resume_deg=21'),BASE[:-1],BASE.replace('10000','99999999999999999999999')):
                with self.subTest(text=text): self.assertNotEqual(run(text).returncode,0)
            (p/'policy.om').unlink()
            self.assertNotEqual(run(BASE).returncode,0)
if __name__ == '__main__': unittest.main()
