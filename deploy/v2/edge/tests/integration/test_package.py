"""JSON package: real compiler/files boundary, no SDK or hardware."""
from pathlib import Path
import json
import subprocess
import tempfile
import unittest
EDGE = Path(__file__).parents[2]
BASE = dict(schema_version=2,model_cpu=None,model_npu='policy.om',sha256_cpu=None,sha256_npu='a'*64,
            control_hz=400,action_scale_mm_s=900,velocity_limit_mm_s=900,acceleration_mm_s2=10000,
            angle_sign=-1,pause_deg=20,resume_deg=18)
class PackageLoadTest(unittest.TestCase):
    def test_load_and_reject_malformed_packages(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)
            (p/'probe.c').write_text('#include "package.h"\n#include <stdio.h>\nint main(int n,char**v){edge_package p;char e[200];(void)n;int r=edge_package_load(v[1],&p,e,sizeof(e));if(r){puts(e);return 1;}printf("%.0f %d\\n",p.control_hz,p.velocity_limit_mm_s);return 0;}')
            subprocess.run(['cc','-DYYJSON_DISABLE_WRITER=1','-DYYJSON_DISABLE_UTILS=1','-DYYJSON_DISABLE_FILE=1','-DYYJSON_DISABLE_INCR_READER=1','-DYYJSON_READER_DEPTH_LIMIT=16','-std=c11','-Wall','-Wextra','-Werror','-I'+str(EDGE),str(p/'probe.c'),str(EDGE/'package.c'),str(EDGE/'third_party/yyjson/yyjson.c'),'-lm','-o',str(p/'probe')],check=True)
            (p/'policy.om').write_bytes(b'model')
            (p/'policy.fp32').write_bytes(b'cpu')
            def run(value):
                text=value if isinstance(value,str) else json.dumps(value)
                (p/'edge-package.json').write_bytes(text.encode())
                return subprocess.run([str(p/'probe'),str(p/'edge-package.json')],capture_output=True,text=True)
            self.assertEqual(run(BASE).returncode,0)
            self.assertEqual(run(dict(reversed(list(BASE.items())))).returncode,0)
            cpu=dict(BASE,model_cpu='policy.fp32',sha256_cpu='b'*64,model_npu=None,sha256_npu=None)
            self.assertEqual(run(cpu).returncode,0)
            bad=[dict(BASE,unknown=1),dict(BASE,schema_version=1),dict(BASE,schema_version=True),
                 dict(BASE,control_hz='400'),dict(BASE,control_hz=float('nan')),dict(BASE,model_npu='../policy.om'),
                 dict(BASE,model_npu={'file':'policy.om'}),dict(BASE,sha256_npu=None),dict(BASE,pause_deg=21),
                 dict(BASE,acceleration_mm_s2=1.5),dict(BASE,acceleration_mm_s2=2**32),
                 dict(BASE,model_npu=None,sha256_npu=None),
                 json.dumps(BASE)[:-1]+',"control_hz":400}',json.dumps(BASE)+' {}',
                 json.dumps(BASE)+'\x00junk',json.dumps(BASE).replace('policy.om','policy.om\\u0000junk'),'[]','x'*8193]
            missing=dict(BASE);del missing['resume_deg'];bad.append(missing)
            for value in bad:
                with self.subTest(value=value): self.assertNotEqual(run(value).returncode,0)
            (p/'policy.om').unlink();self.assertNotEqual(run(BASE).returncode,0)
if __name__=='__main__': unittest.main()
