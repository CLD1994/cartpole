"""统一状态帧的会话门禁单元测试。"""
import unittest
from session import Session, Limits, Phase

def frame(*,boot=1,tick=1000,cs=1,ps=1,flags=3,ca=0,pa=0,x=10,th=20,vx=30,w=40):
 if not flags&1:cs,ca,x,vx=0,65535,0,0
 if not flags&2:ps,pa,th,w=0,65535,0,0
 return f'STATE {boot:08x} {tick:08x} {flags:x} {cs:04x} {ps:04x} {ca:04x} {pa:04x} {x&0xffffffff:08x} {th&0xffffffff:08x} {vx&0xffffffff:08x} {w&0xffffffff:08x}'

class Port:
 def __init__(self):self.sent=[]
 def write(self,b):self.sent.append(b);return len(b)
class StateSessionTests(unittest.TestCase):
 def setUp(self):
  self.now=10.;self.p=Port();self.s=Session(self.p,lambda:self.now,Limits(500,1250,10000,10000))
 def ready(self,**kw):
  self.s.receive('MODE READY NONE')
  for k,v in {'CAL_STATE':2,'CAL_LENGTH_UM':853325,'CAL_SAVED':1,'CAL_STORAGE':0,**self.s.limits.config}.items():self.s.receive(f'OK {k} {v}')
  self.s.receive(frame(**kw));self.s.start()
 def test_complete_frame_works_without_heartbeat(self):
  self.ready();self.assertEqual(self.s.observation.si(),(1e-5,2e-5,.03,.04))
 def test_each_invalid_lane_stops(self):
  for flags in (0,1,2):
   with self.subTest(flags=flags):
    self.setUp();self.ready();self.s.receive(frame(tick=2000,cs=2,ps=2,flags=flags));self.assertEqual(self.s.phase,Phase.STOPPING);self.assertIsNone(self.s.observation)
 def test_duplicate_cannot_renew_observation(self):
  self.ready();old=self.s.observation;self.now+=.002
  self.s.receive(frame(tick=3000,ca=2000,pa=2000));self.assertIs(self.s.observation,old)
 def test_boot_change_stops(self):
  self.ready();self.s.receive(frame(boot=2));self.assertEqual(self.s.failure,'state_boot_changed')
 def test_backward_seq_stops(self):
  self.ready(cs=20,ps=20);self.s.receive(frame(tick=2000,cs=19,ps=21));self.assertEqual(self.s.failure,'state_sequence')
 def test_wrap_is_forward(self):
  self.ready(tick=0xffffff00,cs=65535,ps=65535);self.now+=.001
  self.s.receive(frame(tick=744,cs=0,ps=0));self.assertEqual(self.s.phase,Phase.ACTIVE);self.assertEqual(self.s.observation.received_at,self.now)
 def test_device_tick_backwards_stops(self):
  self.ready();self.s.receive(frame(tick=999,cs=2,ps=2));self.assertEqual(self.s.failure,'state_clock')
 def test_old_protocol_rejected(self):
  for line in ('STATE 1 2 3 4','VEL_VALID 1 1','CART_STATE1 123 0'):
   with self.subTest(line=line):
    self.setUp();self.ready();self.s.receive(line);self.assertEqual(self.s.phase,Phase.STOPPING)
 def test_device_age_subtracted(self):
  self.ready(ca=500,pa=1000);self.assertAlmostEqual(self.s.observation.received_at,self.now-.001)
 def test_host_backlog_not_hidden_by_new_frame(self):
  self.ready();self.now+=.01;self.s.receive(frame(tick=3000,cs=2,ps=2));self.assertAlmostEqual(self.s.observation.received_at,self.now-.008)
 def test_expired_stream_does_not_restart(self):
  self.ready();self.now+=.101;self.s.receive(frame(tick=102000,cs=2,ps=2));self.assertEqual(self.s.phase,Phase.STOPPING)

 def test_old_protocol_reported_before_start(self):
  self.s.receive('VEL_VALID 1 1')
  self.assertEqual(self.s.failure,'unsupported_protocol')
  self.assertEqual(self.s.phase,Phase.FINISHED)
  self.assertEqual(self.p.sent,[])

 def test_invalid_to_valid_same_tick_does_not_create_measurement(self):
  self.s.receive(frame(flags=0))
  self.s.receive(frame())
  self.assertIsNone(self.s.observation)
  self.s.receive(frame(tick=2000,cs=2,ps=2))
  self.assertIsNotNone(self.s.observation)

 def test_legacy_state_reported_before_start(self):
  self.s.receive('STATE 1 2 3 4')
  self.assertEqual(self.s.failure,'malformed_STATE')
  self.assertEqual(self.s.phase,Phase.FINISHED)
  self.assertEqual(self.p.sent,[])

 def test_new_sequence_with_regressing_sample_time_stops(self):
  self.ready(tick=100000,cs=100,ps=100)
  self.s.receive(frame(tick=102000,cs=101,ps=101,ca=20000))
  self.assertEqual(self.s.failure,'state_sample_time')
 def test_duplicate_sequence_changed_value_stops(self):
  self.ready();self.s.receive(frame(tick=3000,ca=2000,pa=2000,x=11))
  self.assertEqual(self.s.failure,'state_sample_time')
 def test_invalid_frame_preserves_sample_watermark(self):
  self.s.receive(frame(tick=100000,cs=100,ps=100))
  self.s.receive(frame(tick=101000,flags=0))
  self.s.receive(frame(tick=102000,cs=101,ps=101,ca=20000))
  self.assertIsNone(self.s.observation)
  self.assertEqual(self.s._sample_watermarks['cart'][0].seq,100)
 def test_noncanonical_whitespace_rejected(self):
  for line in (frame().replace(' ', '  ',1),frame()+' ',frame().replace(' ','\t',1)):
   with self.assertRaises(ValueError):
    from state_frame import parse_state
    parse_state(line)

 def test_sample_corruption_before_start_latches(self):
  self.s.receive(frame(tick=100000,cs=100,ps=100))
  self.s.receive(frame(tick=102000,cs=101,ps=101,ca=20000))
  self.assertEqual(self.s.phase,Phase.FINISHED)
  self.s.receive(frame(tick=104000,cs=102,ps=102))
  self.assertIsNone(self.s.observation)
  self.assertEqual(self.p.sent,[])
