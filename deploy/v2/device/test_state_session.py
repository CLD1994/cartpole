"""仅测量与有效位的正式协议；通信超时不冒充采样年龄。"""
import unittest
from session import Session, Limits, Phase

def frame(*,flags=3,x=10,th=20,vx=30,w=40):
 if not flags&1:x,vx=0,0
 if not flags&2:th,w=0,0
 return f'STATE {flags} {x} {th} {vx} {w}'
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
 def test_atomic_valid_frame(self):
  self.ready();self.assertEqual(self.s.observation.si(),(1e-5,2e-5,.03,.04))
 def test_each_invalid_lane_stops(self):
  for flags in (0,1,2):
   with self.subTest(flags=flags):
    self.setUp();self.ready();self.s.receive(frame(flags=flags));self.assertEqual(self.s.phase,Phase.STOPPING);self.assertIsNone(self.s.observation)
 def test_repeated_values_refresh_receive_time_not_sample_identity(self):
  self.ready();self.now+=.002;self.s.receive(frame());self.assertEqual(self.s.observation.received_at,self.now)
 def test_received_values_can_change_without_device_metadata(self):
  self.ready();self.s.receive(frame(x=11));self.assertEqual(self.s.observation.x_um,11)
 def test_observing_invalid_may_recover(self):
  self.s.receive(frame(flags=0));self.assertIsNone(self.s.observation)
  self.s.receive(frame());self.assertIsNotNone(self.s.observation)
 def test_expired_stream_cannot_restart(self):
  self.ready();self.now+=.101;self.s.receive(frame());self.assertEqual(self.s.phase,Phase.STOPPING)
 def test_mode_change_stops(self):
  self.ready();self.s.receive('MODE DISABLED NONE');self.assertEqual(self.s.phase,Phase.STOPPING)
 def test_old_protocol_rejected(self):
  for line in ('STATE 1 2 3 4','VEL_VALID 1 1','CART_STATE1 123 0'):
   with self.subTest(line=line):
    self.setUp();self.ready();self.s.receive(line);self.assertEqual(self.s.phase,Phase.STOPPING)
 def test_old_protocol_before_start_never_moves(self):
  self.s.receive('VEL_VALID 1 1');self.assertEqual(self.s.phase,Phase.FINISHED);self.assertEqual(self.p.sent,[])
