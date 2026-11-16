"""Receiver 与真实 Session 协作测试，传输与时钟为受控替身。"""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import unittest
from receiver import Receiver
from session import Session, Limits, Phase
from test_state_session import frame

class Wire:
 def __init__(self):self.data=b'';self.sent=[]
 @property
 def in_waiting(self):return len(self.data)
 def read(self,n):data,self.data=self.data[:n],self.data[n:];return data
 def write(self,b):self.sent.append(b);return len(b)
class IntegrationTests(unittest.TestCase):
 def setUp(self):
  self.now=1.;self.wire=Wire();self.s=Session(self.wire,lambda:self.now,Limits(500,1250,10000,10000));self.r=Receiver(self.wire,self.s,lambda:self.now)
  lines=['MODE READY NONE']+[f'OK {k} {v}' for k,v in {'CAL_STATE':2,'CAL_LENGTH_UM':853325,'CAL_SAVED':1,'CAL_STORAGE':0,**self.s.limits.config}.items()]
  self.send('\n'.join(lines)+'\n')
 def send(self,text):self.wire.data=text.encode();self.assertTrue(self.r.poll())
 def test_fragmented_atomic_state_and_invalid_pole_stop(self):
  line=frame()+'\n';self.send(line[:10]);self.assertIsNone(self.s.observation)
  self.now+=.001;self.send(line[10:]);self.s.start();self.assertEqual(self.s.phase,Phase.ACTIVE)
  self.now+=.002;self.send(frame(flags=1)+'\n');self.assertEqual(self.s.phase,Phase.STOPPING);self.assertIsNone(self.s.observation);self.assertIn(b'VEL 0\n',self.wire.sent)
 def test_bad_batch_cannot_deliver_good_frame_first(self):
  self.wire.data=(frame()+'\n').encode()+b'\xff\n';self.assertFalse(self.r.poll());self.assertIsNone(self.s.observation);self.assertEqual(self.r.error,'invalid_ascii')
 def test_partial_frame_timeout_keeps_state_unavailable(self):
  self.send(frame()[:25]);self.now+=.021;self.wire.data=b'';self.assertFalse(self.r.poll());self.assertIsNone(self.s.observation)
