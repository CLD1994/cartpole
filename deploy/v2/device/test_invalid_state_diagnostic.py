import unittest
from unittest.mock import Mock,patch
from run_policy import BoundSession
from session import Limits,Phase
class InvalidStateDiagnosticTest(unittest.TestCase):
 def test_first_invalid_frame_records_lane_and_receive_time(self):
  for flags,cart,pole in [(0,False,False),(1,True,False),(2,False,True)]:
   with self.subTest(flags=flags):
    t=Mock();t.write.side_effect=len
    s=BoundSession(t,lambda:12.5,Limits(400,900,10000,10000),{})
    s.phase=Phase.ACTIVE
    raw=f'STATE {flags} 0 0 0 0'
    with patch('session.Session.receive') as downstream:
     s.receive(raw)
     downstream.assert_called_once_with(raw)
    self.assertEqual(s.first_invalid_state,{'raw_line':raw,'flags':flags,'cart_valid':cart,'pole_valid':pole,'host_received_clock_s':12.5,'phase':'active'})
    with patch('session.Session.receive'):s.receive('STATE 0 0 0 0 0')
    self.assertEqual(s.first_invalid_state['flags'],flags)
 def test_valid_and_malformed_frames_do_not_create_lane_evidence(self):
  s=BoundSession(Mock(),lambda:1.,Limits(400,900,10000,10000),{});s.phase=Phase.ACTIVE
  with patch('session.Session.receive'):
   s.receive('STATE 3 0 0 0 0');s.receive('STATE corrupt')
  self.assertIsNone(s.first_invalid_state)
