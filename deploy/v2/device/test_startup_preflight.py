import unittest
from unittest.mock import Mock
import run_policy as r
class StartupTest(unittest.TestCase):
 def test_wait_refreshes_and_preserves_configuration_blocker(self):
  now=[0.];session=Mock();session.failure=None;session.mode=('READY','NONE');session._mode_at=0.;session.values={'VEL_MAX':(1250,0.)};session.expected_readback={'VEL_MAX':900}
  session.preflight_reason.side_effect=lambda:'readback_VEL_MAX' if now[0]-session._mode_at<=1 else 'mode_not_ready'
  session.refresh.side_effect=lambda:setattr(session,'_mode_at',now[0])
  receiver=Mock();receiver.poll.return_value=True;transport=Mock();transport.partial=False;transport.in_waiting=0;report={}
  with self.assertRaisesRegex(RuntimeError,'preflight:readback_VEL_MAX'):
   r.await_startup(session,receiver,transport,report,clock=lambda:now[0],sleep=lambda _:now.__setitem__(0,now[0]+.1))
  self.assertGreaterEqual(session.refresh.call_count,3)
  self.assertEqual(report['startup_preflight']['first_blocker']['reason'],'readback_VEL_MAX')
  self.assertEqual(report['startup_preflight']['last_blocker']['readback']['VEL_MAX'],1250)
 def test_ready_returns_without_refresh_or_motion(self):
  session=Mock();session.failure=None;session.preflight_reason.return_value=None;session.values={'STEP_LOSS_UM':(3000,0.)}
  receiver=Mock();receiver.poll.return_value=True;transport=Mock();transport.partial=False;transport.in_waiting=0
  r.await_startup(session,receiver,transport,{},clock=lambda:0.,sleep=lambda _:None)
  session.refresh.assert_not_called()
