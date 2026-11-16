"""统一状态帧解析契约的单元测试。"""
import unittest
from state_frame import parse_state

GOOD='STATE 12345678 ffffffff 3 ffff 0000 0001 0002 ffffffff 00000002 fffffffd 00000004'
class FrameTests(unittest.TestCase):
 def test_valid_signed_and_wrap_values(self):
  f=parse_state(GOOD)
  self.assertEqual((f.boot,f.tick,f.cart.seq,f.pole.seq),(0x12345678,0xffffffff,65535,0))
  self.assertEqual((f.cart.position,f.pole.position,f.cart.velocity,f.pole.velocity),(-1,2,-3,4))
 def test_invalid_lane_returns_none(self):
  f=parse_state('STATE 00000000 00000001 1 0001 0000 0000 ffff 00000001 00000000 00000002 00000000')
  self.assertIsNone(f.pole);self.assertEqual(f.cart.position,1)
 def test_uppercase_hex(self):
  self.assertEqual(parse_state(GOOD.upper()).tick,0xffffffff)
 def test_bad_frames_rejected(self):
  invalid=[GOOD+' 0',GOOD.replace(' 3 ',' 4 '),GOOD.replace(' 0001 0002 ',' 4e21 0002 '),GOOD.replace(' ffffffff 00000002',' ffffffff 00300000'),GOOD.replace('12345678','1234567'),GOOD.replace('12345678','+2345678'),'STATE 0 0 0 0','VEL_VALID 1 1','CART_STATE1 1 0',GOOD.replace(' 3 ',' 1 ')]
  for line in invalid:
   with self.subTest(line=line),self.assertRaises(ValueError):parse_state(line)
