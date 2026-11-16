"""正式十进制 STATE 的规范格式、范围与无效占位。"""
import unittest
from state_frame import parse_state
GOOD='STATE 3 -2147483648 -3141592 2147483647 -2147483648'
class FrameTests(unittest.TestCase):
 def test_valid_signed_extrema(self):
  f=parse_state(GOOD)
  self.assertEqual((f.cart.position,f.pole.position,f.cart.velocity,f.pole.velocity),(-2147483648,-3141592,2147483647,-2147483648))
 def test_invalid_lane_returns_none(self):
  f=parse_state('STATE 1 1 0 2 0')
  self.assertIsNone(f.pole);self.assertEqual(f.cart.position,1)
 def test_zero_values_can_be_valid(self):
  f=parse_state('STATE 3 0 0 0 0');self.assertIsNotNone(f.cart);self.assertIsNotNone(f.pole)
 def test_bad_frames_rejected(self):
  invalid=[GOOD+' 0','STATE 4 0 0 0 0','STATE 3 2147483648 0 0 0','STATE 3 0 -3141593 0 0','STATE 3 +1 0 0 0','STATE 3 -0 0 0 0','STATE 3 01 0 0 0','STATE 3  0 0 0 0','STATE 3\t0 0 0 0','STATE 3 0 0 0 0 ','STATE 0 1 0 0 0','STATE 1 0 0 0 1','STATE 0 0 0 0','VEL_VALID 1 1','CART_STATE1 1 0','STATE 00000001 3 0000 0000 00000000 00000000 00000000 00000000']
  for line in invalid:
   with self.subTest(line=line),self.assertRaises(ValueError):parse_state(line)

 def test_wire_length_bounds(self):
  shortest='STATE 0 0 0 0 0\n'
  longest='STATE 3 -2147483648 -3141592 -2147483648 -2147483648\n'
  self.assertEqual(len(shortest),16);self.assertEqual(len(longest),53)
  parse_state(shortest[:-1]);parse_state(longest[:-1])
