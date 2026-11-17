import unittest
from test_model_manifest import config
from model_manifest import training_contract,validate_manifest
from experiment_setup import limits_from_manifest
class V090Test(unittest.TestCase):
 def test_real_training_scale_is_preserved(self):
  c=config();c['actions']['joint_effort'].update(vel_max=.9,acc_max=10.)
  c['events']['randomize_tracking']['params'].update(vel_max_range=[.9,.9],acc_max_range=[10.,10.])
  m={'schema_version':1,**training_contract(c),'training_source':{'association':'operator_selected_checkpoint_and_onnx','checkpoint_name':'candidate_2544.pt','env_yaml_sha256':'a'*64,'checkpoint_sha256':'b'*64}}
  self.assertEqual(validate_manifest(m)['action_scale_m_s'],.9)
  limits=limits_from_manifest(m)
  self.assertEqual(limits.speed_mm_s,900)
if __name__=='__main__':unittest.main()
