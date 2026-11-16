"""模型配置契约的纯单元测试；不加载仿真或真实文件。"""
import copy
import unittest
from unittest.mock import patch
import model_manifest as m


def config():
    return {'sim': {'dt': 0.0005}, 'decimation': 5,
            'actions': {'joint_effort': {'class_type': 'cartpole.tasks.manager_based.cartpole.mdp.vel_tracking_action:JointVelocityTrackingAction', 'joint_names': ['slider_to_cart'], 'vel_max': 1.25, 'acc_max': 9.0}},
            'observations': {'policy': {'joint_pos_rel': {'func': 'isaaclab.envs.mdp.observations:joint_pos_rel'}, 'joint_vel_rel': {'func': 'cartpole.tasks.manager_based.cartpole.mdp.velocity_window_observation:JointVelocityWindow', 'params': {'window_s': .016}}, 'concatenate_terms': True}},
            'events': {'randomize_tracking': {'params': {'vel_max_range': [1.25, 1.25], 'acc_max_range': [8.0, 9.0]}}}}


class TrainingContractTest(unittest.TestCase):
    def test_uses_saved_values_not_recipe_name(self):
        c = config()
        self.assertEqual(m.training_contract(c)['training_control_hz'], 400)
        self.assertEqual(m.training_contract(c)['training_acceleration_m_s2'], 9.0)
        c['sim']['dt'] = 0.001; c['decimation'] = 2
        c['actions']['joint_effort']['acc_max'] = 10
        c['events']['randomize_tracking']['params']['acc_max_range'] = [8, 10]
        self.assertEqual(m.training_contract(c)['training_control_hz'], 500)
        self.assertEqual(m.training_contract(c)['training_acceleration_m_s2'], 10)

    def test_unresolved_module_reference_is_supported(self):
        c = config(); c['actions']['joint_effort']['class_type'] = '{DIR}.vel_tracking_action:JointVelocityTrackingAction'
        self.assertEqual(m.training_contract(c)['action_dim'], 1)

    def test_unsupported_speed_rejected(self):
        c = config(); c['actions']['joint_effort']['vel_max'] = 2
        with self.assertRaises(ValueError): m.training_contract(c)

    def test_missing_acceleration_is_not_inferred(self):
        c = config(); del c['actions']['joint_effort']['acc_max']
        with self.assertRaisesRegex(ValueError, 'training_configuration'):
            m.training_contract(c)

    def test_extra_action_rejected(self):
        c = config(); c['actions']['acceleration'] = {'scale': 9}
        with self.assertRaisesRegex(ValueError, 'single_action'):
            m.training_contract(c)

    def test_observation_order_not_guessed(self):
        c = config(); p = c['observations']['policy']
        c['observations']['policy'] = {'joint_vel_rel': p['joint_vel_rel'], 'joint_pos_rel': p['joint_pos_rel'], 'concatenate_terms': True}
        with self.assertRaisesRegex(ValueError, 'observation'):
            m.training_contract(c)

    def test_randomization_cannot_change_target_scale(self):
        c = config(); c['events']['randomize_tracking']['params']['vel_max_range'] = [1, 2]
        with self.assertRaisesRegex(ValueError, 'randomization'):
            m.training_contract(c)

    def test_unknown_observation_function_rejected(self):
        c = config(); c['observations']['policy']['joint_vel_rel']['func'] = 'unknown:JointVelocityWindow'
        with self.assertRaisesRegex(ValueError, 'observation'):
            m.training_contract(c)

    def test_nonfinite_config_rejected(self):
        c = config(); c['sim']['dt'] = float('nan')
        with self.assertRaises(ValueError): m.training_contract(c)

    def test_observation_transforms_rejected(self):
        for target, key, value in [('term', 'clip', [-1, 1]), ('term', 'modifiers', [{'func': 'unknown'}]), ('group', 'concatenate_dim', 0), ('group', 'flatten_history_dim', False)]:
            with self.subTest(key=key):
                c = config(); group = c['observations']['policy']
                (group['joint_pos_rel'] if target == 'term' else group)[key] = value
                with self.assertRaisesRegex(ValueError, 'observation'): m.training_contract(c)

    def test_action_transform_rejected(self):
        for key, value in [('scale', 100), ('offset', 1), ('clip', {'slider_to_cart': [-.1, .1]}), ('asset_name', 'other')]:
            with self.subTest(key=key):
                c = config(); c['actions']['joint_effort'][key] = value
                with self.assertRaisesRegex(ValueError, 'action'): m.training_contract(c)

    def test_changed_joint_selection_rejected(self):
        c = config(); c['observations']['policy']['joint_pos_rel']['params'] = {'asset_cfg': {'joint_names': ['cart_to_pole', 'slider_to_cart']}}
        with self.assertRaisesRegex(ValueError, 'observation'): m.training_contract(c)

    def test_group_history_rejected(self):
        c = config(); c['observations']['policy']['history_length'] = 2
        with self.assertRaisesRegex(ValueError, 'observation'): m.training_contract(c)

    def test_manifest_requires_explicit_source_association(self):
        manifest = {'schema_version': 1, **m.training_contract(config())}
        with self.assertRaisesRegex(ValueError, 'source'): m.validate_manifest(manifest)

    def test_load_checks_model_files(self):
        import json
        manifest = {'schema_version': 1, **m.training_contract(config()), 'training_source': {
            'association': 'operator_selected_checkpoint_and_onnx', 'checkpoint_name': 'candidate.pt',
            'env_yaml_sha256': 'a'*64, 'checkpoint_sha256': 'b'*64}}
        with patch('model_manifest.Path.read_text', return_value=json.dumps(manifest)), patch('model_manifest.verify_model_files', side_effect=ValueError('onnx_hash_mismatch')):
            with self.assertRaisesRegex(ValueError, 'onnx_hash_mismatch'): m.load_manifest('model.json', 'policy.onnx')


if __name__ == '__main__': unittest.main()
