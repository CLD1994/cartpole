"""模型准备工具的边界测试；无文件、无仿真、无串口。"""
import unittest
import prepare_model as p


class PrepareTest(unittest.TestCase):
    def test_python_tuple_tag_is_inert_data(self):
        data = p.read_training_yaml('values: !!python/tuple [8.0, 9.0]\n')
        self.assertEqual(data, {'values': [8.0, 9.0]})

    def test_slice_tag_is_inert_data(self):
        data = p.read_training_yaml('ids: !!python/object/apply:builtins.slice [null, null, null]\n')
        self.assertEqual(data, {'ids': {'slice': [None, None, None]}})

    def test_yaml_object_execution_tags_rejected(self):
        with self.assertRaises(ValueError):
            p.read_training_yaml('x: !!python/object/apply:os.system [echo nope]')

    def test_policy_shape_accepts_dynamic_batch(self):
        p.validate_policy_shape([[None, 4]], [[None, 1]])

    def test_dual_output_rejected(self):
        with self.assertRaisesRegex(ValueError, 'shape'):
            p.validate_policy_shape([[None, 4]], [[None, 2]])

    def test_unknown_width_rejected(self):
        with self.assertRaisesRegex(ValueError, 'shape'):
            p.validate_policy_shape([[None, None]], [[None, 1]])


if __name__ == '__main__': unittest.main()
