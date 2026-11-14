"""训练任务的区间约束；不依赖 Isaac Lab，也不改变设备档案。"""
import math


def validate_training_ranges(*, soft_limit_m, boundary_inset_m,
                             initial_position_range, acceleration_range):
    """校验训练初始化与随机化范围，返回对称的位置终止边界。"""
    initial_lo, initial_hi = initial_position_range
    acceleration_lo, acceleration_hi = acceleration_range
    values = (soft_limit_m, boundary_inset_m, initial_lo, initial_hi,
              acceleration_lo, acceleration_hi)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
           for v in values):
        raise ValueError('训练区间参数必须为有限数值')
    if not 0 <= boundary_inset_m < soft_limit_m:
        raise ValueError('训练内缩量必须非负且小于软限位边界')
    limit = soft_limit_m - boundary_inset_m
    if not -limit <= initial_lo <= initial_hi <= limit:
        raise ValueError('初始位置区间必须有序且完整位于训练终止边界内')
    if not 0 < acceleration_lo <= acceleration_hi:
        raise ValueError('加速度随机化区间必须为有序正数')
    return limit
