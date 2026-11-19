#ifndef EDGE_CARTPOLE_TRAVEL_H
#define EDGE_CARTPOLE_TRAVEL_H
#include "cartpole.h"

typedef struct {
    double reserve_mm, inflight_ms, start_tolerance_mm, max_seconds;
    int plan_only;
} cartpole_travel_options_t;

typedef struct {
    int plan_valid, motion_attempted, zero_requested, has_final_position, stop_confirmed;
    double physical_mm, soft_mm, target_mm, braking_budget_mm, watchdog_budget_mm;
    double zero_position_mm, final_position_mm;
    /* 统计区间以首个完整非零写入结束时刻为起点，以主机接收时间为依据。 */
    int observed_target_95, observed_peak;
    double target_95_s, peak_velocity_mm_s, peak_at_s;
} cartpole_travel_report_t;

/* 规划只读；执行不自动使能或禁用。制动预算不是实际链路延迟或物理停车保证。
 * stop_confirmed 表示收到零速请求之后的新鲜停稳证据，不表示 DISABLED。 */
cartpole_result_t cartpole_travel_velocity(cartpole_t *, double velocity_mm_s,
                                         const cartpole_travel_options_t *,
                                         cartpole_travel_report_t *);
#endif
