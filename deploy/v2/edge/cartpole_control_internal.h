#ifndef EDGE_CARTPOLE_CONTROL_INTERNAL_H
#define EDGE_CARTPOLE_CONTROL_INTERNAL_H
#include "cartpole_control.h"
/* 上层操作内部复用；调用者必须已处于 cartpole_execute_operation 作用域。
 * 不访问设备内部状态，不是应用公开 API。 */
cartpole_result_t cartpole_control_run_schedule(cartpole_t *, double, double,
                                               cartpole_tick_handler_t, void *);
#endif
