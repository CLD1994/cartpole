#ifndef EDGE_CONTROL_H
#define EDGE_CONTROL_H
/* 纯控制决策，不访问串口或推理设备。时间为主机接收年龄，不冒充设备采样时间。 */
typedef enum { CONTROL_DRIVE, CONTROL_PAUSE, CONTROL_WAIT, CONTROL_STOP } control_decision;
typedef struct { int paused, stopped, has_time; unsigned long long generation; double last_observed_at; } control_gate;
typedef struct {
    int ready, flags, backlog, io_error;
    double age_s, theta_rad;
} control_input;
control_decision control_check(control_gate *gate, const control_input *in);
control_decision control_observe(control_gate *gate, const control_input *in, double received_at);
double control_velocity(double action, double scale, double limit, double x_m, double track_limit_m);
double control_next_tick(double previous, double now, double period);
#endif
