#ifndef EDGE_CARTPOLE_INTERNAL_H
#define EDGE_CARTPOLE_INTERNAL_H
#include "cartpole_device.h"
struct cartpole {
    cartpole_device_t device;
    cartpole_runtime_t *runtime;
    cartpole_event_handler_t subscriber;
    void *subscriber_context;
    cartpole_event_handler_t controller_observer;
    void *controller_context;
};
cartpole_receive_result_t cartpole_receive(cartpole_t *);
cartpole_send_result_t cartpole_send_velocity(cartpole_t *, double, double, int);
cartpole_result_t cartpole_stream_result(cartpole_t *);
void cartpole_set_control_observer(cartpole_t *, cartpole_event_handler_t, void *);
int cartpole_callback_enter(cartpole_t *);
int cartpole_callback_leave(cartpole_t *);
int cartpole_loop_begin(cartpole_t *);
void cartpole_loop_end(cartpole_t *);
void cartpole_sleep_to(cartpole_t *, double);
#endif
