#ifndef EDGE_CARTPOLE_INTERNAL_H
#define EDGE_CARTPOLE_INTERNAL_H
#include "cartpole_device.h"
struct cartpole {
    cartpole_device_t device;
    const cartpole_clock_t *clock;
    int stop_requested, callback_depth, callback_violation, operation_active, operation_violation;
    cartpole_cancellation_probe_t cancellation;
    void *cancellation_context;
    cartpole_event_handler_t subscriber, operation_observer;
    void *subscriber_context, *operation_context;
};
#endif
