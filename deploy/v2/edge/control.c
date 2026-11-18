#include "control.h"
#include <math.h>

control_decision control_check(control_gate *gate, const control_input *in)
{
    if (gate->stopped) return CONTROL_STOP;
    const double radians = 3.14159265358979323846 / 180.0;
    if (in->io_error || !in->ready || !(in->flags & 1) ||
        !isfinite(in->age_s) || in->age_s < 0 || in->age_s >= .100)
        { gate->stopped=1; gate->generation++; return CONTROL_STOP; }
    /* 未排空的输入不能被当成最新观测。未发命令期间仍由固件看门狗制动。 */
    if (in->backlog || in->age_s > .020) return CONTROL_WAIT;
    if (!(in->flags & 2)) { if (!gate->paused) gate->generation++; gate->paused=1; return CONTROL_PAUSE; }
    if (!isfinite(in->theta_rad)) { gate->stopped=1; gate->generation++; return CONTROL_STOP; }
    int was_paused=gate->paused;
    const double a = fabs(in->theta_rad);
    if (a > 20.0*radians) gate->paused=1;
    else if (a <= 18.0*radians) gate->paused=0;
    if (gate->paused!=was_paused) gate->generation++;
    return gate->paused ? CONTROL_PAUSE : CONTROL_DRIVE;
}

control_decision control_observe(control_gate *gate, const control_input *in, double received_at)
{
    if (gate->has_time && (received_at < gate->last_observed_at || received_at-gate->last_observed_at >= .100)) {
        gate->stopped=1; gate->generation++;
    }
    if (received_at>=0) { gate->last_observed_at=received_at; gate->has_time=1; }
    return control_check(gate,in);
}

double control_calculate_velocity(double action, double scale, double limit, double x_m, double track_limit_m)
{
    if (!isfinite(action) || !isfinite(scale) || !isfinite(limit) ||
        !isfinite(x_m) || !isfinite(track_limit_m) || scale <= 0 || limit <= 0 || track_limit_m <= 0)
        return NAN;
    double v = fmax(-1., fmin(1., action)) * scale;
    v = fmax(-limit, fmin(limit,v));
    if ((x_m >= track_limit_m && v > 0) || (x_m <= -track_limit_m && v < 0)) v=0;
    return v;
}

double control_calculate_next_tick(double previous, double now, double period)
{
    /* 跳过已错过的格点，不补发历史动作。 */
    if (previous > now) return previous;
    return previous + (floor((now-previous)/period)+1)*period;
}
