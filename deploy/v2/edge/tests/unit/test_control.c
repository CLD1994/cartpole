#include "control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static control_input good(void) {
    control_input x = {0}; x.ready=1; x.flags=3; x.age_s=.001; x.theta_rad=0; return x;
}
int main(void) {
    control_gate g={0}; control_input x=good();
    assert(control_check(&g,&x)==CONTROL_DRIVE);
    x.theta_rad=20*M_PI/180.; assert(control_check(&g,&x)==CONTROL_DRIVE);
    x.theta_rad=20.01*M_PI/180.; assert(control_check(&g,&x)==CONTROL_PAUSE);
    x.theta_rad=19*M_PI/180.; assert(control_check(&g,&x)==CONTROL_PAUSE);
    x.theta_rad=18*M_PI/180.; assert(control_check(&g,&x)==CONTROL_DRIVE);
    x=good(); x.flags=1; assert(control_check(&g,&x)==CONTROL_PAUSE);
    x.flags=3; assert(control_check(&g,&x)==CONTROL_DRIVE);
    x.flags=2; assert(control_check(&g,&x)==CONTROL_STOP);
    x=good(); assert(control_check(&g,&x)==CONTROL_STOP); /* 失效不能被下一帧覆盖 */
    g=(control_gate){0};
    x=good(); x.ready=0; assert(control_check(&g,&x)==CONTROL_STOP);
    g=(control_gate){0}; x=good(); x.age_s=.100; assert(control_check(&g,&x)==CONTROL_STOP);
    g=(control_gate){0}; x=good(); x.age_s=.021; assert(control_check(&g,&x)==CONTROL_WAIT);
    x=good(); x.backlog=1; assert(control_check(&g,&x)==CONTROL_WAIT);
    x=good(); x.io_error=1; assert(control_check(&g,&x)==CONTROL_STOP);
    g=(control_gate){0}; x=good(); x.theta_rad=NAN; assert(control_check(&g,&x)==CONTROL_STOP);
    g=(control_gate){0}; x=good();
    assert(control_observe(&g,&x,1.)==CONTROL_DRIVE);
    assert(control_observe(&g,&x,1.101)==CONTROL_STOP); /* 恢复帧不掩盖停流 */
    g=(control_gate){0}; x=good(); control_observe(&g,&x,1.);
    unsigned long long generation=g.generation;
    x.theta_rad=.4; control_observe(&g,&x,1.002);
    x.theta_rad=0; control_observe(&g,&x,1.004);
    assert(g.generation!=generation); /* 即使同批恢复，旧推理候选也应丢弃 */
    assert(control_calculate_velocity(2,900,800,0,.4)==800);
    assert(control_calculate_velocity(-1,900,900,-.41,.4)==0);
    assert(control_calculate_velocity(1,900,900,-.41,.4)==900);
    assert(isnan(control_calculate_velocity(NAN,900,900,0,.4)));
    assert(fabs(control_calculate_next_tick(1.,1.020,.0025)-1.0225)<1e-8);
    puts("control: passed");
}
