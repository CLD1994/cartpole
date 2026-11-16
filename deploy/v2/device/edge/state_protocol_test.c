#include "session.h"
#include "receiver.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

struct link { unsigned sent; const char *bytes; double now; };
static bool send_line(void *p, const char *line) { (void)line; ++((struct link *)p)->sent; return true; }
static device_session *prepared(struct link *p) {
    device_limits limits = {1250, 10000, 10000, 100, .1, .3, 1, 1};
    device_session *s = device_session_create(limits, send_line, p);
    const char *meta[] = {"MODE READY NONE", "OK CAL_STATE 2", "OK CAL_LENGTH_UM 853940", "OK CAL_SAVED 1", "OK CAL_STORAGE 0", "OK VEL_MAX 1250", "OK ACC_MAX 10000", "OK WDT_MS 100", "OK X_MARGIN_UM 10000"};
    for (unsigned i = 0; i < sizeof meta / sizeof meta[0]; ++i) assert(device_session_feed(s, meta[i], 1));
    return s;
}
static const char *frame(unsigned boot, unsigned tick, unsigned seq, unsigned age) {
    static char line[160];
    snprintf(line, sizeof line, "STATE %08x %08x 3 %04x %04x %04x %04x 00000001 ffffffff 00000002 fffffffe", boot, tick, seq, seq, age, age);
    return line;
}
static void test_fixed_fields_and_units(void) {
    struct link l = {0}; device_session *s = prepared(&l);
    assert(device_session_feed(s, frame(1, 1000, 65535, 1000), 1));
    device_snapshot v = device_session_snapshot(s);
    assert(v.state.x_um == 1 && v.state.theta_urad == -1 && v.state.vx_mm_s == 2 && v.state.vtheta_mrad_s == -2);
    assert(v.state.received_s > .9989 && v.state.received_s < .9991);
    device_session_destroy(s);
}
static void test_invalid_lane_never_yields_observation(void) {
    const char *frames[] = {
        "STATE 00000001 000003e8 1 0001 0000 0000 ffff 00000000 00000000 00000000 00000000",
        "STATE 00000001 000003e8 2 0000 0001 ffff 0000 00000000 00000000 00000000 00000000",
        "STATE 00000001 000003e8 0 0000 0000 ffff ffff 00000000 00000000 00000000 00000000"};
    for (unsigned i=0;i<3;++i) { struct link l={0};device_session *s=prepared(&l);assert(device_session_feed(s,frames[i],1));assert(!device_session_snapshot(s).state.available);assert(!device_session_start(s,1));device_session_destroy(s); }
}
static void test_malformed_inputs(void) {
    const char *frames[] = {
        "STATE 0 0 0 0", "VEL_VALID 1 1", "CART_STATE1 1 0",
        "STATE  00000001 000003e8 3 0001 0001 0000 0000 00000001 ffffffff 00000002 fffffffe",
        "STATE\t00000001 000003e8 3 0001 0001 0000 0000 00000001 ffffffff 00000002 fffffffe",
        "STATE 00000001 000003e8 3 0001 0001 0000 0000 00000001 ffffffff 00000002 fffffffe ",
        "STATE 00000001 000003e8 4 0001 0001 0000 0000 00000000 00000000 00000000 00000000",
        "STATE 00000001 000003e8 3 0001 0001 4e21 0000 00000000 00000000 00000000 00000000",
        "STATE 00000001 000003e8 0 0001 0000 ffff ffff 00000000 00000000 00000000 00000000",
        "STATE 00000001 000003e8 0 0000 0000 ffff ffff 00000001 00000000 00000000 00000000",
        "STATE 00000001 000003e8 3 0001 0001 0000 0000 00000000 00300000 00000000 00000000",
        "STATE 00000001 000003e8 3 +001 0001 0000 0000 00000000 00000000 00000000 00000000"};
    for (unsigned i=0;i<sizeof frames/sizeof frames[0];++i) {struct link l={0};device_session *s=prepared(&l);assert(!device_session_feed(s,frames[i],1));assert(!device_session_snapshot(s).state.available);assert(device_session_snapshot(s).error==DEVICE_INVALID_LINE);assert(device_session_snapshot(s).phase==DEVICE_FINISHED);assert(l.sent==0);device_session_destroy(s);}
}
static void test_tick_and_sequence_wrap(void) {
    struct link l={0};device_session *s=prepared(&l);
    assert(device_session_feed(s,frame(1,0xfffffff0,65535,0),1));assert(device_session_start(s,1));
    assert(device_session_feed(s,frame(1,0x10,0,0),1.000032));assert(device_session_snapshot(s).phase==DEVICE_ACTIVE);device_session_destroy(s);
}
static void test_duplicate_does_not_renew(void) {
    struct link l={0};device_session *s=prepared(&l);assert(device_session_feed(s,frame(1,1000,1,0),1));assert(device_session_start(s,1));
    assert(device_session_feed(s,frame(1,2000,1,1000),1.05));assert(device_session_snapshot(s).state.received_s==1);
    device_session_tick(s,1.101);assert(device_session_snapshot(s).phase==DEVICE_STOPPING);device_session_destroy(s);
}
static void test_reboot_and_regression_stop(void) {
    for(unsigned i=0;i<3;++i){struct link l={0};device_session *s=prepared(&l);assert(device_session_feed(s,frame(1,1000,2,0),1));assert(device_session_start(s,1));
    assert(!device_session_feed(s,frame(i==0?2:1,i==1?999:2000,i==2?1:3,0),1.01));assert(device_session_snapshot(s).phase==DEVICE_STOPPING);device_session_destroy(s);}
}
static void test_relative_backlog_counts_in_age(void) {
    struct link l={0};device_session *s=prepared(&l);assert(device_session_feed(s,frame(1,1000,1,0),1));assert(device_session_start(s,1));
    assert(device_session_feed(s,frame(1,2000,2,0),1.05));
    assert(device_session_snapshot(s).state.received_s<1.002);
    assert(device_session_feed(s,frame(1,3000,3,0),1.09));
    device_session_tick(s,1.104);assert(device_session_snapshot(s).phase==DEVICE_STOPPING);device_session_destroy(s);
}
static bool pending(void *p,size_t *n){*n=strlen(((struct link*)p)->bytes);return true;}
static bool read_bytes(void *p,unsigned char *out,size_t n,size_t *got){struct link *l=p;memcpy(out,l->bytes,n);l->bytes+=n;*got=n;return true;}
static double clock_now(void *p){return ((struct link*)p)->now;}
static void test_receiver_session_integration(void) {
    struct link l={0,"",1};device_session *s=prepared(&l);device_receive_limits limits={1024,256,.02,.01,.02};device_receiver *r=device_receiver_create(s,limits,pending,read_bytes,clock_now,&l);
    l.bytes="STATE 00000001 000003e8 3 0001 0001 0000 0000 ";assert(device_receiver_poll(r));assert(!device_session_snapshot(s).state.available);
    l.bytes="00000001 ffffffff 00000002 fffffffe\n";l.now=1.001;assert(device_receiver_poll(r));assert(device_session_start(s,1.001));
    l.bytes="STATE 00000001 000007d0 1 0002 0000 0000 ffff 00000000 00000000 00000000 00000000\n";l.now=1.002;(void)device_receiver_poll(r);assert(device_session_snapshot(s).phase==DEVICE_STOPPING);
    device_receiver_destroy(r);device_session_destroy(s);
}
static void test_age_and_duplicate_tick(void) {
    struct link l={0};device_session *s=prepared(&l);
    assert(device_session_feed(s,frame(1,1000,1,20000),1));
    assert(device_session_snapshot(s).state.received_s < .981);
    assert(device_session_start(s,1));
    assert(!device_session_feed(s,frame(1,1000,2,0),1.01));
    assert(device_session_snapshot(s).phase==DEVICE_STOPPING);
    assert(device_session_snapshot(s).error==DEVICE_INVALID_MEASUREMENT);
    device_session_destroy(s);
}
static void test_one_duplicate_lane_does_not_renew(void) {
    struct link l={0};device_session *s=prepared(&l);assert(device_session_feed(s,frame(1,1000,1,0),1));
    assert(device_session_start(s,1));
    assert(device_session_feed(s,"STATE 00000001 000007d0 3 0001 0002 03e8 0000 00000001 ffffffff 00000002 fffffffe",1.001));
    assert(device_session_snapshot(s).state.received_s==1);device_session_destroy(s);
}
static void test_reboot_before_start_requires_new_session(void) {
    struct link l={0};device_session *s=prepared(&l);assert(device_session_feed(s,frame(1,1000,1,0),1));
    (void)device_session_feed(s,frame(2,2000,2,0),1.001);
    (void)device_session_feed(s,frame(2,3000,3,0),1.002);
    assert(!device_session_start(s,1.002));device_session_destroy(s);
}
static void test_sample_watermarks(void) {
    const char *bad[] = {
      "STATE 00000001 000007d0 3 0002 0002 07d0 0000 00000001 ffffffff 00000002 fffffffe",
      "STATE 00000001 000007d0 3 0001 0001 0000 0000 00000001 ffffffff 00000002 fffffffe",
      "STATE 00000001 000007d0 3 0001 0001 03e8 03e8 00000003 ffffffff 00000002 fffffffe"
    };
    for(unsigned i=0;i<3;++i){
      struct link l={0};device_session *s=prepared(&l);
      assert(device_session_feed(s,frame(1,1000,1,0),1));
      assert(!device_session_feed(s,bad[i],1.001));
      assert(device_session_snapshot(s).phase==DEVICE_FINISHED);
      assert(device_session_snapshot(s).error==DEVICE_INVALID_MEASUREMENT);
      assert(l.sent==0);device_session_destroy(s);
    }
    struct link l={0};device_session *s=prepared(&l);
    assert(device_session_feed(s,frame(1,1000,2,0),1));
    assert(device_session_feed(s,"STATE 00000001 000005dc 0 0000 0000 ffff ffff 00000000 00000000 00000000 00000000",1.0005));
    assert(!device_session_feed(s,frame(1,2000,1,0),1.001));
    assert(device_session_snapshot(s).phase==DEVICE_FINISHED);device_session_destroy(s);
}
int main(int argc, char **argv) {
    (void)argv;
    if (argc > 1) {test_receiver_session_integration();puts("STATE receiver/session integration passed");return 0;}
    test_sample_watermarks();test_fixed_fields_and_units();test_invalid_lane_never_yields_observation();test_malformed_inputs();
    test_tick_and_sequence_wrap();test_duplicate_does_not_renew();test_reboot_and_regression_stop();
    test_relative_backlog_counts_in_age();test_age_and_duplicate_tick();test_one_duplicate_lane_does_not_renew();
    test_reboot_before_start_requires_new_session();puts("STATE protocol: 11 unit groups passed");return 0;
}
