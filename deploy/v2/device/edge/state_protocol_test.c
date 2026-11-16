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

static void test_values(void) {
    struct link l={0};device_session*s=prepared(&l);
    assert(device_session_feed(s,"STATE 3 -2147483648 -3141592 2147483647 -2147483648",1));
    device_snapshot v=device_session_snapshot(s);
    assert(v.state.available && v.state.x_um==INT32_MIN && v.state.theta_urad==-3141592);
    assert(v.state.vx_mm_s==INT32_MAX && v.state.vtheta_mrad_s==INT32_MIN);
    assert(v.state.received_s==1);device_session_destroy(s);
}
static void test_invalid_lanes(void) {
    const char *frames[]={"STATE 0 0 0 0 0","STATE 1 1 0 2 0","STATE 2 0 1 0 2"};
    for(unsigned i=0;i<3;++i){
        struct link l={0};device_session*s=prepared(&l);
        assert(device_session_feed(s,frames[i],1));
        assert(!device_session_snapshot(s).state.available);
        assert(!device_session_start(s,1));device_session_destroy(s);
    }
}
static void test_malformed(void) {
    const char *bad[]={
      "STATE 0 0 0 0","VEL_VALID 1 1","CART_STATE1 1 0",
      "STATE 3 2147483648 0 0 0","STATE 3 0 3141593 0 0",
      "STATE 3 0 0 -2147483649 0","STATE 4 0 0 0 0",
      "STATE 1 0 1 0 0","STATE 2 1 0 0 0",
      "STATE 3 +1 0 0 0","STATE 3 01 0 0 0","STATE 3 -0 0 0 0",
      "STATE  3 0 0 0 0","STATE\t3 0 0 0 0","STATE 3 0 0 0 0 ",
      "STATE 000003e8 3 0000 0000 00000000 00000000 00000000 00000000"
    };
    for(unsigned i=0;i<sizeof bad/sizeof bad[0];++i){
        struct link l={0};device_session*s=prepared(&l);
        assert(!device_session_feed(s,bad[i],1));
        assert(device_session_snapshot(s).error==DEVICE_INVALID_LINE);
        assert(device_session_snapshot(s).phase==DEVICE_FINISHED);
        assert(l.sent==0);device_session_destroy(s);
    }
}
static void test_receipt_freshness_only(void) {
    struct link l={0};device_session*s=prepared(&l);
    assert(device_session_feed(s,"STATE 3 1 -1 2 -2",1));
    assert(device_session_start(s,1));
    assert(device_session_feed(s,"STATE 3 1 -1 2 -2",1.05));
    assert(device_session_snapshot(s).state.received_s==1.05);
    device_session_tick(s,1.151);
    assert(device_session_snapshot(s).phase==DEVICE_STOPPING);device_session_destroy(s);
}
static void test_invalid_stops_active(void) {
    struct link l={0};device_session*s=prepared(&l);
    assert(device_session_feed(s,"STATE 3 1 -1 2 -2",1));assert(device_session_start(s,1));
    assert(!device_session_feed(s,"STATE 1 1 0 2 0",1.001));
    assert(device_session_snapshot(s).phase==DEVICE_STOPPING);
    assert(!device_session_snapshot(s).state.available);
    assert(!device_session_feed(s,"STATE 3 1 -1 2 -2",1.002));
    assert(device_session_snapshot(s).phase==DEVICE_STOPPING);device_session_destroy(s);
}
static void test_observing_recovers(void) {
    struct link l={0};device_session*s=prepared(&l);
    assert(device_session_feed(s,"STATE 1 1 0 2 0",1));
    assert(device_session_feed(s,"STATE 3 1 -1 2 -2",1.001));
    assert(device_session_start(s,1.001));device_session_destroy(s);
}
static bool pending(void*p,size_t*n){*n=strlen(((struct link*)p)->bytes);return true;}
static bool read_bytes(void*p,unsigned char*out,size_t n,size_t*got){struct link*l=p;memcpy(out,l->bytes,n);l->bytes+=n;*got=n;return true;}
static double clock_now(void*p){return ((struct link*)p)->now;}
static void integration(void){
    struct link l={0,"",1};device_session*s=prepared(&l);
    device_receive_limits limits={1024,256,.02,.01,.02};
    device_receiver*r=device_receiver_create(s,limits,pending,read_bytes,clock_now,&l);
    l.bytes="STATE 3 1 -1 ";assert(device_receiver_poll(r));assert(!device_session_snapshot(s).state.available);
    l.bytes="2 -2\n";l.now=1.001;assert(device_receiver_poll(r));assert(device_session_start(s,1.001));
    l.bytes="STATE 1 1 0 2 0\n";l.now=1.002;(void)device_receiver_poll(r);
    assert(device_session_snapshot(s).phase==DEVICE_STOPPING);
    device_receiver_destroy(r);device_session_destroy(s);
}
int main(int argc,char**argv){
    (void)argv;if(argc>1){integration();puts("STATE receiver/session integration passed");return 0;}
    test_values();test_invalid_lanes();test_malformed();test_receipt_freshness_only();
    test_invalid_stops_active();test_observing_recovers();puts("STATE protocol: 6 unit groups passed");return 0;
}
