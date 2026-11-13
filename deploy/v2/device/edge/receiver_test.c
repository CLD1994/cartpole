#include "receiver.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
/* 以公开session.h约束替身；不链接或检查会话内部实现。 */
struct session_spy { unsigned feeds, aborts, ticks; char last[257]; };
void device_session_tick(device_session *s,double t) {(void)t; ((struct session_spy *)s)->ticks++;}
void device_session_abort(device_session *s,double t) {(void)t; ((struct session_spy *)s)->aborts++;}
bool device_session_feed(device_session *s,const char *line,double t) {
    (void)t; struct session_spy *spy=(struct session_spy *)s;
    spy->feeds++; snprintf(spy->last,sizeof(spy->last),"%s",line); return true;
}
struct source { const char *data; double now; bool broken; double delay; size_t reads; };
static bool pending(void *ctx,size_t *n) {struct source *s=ctx; *n=strlen(s->data); return !s->broken;}
static bool read_bytes(void *ctx,unsigned char *out,size_t n,size_t *got) {
    struct source *s=ctx; memcpy(out,s->data,n); s->data+=n; *got=n; s->now+=s->delay; s->reads++; return true;
}
static double clock_now(void *ctx) {return ((struct source *)ctx)->now;}
static device_receiver *receiver(struct session_spy *s,struct source *w) {
    device_receive_limits l={1024,256,.02,.01,.02};
    return device_receiver_create((device_session *)s,l,pending,read_bytes,clock_now,w);
}
static void test_fragments(void) {
    struct source w={"STATE 1",1,false,0,0}; struct session_spy s={0}; device_receiver *r=receiver(&s,&w);
    assert(device_receiver_poll(r)); assert(!s.feeds);
    w.data=" 2 3 4\r\nVEL_VALID 1 1\n"; assert(device_receiver_poll(r));
    assert(s.feeds==2); assert(!strcmp(s.last,"VEL_VALID 1 1")); assert(w.reads==2);
    device_receiver_destroy(r);
}
static void test_bad_batch_and_resync(void) {
    struct source w={"STATE 1 2 3 4\n\xff",1,false,0,0}; struct session_spy s={0}; device_receiver *r=receiver(&s,&w);
    assert(!device_receiver_poll(r)); assert(device_receiver_error(r)==RECEIVE_INVALID_ASCII);
    assert(s.aborts==1 && !s.feeds);
    w.data="discard\nMODE DISABLED NONE\n"; assert(device_receiver_poll(r));
    assert(s.feeds==1 && !strcmp(s.last,"MODE DISABLED NONE"));
    assert(device_receiver_error(r)==RECEIVE_INVALID_ASCII); device_receiver_destroy(r);
}
static void test_deadlines(void) {
    struct source w={"",1,false,0,0}; struct session_spy s={0}; device_receiver *r=receiver(&s,&w);
    assert(device_receiver_poll(r)); w.now+=.03; w.data="STATE 1 2 3 4\n";
    assert(!device_receiver_poll(r)); assert(device_receiver_error(r)==RECEIVE_POLL_DEADLINE);
    assert(!w.reads && !s.feeds); device_receiver_destroy(r);
}
static void test_read_deadline(void) {
    struct source w={"STATE 1 2 3 4\n",1,false,.011,0}; struct session_spy s={0}; device_receiver *r=receiver(&s,&w);
    assert(!device_receiver_poll(r)); assert(device_receiver_error(r)==RECEIVE_READ_DEADLINE);
    assert(!s.feeds); device_receiver_destroy(r);
}
static void test_io_error(void) {
    struct source w={"",1,true,0,0}; struct session_spy s={0}; device_receiver *r=receiver(&s,&w);
    assert(!device_receiver_poll(r)); assert(device_receiver_error(r)==RECEIVE_IO); device_receiver_destroy(r);
}
static void test_partial_timeout(void) {
    struct source w={"STATE",1,false,0,0}; struct session_spy s={0}; device_receiver *r=receiver(&s,&w);
    assert(device_receiver_poll(r)); w.now+=.015; assert(device_receiver_poll(r)); w.now+=.015;
    assert(!device_receiver_poll(r)); assert(device_receiver_error(r)==RECEIVE_PARTIAL_TIMEOUT); device_receiver_destroy(r);
}
static void test_size_limits(void) {
    char data[1026]; memset(data,'x',sizeof(data)-1); data[1025]=0;
    struct source w={data,1,false,0,0}; struct session_spy s={0}; device_receiver *r=receiver(&s,&w);
    assert(!device_receiver_poll(r)); assert(device_receiver_error(r)==RECEIVE_BACKLOG); assert(!w.reads);
    device_receiver_destroy(r); data[257]=0; r=receiver(&s,&w);
    assert(!device_receiver_poll(r)); assert(device_receiver_error(r)==RECEIVE_LINE_OVERFLOW); assert(!s.feeds);
    device_receiver_destroy(r);
}
static void test_read_crosses_partial_deadline(void) {
    struct source w={"STATE ",1,false,0,0}; struct session_spy s={0}; device_receiver *r=receiver(&s,&w);
    assert(device_receiver_poll(r)); w.now+=.019; w.delay=.009; w.data="1 2 3 4\n";
    assert(!device_receiver_poll(r)); assert(device_receiver_error(r)==RECEIVE_PARTIAL_TIMEOUT);
    assert(!s.feeds); device_receiver_destroy(r);
}
int main(void) {test_fragments();test_bad_batch_and_resync();test_deadlines();test_read_deadline();test_io_error();test_partial_timeout();test_size_limits();test_read_crosses_partial_deadline();puts("receiver: 8 tests passed");}
