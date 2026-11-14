#include "receiver.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
struct device_receiver {
    device_session *session;
    device_receive_limits limits;
    device_pending_fn pending;
    device_read_fn read;
    device_clock_fn clock;
    void *context;
    unsigned char *buffer;
    size_t used;
    double last, partial_since;
    bool polled, partial, desync;
    device_receive_error error;
};
device_receiver *device_receiver_create(device_session *s, device_receive_limits l,
        device_pending_fn pending, device_read_fn read, device_clock_fn clock, void *ctx) {
    if (!s || !pending || !read || !clock || !l.max_line || l.max_line > l.max_pending ||
        l.max_pending > 1048576 || !isfinite(l.max_poll_s) || l.max_poll_s <= 0 ||
        !isfinite(l.max_read_s) || l.max_read_s <= 0 ||
        !isfinite(l.partial_timeout_s) || l.partial_timeout_s <= 0) return NULL;
    device_receiver *r = calloc(1, sizeof(*r));
    if (!r) return NULL;
    r->buffer = malloc(l.max_pending + l.max_line + 1);
    if (!r->buffer) { free(r); return NULL; }
    r->session=s; r->limits=l; r->pending=pending; r->read=read; r->clock=clock; r->context=ctx;
    return r;
}
void device_receiver_destroy(device_receiver *r) { if(r) { free(r->buffer); free(r); } }
device_receive_error device_receiver_error(const device_receiver *r) { return r->error; }
static bool fail(device_receiver *r, device_receive_error e, double now) {
    if (r->error == RECEIVE_OK) r->error=e;
    r->used=0; r->partial=false; r->desync=true;
    device_session_abort(r->session, now);
    return false;
}
bool device_receiver_poll(device_receiver *r) {
    double now=r->clock(r->context);
    device_session_tick(r->session, now);
    double gap=now-r->last; r->last=now;
    bool bad=r->polled && (gap < 0 || gap > r->limits.max_poll_s);
    r->polled=true;
    if (!isfinite(now) || bad) return fail(r, RECEIVE_POLL_DEADLINE, now);
    if (r->partial && now-r->partial_since > r->limits.partial_timeout_s)
        return fail(r, RECEIVE_PARTIAL_TIMEOUT, now);
    size_t count=0, got=0, remaining=0;
    if (!r->pending(r->context, &count)) return fail(r, RECEIVE_IO, now);
    if (count > r->limits.max_pending) return fail(r, RECEIVE_BACKLOG, now);
    if (count && !r->read(r->context, r->buffer+r->used, count, &got))
        return fail(r, RECEIVE_IO, r->clock(r->context));
    if (!r->pending(r->context, &remaining)) return fail(r, RECEIVE_IO, r->clock(r->context));
    double end=r->clock(r->context);
    device_session_tick(r->session, end);
    if (!isfinite(end) || end < now || end-now > r->limits.max_read_s)
        return fail(r, RECEIVE_READ_DEADLINE, end);
    if (r->partial && end-r->partial_since > r->limits.partial_timeout_s)
        return fail(r, RECEIVE_PARTIAL_TIMEOUT, end);
    if (remaining > r->limits.max_pending) return fail(r, RECEIVE_BACKLOG, end);
    if (got > count || (count && !got)) return fail(r, RECEIVE_READ_CONTRACT, end);
    size_t total=r->used+got, start=0, length=0;
    /* 先验证整批，避免坏字节之后才发现已把同批STATE交给会话。 */
    for (size_t i=0; i<total; ++i) {
        unsigned char c=r->buffer[i];
        if(c>126 || (c<32 && c!='\r' && c!='\n')) return fail(r, RECEIVE_INVALID_ASCII, end);
        if(c=='\n') length=0;
        else if(++length > r->limits.max_line) return fail(r, RECEIVE_LINE_OVERFLOW, end);
    }
    if(r->desync) {
        while(start<total && r->buffer[start]!='\n') ++start;
        if(start==total) { r->used=0; return true; }
        ++start; r->desync=false;
    }
    for(size_t i=start; i<total; ++i) if(r->buffer[i]=='\n') {
        r->buffer[i]=0;
        if(i>start && r->buffer[i-1]=='\r') r->buffer[i-1]=0;
        device_session_feed(r->session, (char *)r->buffer+start, end);
        start=i+1;
    }
    bool old=r->partial;
    r->used=total-start;
    memmove(r->buffer, r->buffer+start, r->used);
    r->partial=r->used>0;
    if(r->partial && (!old || start>0)) r->partial_since=now;
    return true;
}
