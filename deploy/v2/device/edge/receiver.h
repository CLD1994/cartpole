#ifndef DEVICE_RECEIVER_H
#define DEVICE_RECEIVER_H
#include "session.h"
#include <stddef.h>
/* 回调必须有界且非阻塞。false表示I/O错误；read可返回短读。 */
typedef bool (*device_pending_fn)(void *, size_t *);
typedef bool (*device_read_fn)(void *, unsigned char *, size_t, size_t *);
typedef double (*device_clock_fn)(void *);
typedef struct {
    size_t max_pending, max_line;
    double max_poll_s, max_read_s, partial_timeout_s;
} device_receive_limits;
typedef enum {
    RECEIVE_OK, RECEIVE_POLL_DEADLINE, RECEIVE_PARTIAL_TIMEOUT,
    RECEIVE_BACKLOG, RECEIVE_IO, RECEIVE_READ_DEADLINE,
    RECEIVE_READ_CONTRACT, RECEIVE_LINE_OVERFLOW, RECEIVE_INVALID_ASCII
} device_receive_error;
typedef struct device_receiver device_receiver;
/* session与context由调用方持有，必须活到destroy。create不执行I/O。 */
device_receiver *device_receiver_create(device_session *, device_receive_limits,
    device_pending_fn, device_read_fn, device_clock_fn, void *);
void device_receiver_destroy(device_receiver *);
/* 每次至多读取一次。失败锁存首次error并abort会话；可继续poll进行停机确认。 */
bool device_receiver_poll(device_receiver *);
device_receive_error device_receiver_error(const device_receiver *);
#endif
