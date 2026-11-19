#include "fake_uart.h"
#include <assert.h>
#include <string.h>

static fake_uart_t ports[8];
static unsigned count;

void fake_uart_reset(void)
{
    memset(ports, 0, sizeof ports);
    count = 0;
}
fake_uart_t *fake_uart_port(unsigned index)
{
    assert(index < 8);
    return &ports[index];
}
void fake_uart_queue(fake_uart_t *p, const char *s)
{
    size_t n = strlen(s);
    assert(p->pending + n <= sizeof p->input);
    memcpy(p->input + p->pending, s, n);
    p->pending += n;
}
int uart_open(uart_t *u, const char *path, int baud)
{
    assert(path && baud > 0);
    assert(count < 8);
    u->fd = (int)++count;
    return 0;
}
void uart_close(uart_t *u) { u->fd = -1; }
static fake_uart_t *port(uart_t *u)
{
    assert(u->fd > 0 && (unsigned)u->fd <= count);
    return &ports[u->fd - 1];
}
int uart_read(uart_t *u, uint8_t *data, size_t cap)
{
    fake_uart_t *p = port(u);
    p->reads++;
    if (p->before_read)
        p->before_read(p);
    if (p->read_error)
        return -1;
    size_t n = p->pending < cap ? p->pending : cap;
    memcpy(data, p->input, n);
    memmove(p->input, p->input + n, p->pending - n);
    p->pending -= n;
    return (int)n;
}
int uart_try_write_all(uart_t *u, const uint8_t *data, size_t n)
{
    fake_uart_t *p = port(u);
    return p->write ? p->write(p, data, n) : 0;
}
int uart_write_all(uart_t *u, const uint8_t *data, size_t n)
{
    return uart_try_write_all(u, data, n);
}
