#ifndef TEST_FAKE_UART_H
#define TEST_FAKE_UART_H
#include "uart.h"
/* 模拟 UART 边界：各端口独立排队；回调由场景设置响应及时间副作用。 */
typedef struct fake_uart {
    unsigned char input[16384];
    size_t pending;
    unsigned reads;
    int read_error;
    void (*before_read)(struct fake_uart *);
    int (*write)(struct fake_uart *, const uint8_t *, size_t);
} fake_uart_t;
void fake_uart_reset(void);
fake_uart_t *fake_uart_port(unsigned index);
void fake_uart_queue(fake_uart_t *, const char *);
#endif
