#include "proto.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static int callbacks;
static char last[160];
static void record(const char *s, size_t n, void *u)
{
    (void)u;
    callbacks++;
    memcpy(last, s, n);
    last[n] = 0;
}
int main(void)
{
    proto_msg_t m;
    assert(proto_parse_line("STATE 3 -12 30 4 -5", &m) == PROTO_STATE);
    assert(m.x_um == -12 && m.theta_urad == 30 && m.vx_mm_s == 4 && m.vth_mrad_s == -5);
    const char *bad[] = {" STATE 3 0 0 0 0",      "STATE 1 1 4 2 0",          "STATE 2 1 2 0 3",
                         "STATE 3 0 3141593 0 0", "STATE 3 2147483648 0 0 0", "STATE 3 +1 0 0 0",
                         "STATE 3 01 0 0 0",      "STATE 3 -0 0 0 0",         "STATE 3 0 0 0 0 ",
                         "STATE 0 0 0 0",         "STATE 4 0 0 0 0",          "STATE 3 1-2 0 0 0"};
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++)
        assert(proto_parse_line(bad[i], &m) == PROTO_UNKNOWN);
    assert(proto_parse_line("STATE 0 0 0 0 0", &m) == PROTO_STATE);
    assert(proto_parse_line("STATE 1 2147483647 0 -2147483648 0", &m) == PROTO_STATE);
    proto_rx_t rx;
    proto_rx_init(&rx);
    char longline[200];
    memset(longline, ' ', sizeof longline);
    memcpy(longline, "STATE 3 0 0 0 0", 15);
    longline[199] = '\n';
    proto_rx_feed(&rx, (uint8_t *)longline, sizeof longline, record, 0);
    assert(callbacks == 1 && proto_parse_line(last, &m) == PROTO_UNKNOWN);
    const uint8_t good[] = "STATE 3 0 0 0 0\n";
    proto_rx_feed(&rx, good, sizeof(good) - 1, record, 0);
    assert(callbacks == 2 && proto_parse_line(last, &m) == PROTO_STATE);
    const uint8_t embedded[] = {'S', 'T', 'A', 'T', 'E', ' ', '3', ' ', '0',
                                ' ', '0', ' ', '0', ' ', '0', 0,   'X', '\n'};
    proto_rx_feed(&rx, embedded, sizeof embedded, record, 0);
    assert(proto_parse_line(last, &m) == PROTO_UNKNOWN);
    char buf[64];
    assert(proto_parse_line("MODE FAULT POWER", &m) == PROTO_MODE);
    assert(m.mode == FW_FAULT && strcmp(proto_reason_name(m.reason), "POWER") == 0);
    proto_command_t command;
    assert(proto_parse_command("CONFIRM LEFT\n", &command));
    assert(!proto_parse_command("CONFIRM RIGHT\n", &command));
    assert(proto_format_simple(buf, "QUERY\nENABLE") == 0);
    assert(proto_format_config(buf, "A\nENABLE", 1) == 0);
    puts("edge proto passed");
    return 0;
}
