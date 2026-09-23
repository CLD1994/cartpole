#define _POSIX_C_SOURCE 200809L
#include "cartpole_npu.h"
#include "acl/svp_acl.h"

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

enum exit_code { EXIT_OK = 0, EXIT_INPUT = 2, EXIT_EXECUTION = 3 };
enum line_result { LINE_END, LINE_OK, LINE_INVALID, LINE_IO };

/* 有界逐行读取，拒绝嵌入 NUL；不让长行被拆成多个有效样本。 */
static enum line_result read_line(FILE *file, char line[1024])
{
    size_t used = 0;
    int ch;
    while ((ch = fgetc(file)) != EOF) {
        if (ch == '\n') { line[used] = 0; return LINE_OK; }
        if (ch == 0 || used == 1023) return LINE_INVALID;
        line[used++] = (char)ch;
    }
    if (ferror(file)) return LINE_IO;
    line[used] = 0;
    return used ? LINE_OK : LINE_END;
}

static const char *whitespace(const char *p)
{ while (isspace((unsigned char)*p)) ++p; return p; }

/* 明确十进制语法，避免 strtof 接受十六进制、NaN 或无穷大。 */
static int parse_observation(const char *line, float obs[4])
{
    const char *p = line, *start;
    char *end;
    size_t i, digits;
    for (i = 0; i < 4; ++i) {
        p = whitespace(p); start = p;
        if (*p == '+' || *p == '-') ++p;
        digits = 0;
        while (isdigit((unsigned char)*p)) { ++p; ++digits; }
        if (*p == '.') {
            ++p; while (isdigit((unsigned char)*p)) { ++p; ++digits; }
        }
        if (!digits) return 0;
        if (*p == 'e' || *p == 'E') {
            ++p; if (*p == '+' || *p == '-') ++p;
            digits = 0; while (isdigit((unsigned char)*p)) { ++p; ++digits; }
            if (!digits) return 0;
        }
        errno = 0; obs[i] = strtof(start, &end);
        if (end != p || !isfinite(obs[i]) || (errno == ERANGE && obs[i] == 0)) return 0;
        p = whitespace(p);
        if (i < 3) { if (*p != ',') return 0; ++p; }
        else if (*p) return 0;
    }
    return 1;
}

static void report_error(const cartpole_error *error)
{
    fprintf(stderr, "推理库错误：stage=%s code=%d sdk=%" PRId32 "\n",
            error->stage ? error->stage : "unknown", (int)error->code, error->sdk_code);
    if (error->cleanup_stage)
        fprintf(stderr, "清理错误：stage=%s sdk=%" PRId32 "\n", error->cleanup_stage, error->cleanup_code);
}

static int replay(FILE *input, FILE *output, cartpole_npu *model, uint64_t *samples)
{
    char line[1024];
    enum line_result state;
    cartpole_error error;
    struct timespec start, end;
    float obs[4], action;
    int64_t elapsed;
    while ((state = read_line(input, line)) != LINE_END) {
        if (state == LINE_IO) { fprintf(stderr, "读取观测失败\n"); return EXIT_EXECUTION; }
        if (state == LINE_INVALID || !parse_observation(line, obs)) {
            fprintf(stderr, "第 %" PRIu64 " 行不是四个有限十进制 float，或超过 1023 字节\n", *samples + 1);
            return EXIT_INPUT;
        }
        if (clock_gettime(CLOCK_MONOTONIC, &start)) { perror("clock_gettime"); return EXIT_EXECUTION; }
        if (cartpole_npu_infer(model, obs, &action, &error)) {
            fprintf(stderr, "推理失败：line=%" PRIu64 "\n", *samples + 1);
            report_error(&error); return EXIT_EXECUTION;
        }
        if (clock_gettime(CLOCK_MONOTONIC, &end)) { perror("clock_gettime"); return EXIT_EXECUTION; }
        elapsed = ((int64_t)end.tv_sec - (int64_t)start.tv_sec)*INT64_C(1000000000) + end.tv_nsec - start.tv_nsec;
        if (elapsed < 0) { fprintf(stderr, "单调时钟倒退\n"); return EXIT_EXECUTION; }
        if (fprintf(output, "%" PRIu64 ",%.9g,%" PRId64 "\n", *samples, (double)action, elapsed) < 0) {
            perror("写入动作"); return EXIT_EXECUTION;
        }
        ++*samples;
    }
    if (!*samples) { fprintf(stderr, "观测文件为空\n"); return EXIT_INPUT; }
    return EXIT_OK;
}

int main(int argc, char **argv)
{
    FILE *input = NULL, *output = NULL;
    cartpole_npu *model = NULL;
    cartpole_error error;
    svp_acl_error rc;
    uint64_t samples = 0;
    int fd, initialized = 0, device_set = 0, result = EXIT_EXECUTION;
    if (argc == 2 && !strcmp(argv[1], "--help")) {
        puts("用法：cartpole-replay MODEL.om OBS.csv ACTIONS.csv\n观测 CSV 无表头，每行：小车位置,杆角度,小车速度,杆角速度。输出文件必须不存在。");
        return EXIT_OK;
    }
    if (argc != 4) { fprintf(stderr, "用法：cartpole-replay MODEL.om OBS.csv ACTIONS.csv\n"); return EXIT_INPUT; }
    input = fopen(argv[2], "rb");
    if (!input) { perror(argv[2]); return EXIT_INPUT; }
    fd = open(argv[3], O_WRONLY | O_CREAT | O_EXCL, 0666);
    if (fd < 0) { perror(argv[3]); fclose(input); return EXIT_INPUT; }
    output = fdopen(fd, "w");
    if (!output) { perror("fdopen"); close(fd); fclose(input); return EXIT_EXECUTION; }
    if (fputs("sample,action,inference_ns\n", output) < 0) { perror("写入表头"); goto cleanup; }
    rc = svp_acl_init(NULL);
    if (rc) { fprintf(stderr, "init sdk=%" PRId32 "\n", rc); goto cleanup; }
    initialized = 1;
    rc = svp_acl_rt_set_device(0);
    if (rc) { fprintf(stderr, "set_device sdk=%" PRId32 "\n", rc); goto cleanup; }
    device_set = 1;
    if (cartpole_npu_open(argv[1], &model, &error)) { report_error(&error); goto cleanup; }
    result = replay(input, output, model, &samples);
cleanup:
    if (cartpole_npu_close(&model, &error)) { report_error(&error); result = EXIT_EXECUTION; }
    /* 未释放的句柄仍依赖运行时：不 reset/finalize，由本进程退出结束生命周期。 */
    if (!model) {
        if (device_set) {
            rc = svp_acl_rt_reset_device(0);
            if (rc) { fprintf(stderr, "reset_device sdk=%" PRId32 "\n", rc); result = EXIT_EXECUTION; }
            else device_set = 0;
        }
        if (initialized && !device_set) {
            rc = svp_acl_finalize();
            if (rc) { fprintf(stderr, "finalize sdk=%" PRId32 "\n", rc); result = EXIT_EXECUTION; }
        }
    }
    if (fclose(input)) { perror("关闭观测文件"); result = EXIT_EXECUTION; }
    if (fclose(output)) { perror("关闭动作文件"); result = EXIT_EXECUTION; }
    if (result == EXIT_OK) fprintf(stderr, "回放完成：samples=%" PRIu64 "\n", samples);
    else fprintf(stderr, "回放未完成：已写入的动作文件仅供诊断\n");
    return result;
}
