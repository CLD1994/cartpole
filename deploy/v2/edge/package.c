#include "package.h"
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 有限业务清单，不解析 JSON。每个字段固定顺序且必须以 LF 结束。 */
static int field(FILE *f, const char *key, char *value, size_t cap) {
    char line[256];
    if (!fgets(line, sizeof(line), f)) return -1;
    size_t n = strlen(line), k = strlen(key);
    if (!n || line[n-1] != '\n' || strncmp(line,key,k) || n <= k+1 || n-k > cap) return -1;
    memcpy(value,line+k,n-k-1); value[n-k-1] = 0;
    return 0;
}
static int integer(FILE *f, const char *key, int *out) {
    char text[128], *end;
    if (field(f,key,text,sizeof(text))) return -1;
    errno = 0;
    long v = strtol(text,&end,10);
    if (errno || end == text || *end || v < INT_MIN || v > INT_MAX || text[0]==' ' || text[0]=='\t') return -1;
    *out = (int)v; return 0;
}
static int number(FILE *f, const char *key, double *out) {
    char text[128], *end;
    if (field(f,key,text,sizeof(text))) return -1;
    errno = 0;
    double v = strtod(text,&end);
    if (errno || end == text || *end || !isfinite(v) || text[0]==' ' || text[0]=='\t') return -1;
    *out = v; return 0;
}
static int model_file(const char *manifest, const char *name, const char *expected,
                      const char *hash, char *dest, size_t cap) {
    if (!strcmp(name,"none")) return strcmp(hash,"none") ? -1 : 0;
    if (strcmp(name,expected) || strlen(hash)!=64) return -1;
    for (int i=0;i<64;i++) if (!((hash[i]>='0' && hash[i]<='9') || (hash[i]>='a' && hash[i]<='f'))) return -1;
    const char *slash=strrchr(manifest,'/');
    size_t prefix=slash ? (size_t)(slash-manifest+1) : 0;
    if (prefix+strlen(name)>=cap) return -1;
    memcpy(dest,manifest,prefix); strcpy(dest+prefix,name);
    FILE *f=fopen(dest,"rb");
    if (!f) return -1;
    int valid=fgetc(f)!=EOF;
    fclose(f);
    return valid ? 0 : -1;
}
int edge_package_load(const char *path, edge_package *out, char *err, size_t cap) {
    edge_package p = {0};
    FILE *f = NULL;
    const char *why = "invalid_manifest";
    char header[64], cpu[128], npu[128];
    if (out) memset(out,0,sizeof(*out));
    if (err && cap) err[0] = 0;
    if (!path || !out) goto fail;
    f = fopen(path,"rb");
    if (!f) { why="manifest_unreadable"; goto fail; }
    if (!fgets(header,sizeof(header),f) || strcmp(header,"EDGE_PACKAGE_V2\n")) goto fail;
    if (field(f,"model_cpu=",cpu,sizeof(cpu)) || field(f,"model_npu=",npu,sizeof(npu)) ||
        field(f,"sha256_cpu=",p.sha256_cpu,sizeof(p.sha256_cpu)) ||
        field(f,"sha256_npu=",p.sha256_npu,sizeof(p.sha256_npu))) goto fail;
    if (number(f,"control_hz=",&p.control_hz) ||
        integer(f,"action_scale_mm_s=",&p.action_scale_mm_s) ||
        integer(f,"velocity_limit_mm_s=",&p.velocity_limit_mm_s) ||
        integer(f,"acceleration_mm_s2=",&p.acceleration_mm_s2) ||
        integer(f,"angle_sign=",&p.angle_sign) ||
        number(f,"pause_deg=",&p.pause_deg) || number(f,"resume_deg=",&p.resume_deg) ||
        fgetc(f)!=EOF || ferror(f)) goto fail;
    if ((p.control_hz!=400 && p.control_hz!=500) ||
        (p.action_scale_mm_s!=900 && p.action_scale_mm_s!=1250) ||
        p.velocity_limit_mm_s!=p.action_scale_mm_s || p.acceleration_mm_s2<=0 ||
        p.angle_sign!=-1 || p.pause_deg!=20 || p.resume_deg!=18) goto fail;
    fclose(f); f=NULL;
    why="model_unreadable_or_invalid";
    if (model_file(path,cpu,"policy.fp32",p.sha256_cpu,p.model_cpu_path,sizeof(p.model_cpu_path)) ||
        model_file(path,npu,"policy.om",p.sha256_npu,p.model_npu_path,sizeof(p.model_npu_path)) ||
        (!p.model_cpu_path[0] && !p.model_npu_path[0])) goto fail;
    /* SHA256 是打包来源记录。运行时不检测模型文件事后替换。 */
    *out=p;
    return 0;
fail:
    if (f) fclose(f);
    if (err && cap) snprintf(err,cap,"%s",why);
    return -1;
}
