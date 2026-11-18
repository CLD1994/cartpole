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
int edge_package_load(const char *path, edge_package *out, char *err, size_t cap) {
    edge_package p = {0};
    FILE *f = NULL, *model = NULL;
    const char *why = "invalid_manifest";
    char header[64], name[128];
    if (out) memset(out,0,sizeof(*out));
    if (err && cap) err[0] = 0;
    if (!path || !out) goto fail;
    f = fopen(path,"rb");
    if (!f) { why="manifest_unreadable"; goto fail; }
    if (!fgets(header,sizeof(header),f) || strcmp(header,"EDGE_PACKAGE_V1\n")) goto fail;
    if (field(f,"model=",name,sizeof(name)) || strcmp(name,"policy.om") ||
        field(f,"sha256=",p.model_sha256,sizeof(p.model_sha256)) || strlen(p.model_sha256)!=64) goto fail;
    for (int i=0;i<64;i++) if (!((p.model_sha256[i]>='0' && p.model_sha256[i]<='9') || (p.model_sha256[i]>='a' && p.model_sha256[i]<='f'))) goto fail;
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
    const char *slash = strrchr(path,'/');
    size_t prefix = slash ? (size_t)(slash-path+1) : 0;
    if (prefix + strlen(name) >= sizeof(p.model_path)) { why="model_path_too_long"; goto fail; }
    memcpy(p.model_path,path,prefix); strcpy(p.model_path+prefix,name);
    model=fopen(p.model_path,"rb");
    if (!model || fgetc(model)==EOF) { why="model_unreadable_or_empty"; goto fail; }
    fclose(model);
    /* SHA256 是打包来源记录。运行时不检测 OM 文件事后替换。 */
    *out=p;
    return 0;
fail:
    if (f) fclose(f);
    if (model) fclose(model);
    if (err && cap) snprintf(err,cap,"%s",why);
    return -1;
}
