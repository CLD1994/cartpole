#include "package.h"
#include "third_party/yyjson/yyjson.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Package metadata is bounded and parsed by pinned upstream yyjson. */
static const char *keys[]={"schema_version","model_cpu","model_npu","sha256_cpu","sha256_npu",
    "control_hz","action_scale_mm_s","velocity_limit_mm_s","acceleration_mm_s2","angle_sign","pause_deg","resume_deg"};
static int numeric(yyjson_val *root,const char *key,double *out) {
    yyjson_val *v=yyjson_obj_get(root,key);
    if(!yyjson_is_num(v) || !isfinite(yyjson_get_num(v))) return -1;
    *out=yyjson_get_num(v); return 0;
}
static int integer(yyjson_val *root,const char *key,int *out) {
    double n;
    if(numeric(root,key,&n) || n<INT_MIN || n>INT_MAX || trunc(n)!=n) return -1;
    *out=(int)n; return 0;
}
static const char *optional_string(yyjson_val *root,const char *key) {
    yyjson_val *v=yyjson_obj_get(root,key);
    if(yyjson_is_null(v)) return "none";
    if(!yyjson_is_str(v) || strlen(yyjson_get_str(v))!=yyjson_get_len(v)) return NULL;
    return yyjson_get_str(v);
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
    edge_package p={0}; FILE *f=NULL; yyjson_doc *doc=NULL; yyjson_val *root=NULL;
    char data[8193]; const char *why="invalid_manifest";
    if(out) memset(out,0,sizeof(*out));
    if(err && cap) err[0]=0;
    if(!path || !out) goto fail;
    f=fopen(path,"rb");
    if(!f) { why="manifest_unreadable"; goto fail; }
    size_t n=fread(data,1,sizeof(data)-1,f);
    if(ferror(f) || fgetc(f)!=EOF || memchr(data,0,n)) goto fail;
    fclose(f);f=NULL;data[n]=0;
    doc=yyjson_read(data,n,0);
    root=yyjson_doc_get_root(doc);
    if(!yyjson_is_obj(root)) goto fail;
    unsigned seen=0;
    yyjson_obj_iter iter=yyjson_obj_iter_with(root);
    yyjson_val *key;
    while((key=yyjson_obj_iter_next(&iter))) {
        size_t k;
        for(k=0;k<sizeof(keys)/sizeof(keys[0]);k++)
            if(yyjson_get_len(key)==strlen(keys[k]) && !strcmp(yyjson_get_str(key),keys[k])) break;
        if(k==sizeof(keys)/sizeof(keys[0]) || (seen&(1u<<k))) goto fail;
        seen|=1u<<k;
    }
    if(seen!=((1u<<12)-1)) goto fail;
    int version;
    if(integer(root,"schema_version",&version) || version!=2 ||
       numeric(root,"control_hz",&p.control_hz) ||
       integer(root,"action_scale_mm_s",&p.action_scale_mm_s) ||
       integer(root,"velocity_limit_mm_s",&p.velocity_limit_mm_s) ||
       integer(root,"acceleration_mm_s2",&p.acceleration_mm_s2) ||
       integer(root,"angle_sign",&p.angle_sign) ||
       numeric(root,"pause_deg",&p.pause_deg) || numeric(root,"resume_deg",&p.resume_deg)) goto fail;
    if((p.control_hz!=400 && p.control_hz!=500) ||
       (p.action_scale_mm_s!=900 && p.action_scale_mm_s!=1250) ||
       p.velocity_limit_mm_s!=p.action_scale_mm_s || p.acceleration_mm_s2<=0 ||
       p.angle_sign!=-1 || p.pause_deg!=20 || p.resume_deg!=18) goto fail;
    const char *cpu=optional_string(root,"model_cpu"), *npu=optional_string(root,"model_npu");
    const char *hc=optional_string(root,"sha256_cpu"), *hn=optional_string(root,"sha256_npu");
    if(!cpu || !npu || !hc || !hn || strlen(hc)>=sizeof(p.sha256_cpu) || strlen(hn)>=sizeof(p.sha256_npu)) goto fail;
    /* Literal "none" is not a filename: absent entries must be JSON null. */
    for(int i=1;i<=4;i++) {
        yyjson_val *v=yyjson_obj_get(root,keys[i]);
        if(yyjson_is_str(v) && !strcmp(yyjson_get_str(v),"none")) goto fail;
    }
    strcpy(p.sha256_cpu,hc);strcpy(p.sha256_npu,hn);
    why="model_unreadable_or_invalid";
    if(model_file(path,cpu,"policy.fp32",hc,p.model_cpu_path,sizeof(p.model_cpu_path)) ||
       model_file(path,npu,"policy.om",hn,p.model_npu_path,sizeof(p.model_npu_path)) ||
       (!p.model_cpu_path[0] && !p.model_npu_path[0])) goto fail;
    /* Hashes record preparation provenance, not runtime tamper detection. */
    yyjson_doc_free(doc);*out=p;return 0;
fail:
    if(f) fclose(f);
    yyjson_doc_free(doc);
    if(err && cap) snprintf(err,cap,"%s",why);
    return -1;
}
