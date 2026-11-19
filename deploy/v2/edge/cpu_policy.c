/* CPFP32V1：小端维度及 FP32 行优先权重。三层仿射变换，前两层使用
 * ELU(alpha=1)。加载时一次分配；推理期间不分配内存。 */
#include "cpu_policy.h"
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static float *parameters;
static uint32_t dims[4];
static float work[2][256];
static int read_u32(FILE *f, uint32_t *out) {
    unsigned char b[4];
    if (fread(b,1,4,f)!=4) return -1;
    *out=(uint32_t)b[0]|(uint32_t)b[1]<<8|(uint32_t)b[2]<<16|(uint32_t)b[3]<<24;
    return 0;
}
void cpu_policy_deinit(void) { free(parameters); parameters=NULL; memset(dims,0,sizeof dims); }
int cpu_policy_init(const char *path) {
    if (!path || parameters || sizeof(float)!=4 || FLT_RADIX!=2 || FLT_MANT_DIG!=24) return -1;
    FILE *f=fopen(path,"rb"); if (!f) return -1;
    unsigned char magic[8]; uint32_t d[4]; float *p=NULL; int rc=-1;
    if(fread(magic,1,8,f)!=8 || memcmp(magic,"CPFP32V1",8)) goto done;
    for(int i=0;i<4;i++) if(read_u32(f,&d[i])) goto done;
    if(d[0]!=4 || d[3]!=1 || d[1]<1 || d[1]>256 || d[2]<1 || d[2]>256) goto done;
    size_t n=0; for(int i=0;i<3;i++) n+=(size_t)(d[i]+1)*d[i+1];
    p=malloc(n*sizeof(float)); if(!p) goto done;
    for(size_t i=0;i<n;i++) { uint32_t bits; if(read_u32(f,&bits)) goto done; memcpy(&p[i],&bits,4); if(!isfinite(p[i])) goto done; }
    if(fgetc(f)!=EOF || ferror(f)) goto done;
    memcpy(dims,d,sizeof dims); parameters=p; p=NULL; rc=0;
 done: free(p); fclose(f); return rc;
}
int cpu_policy_forward(const float obs[4], float *action) {
    if(!action) return -1;
    *action=0;
    if(!parameters || !obs) return -1;
    for(int i=0;i<4;i++) if(!isfinite(obs[i])) return -1;
    const float *in=obs, *p=parameters;
    float result=0;
    for(int layer=0;layer<3;layer++) {
        unsigned ni=dims[layer], no=dims[layer+1];
        const float *b=p+ni*no; float *out=layer==2?&result:work[layer];
        for(unsigned j=0;j<no;j++) {
            float v=b[j]; for(unsigned k=0;k<ni;k++) v+=p[j*ni+k]*in[k];
            if(!isfinite(v)) return -1;
            if(layer<2 && v<0) v=expm1f(v);
            out[j]=v;
        }
        p=b+no; in=out;
    }
    *action=result; return 0;
}
