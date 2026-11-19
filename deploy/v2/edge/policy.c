#include "policy.h"
#include "cpu_policy.h"
#include <math.h>
#include <stdio.h>
#if EDGE_ENABLE_NPU
#include "policy_npu.h"
#endif
static policy_backend selected;
static int dead;
int policy_init(policy_backend backend,const char *path) {
    if(selected || !path) return -1;
#if EDGE_ENABLE_NPU
    if(npu_policy_active()) return -1;
#endif
    int rc=-1;
    if(backend==POLICY_CPU) rc=cpu_policy_init(path);
#if EDGE_ENABLE_NPU
    else if(backend==POLICY_NPU) rc=npu_policy_init(path);
#else
    else if(backend==POLICY_NPU) fprintf(stderr,"[policy] NPU backend not built\n");
#endif
    if(rc==0) { selected=backend; dead=0; }
    return rc;
}
int policy_forward(const float obs[4],float *action) {
    if(!action) { if(selected) dead=1; return -1; }
    *action=0;
    if(!selected || dead) return -1;
    if(!obs) { dead=1; return -1; }
    for(int i=0;i<4;i++) if(!isfinite(obs[i])) { dead=1; return -1; }
    int rc=-1;
    if(selected==POLICY_CPU) rc=cpu_policy_forward(obs,action);
#if EDGE_ENABLE_NPU
    else if(selected==POLICY_NPU) rc=npu_policy_forward(obs,action);
#endif
    if(rc || !isfinite(*action)) { dead=1; *action=0; return -1; }
    return 0;
}
void policy_deinit(void) {
    cpu_policy_deinit();
#if EDGE_ENABLE_NPU
    /* 初始化失败后仍可能持有资源，此处继续尝试清理。 */
    if(npu_policy_active()) npu_policy_deinit();
#endif
    selected=0; dead=0;
}
