# 回零位置偏差判据实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 回零完成信号从 TMC2209 StallGuard 堵转检测切换为 AS5600 位置偏差判据（达速锁存 + 决策表），StallGuard/DIAG/SGTHRS 全链路退役，协议与清单同步修订。

**Architecture:** 判定逻辑按既有分层落在纯核心 `cartfsm_core`（新增 `cm_homing_event` 决策表函数与 `cm_speed_reached` 达速判据，宿主可测），`cartfsm.c` 外壳在 1 kHz 监督任务的 HOMING 分支接入；驱动芯片不再承担任何判定职责。协议 `deploy/v2/protocol.md` 是线协议权威，随固件落地同步修订。

**Tech Stack:** ESP-IDF v6.1（ESP32-S3，三构建门禁）、C11 宿主测试（WSL gcc）、端侧 C（WSL make）。

**Spec:** `deploy/v2/homing-spec.md`（行为契约权威，commit 244b987）。执行本计划前先通读规格。

## Global Constraints

- 行为契约以 `deploy/v2/homing-spec.md` 为准；线协议文本以 `deploy/v2/protocol.md` 为准。发现计划与规格冲突时停止并上报，不静默选择。
- 两个 git 仓库：固件 `tmp/PD-STEP`（分支 `cartpole`）；文档与端侧在 cartpole 仓库（分支 `master`）。提交信息用中文「域：描述」风格。
- 宿主测试命令（在 `tmp/PD-STEP` 下，已实证可用；wsl 打头的 NAT localhost 警告无害）：
  ```bash
  wsl gcc -std=c11 -Wall -Wextra -Werror -Ihost_tests/stub \
    -Icomponents/cartfsm/include host_tests/cartfsm_core_test.c \
    components/cartfsm/cartfsm_core.c components/cartfsm/params.c \
    -o host_tests/.build_core_test && wsl ./host_tests/.build_core_test \
    && wsl rm -f host_tests/.build_core_test
  ```
  （基线 98/98 断言通过，退出码 0 = 全过）
- 协议解析宿主测试（Task 2 用，基线 49/49）：
  ```bash
  wsl gcc -std=c11 -Wall -Wextra -Werror -Ihost_tests/stub \
    -Icomponents/cartproto/include host_tests/cartproto_parse_test.c \
    components/cartproto/cartproto_parse.c -o host_tests/.build_proto_test \
    && wsl ./host_tests/.build_proto_test && wsl rm -f host_tests/.build_proto_test
  ```
- 固件三构建门禁（PowerShell，不是 Git Bash；Git Bash 下 idf.py 需先清 MSYSTEM）：
  ```powershell
  . C:\esp\v6.1\esp-idf\export.ps1
  cd C:\Users\84580\workspace\cld\cartpole\tmp\PD-STEP
  idf.py -B build build
  idf.py -B build-selftest -DSDKCONFIG=build-selftest/sdkconfig -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;tools/selftest.defaults" build
  idf.py -B build-flowtest -DSDKCONFIG=build-flowtest/sdkconfig -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;tools/flowtest.defaults" build
  ```
- chopper 保持 SpreadCycle，不回 StealthChop（规格 §5）。
- 不新增 HOME 专用阈值参数：`STEP_LOSS_UM` 单一权威双角色（规格 §1）。
- 删除 Kconfig 选项时必须同步清理三份存量 sdkconfig（`sdkconfig` / `build-selftest/sdkconfig` / `build-flowtest/sdkconfig` 各自独立，598c18d 教训）。
- 测试全绿不能替代对规格覆盖的检查；测试只断言本任务声明的契约。

## Review Focus

规格隐含但任务测试未必覆盖、最可能咬人的五类输入/失败模式（每条已钉到 owning task 的测试步骤）：

1. **AS5600 域反向**（编码器计数方向与指令相反）：实测速度为正、达速锁存永不置位，偏差越限必须判 FAULT/STEP_LOSS 而不是假「到位」——Task 1 `test_speed_reached`（正方向不算）+ `test_homing_event`（latch=0 → STEP_LOSS）钉住。
2. **起点压挡块**（回零刚起步电机即堵死）：同样不得假「到位」——Task 2 flowtest `test_home_loss_before_speed` 钉住，Task 6 上板负路径复核。
3. **同拍事件冲突**：编码器失败 vs 偏差越限同拍（E 无效不得参与判定）、到位 vs 超时同拍——Task 1 `test_homing_event` 的优先序行钉住。
4. **测试偏置污染速度判据**：flowtest 注入的 E 偏置若进入 v_meas 差分，会让负路径用例在「锁存被偏置阶跃误置位」下假绿——Task 2 外壳实现中 v_meas 必须用原始 E（`e_raw`），偏置只加进偏差判定的 `e_um`；`test_home_loss_before_speed` 通过即证明未污染。
5. **回零后静止抵挡块误报**：E/C 同帧重定基后静止时 D 恒为 0，不得 FAULT——Task 2 flowtest `test_home_done_to_ready` 的原点断言 + Task 6 静置 1 min 钉住。

---

### Task 1: 纯核心新增到位判定函数（宿主 TDD，纯增量）

**Files:**
- Modify: `tmp/PD-STEP/components/cartfsm/include/cartfsm_core.h`
- Modify: `tmp/PD-STEP/components/cartfsm/cartfsm_core.c`
- Test: `tmp/PD-STEP/host_tests/cartfsm_core_test.c`

**Interfaces:**
- Consumes: 既有 `cm_step_loss`、`cm_event_t`/`cm_mode_t` 枚举。
- Produces（Task 2 外壳将调用，签名不得改动）:
  ```c
  typedef struct {
      bool a5600_ok;      /* AS5600 读取成功（E 有效性；false 最先判） */
      bool m6835_ok;      /* MT6835 读取成功 */
      bool speed_latch;   /* 达速锁存已置位（实测曾达 −0.8×HOME_VEL） */
      long e_um;          /* 实测位置 µm（含测试偏置） */
      long c_um;          /* 指令位置 µm */
      long step_loss_um;  /* 偏差阈值 T */
      bool timed_out;     /* 已超过 HOME_TIMEOUT */
  } cm_homing_in_t;

  cm_event_t cm_homing_event(const cm_homing_in_t *in);
  bool cm_speed_reached(long v_meas_mm_s, long home_vel_mm_s);
  ```
  以及新枚举值 `CE_HOME_DONE`（HOMING 到位事件，`cm_step` 中 HOMING 态消费 → READY/NONE；本任务不删除 `CE_STALL`，删除在 Task 2）。

- [ ] **Step 1: 写失败测试**——`host_tests/cartfsm_core_test.c` 末尾 `main` 前新增两个用例函数，并注册进 `main`：

```c
/* ---- HOMING 到位决策表（homing-spec §2 优先序） ---- */
static void test_homing_event(void)
{
    cm_homing_in_t in = {
        .a5600_ok = true, .m6835_ok = true, .speed_latch = true,
        .e_um = 0, .c_um = 0, .step_loss_um = 3000, .timed_out = false,
    };
    /* 序 1：AS5600 失败最优先——E 无效不得参与任何判定 */
    in.a5600_ok = false; in.e_um = 999999;
    CHECK(cm_homing_event(&in) == CE_AS5600_FAIL, "AS5600 失败最优先");
    in.a5600_ok = true;
    /* 序 2：MT6835 失败次之 */
    in.m6835_ok = false;
    CHECK(cm_homing_event(&in) == CE_MT6835_FAIL, "MT6835 失败次之");
    in.m6835_ok = true;
    /* 序 3：未达速偏差 → STEP_LOSS（起点压挡块/卡死/域反向） */
    in.speed_latch = false; in.e_um = 4000;
    CHECK(cm_homing_event(&in) == CE_STEP_LOSS, "未达速偏差→STEP_LOSS");
    /* 序 4：达速偏差 → 到位 */
    in.speed_latch = true;
    CHECK(cm_homing_event(&in) == CE_HOME_DONE, "达速偏差→到位");
    /* 序 4 优先于序 5：同拍超时也判到位 */
    in.timed_out = true;
    CHECK(cm_homing_event(&in) == CE_HOME_DONE, "到位优先于超时");
    /* 序 5：无偏差时超时（不论锁存） */
    in.e_um = 0; in.speed_latch = false;
    CHECK(cm_homing_event(&in) == CE_HOME_TIMEOUT, "未达速无偏差→超时");
    in.speed_latch = true;
    CHECK(cm_homing_event(&in) == CE_HOME_TIMEOUT, "达速无偏差→超时");
    /* 无事件与 NULL 安全 */
    in.timed_out = false;
    CHECK(cm_homing_event(&in) == 0, "常态无事件");
    CHECK(cm_homing_event(NULL) == 0, "NULL 无事件");
}

/* ---- 达速判据：含符号、0.8×HOME_VEL 门限 ---- */
static void test_speed_reached(void)
{
    CHECK(cm_speed_reached(-40, 50), "−40 达 −0.8×50 门限");
    CHECK(!cm_speed_reached(-39, 50), "−39 未达门限");
    CHECK(cm_speed_reached(-50, 50), "额定速度达标");
    CHECK(!cm_speed_reached(40, 50), "正方向不算（域反向保护）");
    CHECK(!cm_speed_reached(0, 50), "静止不算");
    CHECK(!cm_speed_reached(-40, 0), "HOME_VEL=0 非法输入不算");
    CHECK(!cm_speed_reached(-40, -50), "HOME_VEL 负值不算");
    CHECK(cm_speed_reached(-4, 5), "最小 HOME_VEL=5 的门限 −4");
}
```

`main` 中 `test_step_loss();` 之后追加两行：
```c
    test_homing_event();
    test_speed_reached();
```

同时在 `test_transitions` 的「HOMING + 堵转 → READY」块后追加（新事件的转换行；`CE_STALL` 的旧行本任务不动）：
```c
    /* HOMING + 到位（新判据事件）→ READY/NONE */
    m = CM_HOMING; r = CR_NONE;
    CHECK(step(CE_HOME_DONE) && m == CM_READY && r == CR_NONE,
          "HOMING+HOME_DONE→READY");
```
以及 READY / FAULT / DISABLED 的忽略断言（分别加在各态的忽略组里）：
```c
    m = CM_READY;
    CHECK(!step(CE_HOME_DONE), "READY+HOME_DONE 忽略");
    m = CM_FAULT;
    CHECK(!step(CE_HOME_DONE), "FAULT+HOME_DONE 忽略");
```
（DISABLED 组同样补 `CHECK(!step(CE_HOME_DONE), "DISABLED+HOME_DONE 忽略");`）

- [ ] **Step 2: 跑测试确认失败**

Run: Global Constraints 中的宿主测试命令。
Expected: 编译失败——`CE_HOME_DONE` 未定义、`cm_homing_in_t`/`cm_homing_event`/`cm_speed_reached` 未声明（这就是本任务的红）。

- [ ] **Step 3: 实现纯核心**

`cartfsm_core.h`：`cm_event_t` 枚举中 `CE_STALL` 行后追加
```c
    CE_HOME_DONE,          /* HOMING 到位判定（达速后 |E−C| 越限） */
```
（本任务保留 `CE_STALL` 定义，Task 2 删除。）文件末尾 `cm_step_loss` 声明后追加 Interfaces 块中的 `cm_homing_in_t` 定义与两个函数声明，配注释：
```c
/* ---- HOMING 到位判定（位置偏差判据，homing-spec §2） ---- */
```

`cartfsm_core.c` 末尾追加：
```c
bool cm_speed_reached(long v_meas_mm_s, long home_vel_mm_s)
{
    if (home_vel_mm_s <= 0) {
        return false;
    }
    return v_meas_mm_s * 5 <= -home_vel_mm_s * 4;
}

cm_event_t cm_homing_event(const cm_homing_in_t *in)
{
    if (in == NULL) {
        return (cm_event_t)0;
    }
    if (!in->a5600_ok) {
        return CE_AS5600_FAIL;      /* E 无效时不得用于任何判定 */
    }
    if (!in->m6835_ok) {
        return CE_MT6835_FAIL;
    }
    if (cm_step_loss(in->e_um, in->c_um, in->step_loss_um)) {
        return in->speed_latch ? CE_HOME_DONE : CE_STEP_LOSS;
    }
    if (in->timed_out) {
        return CE_HOME_TIMEOUT;
    }
    return (cm_event_t)0;
}
```

并在 `cm_step` 的 `CM_HOMING` switch 中 `CE_STALL` case 后追加：
```c
        case CE_HOME_DONE:          /* 到位 = 回零完成信号（位置偏差判据） */
            *mode = CM_READY;
            *reason = CR_NONE;
            return true;
```

- [ ] **Step 4: 跑测试确认通过**

Run: 同 Step 2。
Expected: PASS，退出码 0，断言数较 98 增加（新增用例全部计入）。

- [ ] **Step 5: 三构建门禁**（纯增量也应全绿）

Run: Global Constraints 的三构建命令。
Expected: 三个 build 全部成功（`CE_STALL` 等旧符号未动，现有代码不受影响）。

- [ ] **Step 6: Commit**（固件仓库）

```bash
cd tmp/PD-STEP && git add components/cartfsm host_tests/cartfsm_core_test.c
git commit -m "cartfsm：纯核心新增位置偏差到位判定（cm_homing_event 决策表 + cm_speed_reached 达速判据，宿主测试先行）"
```

---

### Task 2: 判定切换原子提交（外壳重写 + 枚举清理 + flowtest 语义重写）

固件内 `CE_STALL`/`CR_STALL`/`sgthrs` 的引用分散在 cartfsm.c、selftest、flowtest 三处，必须一次提交内全部切换才能保持每个构建可编译。

**Files:**
- Modify: `tmp/PD-STEP/components/cartfsm/cartfsm.c`（判定重写与 StallGuard 引用清除）
- Modify: `tmp/PD-STEP/components/cartfsm/include/cartfsm.h`（接口：删 `cartfsm_report_stall`，增 `cartfsm_reason` 与两个测试钩子）
- Modify: `tmp/PD-STEP/components/cartfsm/cartfsm_core.c` + `include/cartfsm_core.h`（删 `CE_STALL`/`CR_STALL`；HOMING 增 `CE_STEP_LOSS` 转换）
- Modify: `tmp/PD-STEP/components/cartfsm/include/params.h` + `params.c`（删 `sgthrs`）
- Modify: `tmp/PD-STEP/components/cartfsm/cartfsm_self_test.c`（向量字样）
- Modify: `tmp/PD-STEP/components/cartfsm/cartfsm_flow_test.c`（用例重写，见 Step 4）
- Test: `tmp/PD-STEP/host_tests/cartfsm_core_test.c`（向量同步）
- Test: `tmp/PD-STEP/host_tests/cartproto_parse_test.c`（一处 SGTHRS 字样）

**Interfaces:**
- Consumes: Task 1 的 `cm_homing_event`/`cm_speed_reached`/`CE_HOME_DONE`。
- Produces（Task 3/6 与上板依赖）:
  ```c
  cm_reason_t cartfsm_reason(void);              /* 当前原因（枚举原子读） */
  /* 仅 CONFIG_CARTFSM_FLOW_TEST 编译： */
  void cartfsm_test_follow_cmd(bool enable);     /* 判定用 E 以指令位置替代（理想编码器替身） */
  void cartfsm_test_speed_latch(int state);      /* 达速锁存覆写：-1 自然 / 0 强制清 / 1 强制置 */
  ```

- [ ] **Step 1: 宿主测试向量先行（红）**

`host_tests/cartfsm_core_test.c`：
1. `test_transitions` 中删除三处 `CE_STALL` 断言（「DISABLED+STALL 忽略」「HOMING+STALL→READY」「READY+STALL→FAULT」「FAULT+STALL 忽略」——Task 1 已加的 `CE_HOME_DONE` 对应行保留）。
2. 把「HOMING 忽略 HOME/STEP_LOSS」块中的
   ```c
   CHECK(!step(CE_STEP_LOSS), "HOMING+STEP_LOSS 忽略");
   ```
   替换为转换断言：
   ```c
   m = CM_HOMING; r = CR_NONE;
   CHECK(step(CE_STEP_LOSS) && m == CM_FAULT && r == CR_STEP_LOSS,
         "HOMING+STEP_LOSS→FAULT（未达速偏差）");
   ```
3. `FAULT+RESET` 用例的 `r = CR_STALL;` 改为 `r = CR_STEP_LOSS;`。
4. `test_params_ranges` 删除 `CHECK(p->sgthrs == 40, ...)` 与 SGTHRS 边界三行。
5. `test_registry_enum`：`fw_params_count() == 12` → `11`；`fw_params_name(11)` → `(10)`，`(12)` → `(11)`；`expect[]` 删除 `"SGTHRS",` 一项，循环上界 12 → 11。
6. `host_tests/cartproto_parse_test.c:49` 的 `p("CONFIG SGTHRS 0") && req.value == 0` 改为 `p("CONFIG HOME_VEL 50") && req.value == 50`，消息同步改。

Run: 两条宿主测试命令。
Expected: 编译失败——`CE_STALL`/`CR_STALL`/`sgthrs` 未定义（红）。

- [ ] **Step 2: 纯核心与参数层清理**

`cartfsm_core.h`：`cm_reason_t` 删 `CR_STALL,`；`cm_event_t` 删 `CE_STALL,` 行（连同注释）；文件头注释第 5 行「堵转在 HOMING 是预期的完成信号，在 READY 跟踪中才是故障。」改为「回零完成信号是位置偏差判据（达速锁存后 |E−C| 越限，homing-spec.md）；未达速的偏差越限是故障。」
`cartfsm_core.c`：`CM_HOMING` 中 `CE_STALL` case 删除（`CE_HOME_DONE` 已覆盖）；`default: return false; /* HOME/STEP_LOSS 忽略 */` 改为 `/* HOME 忽略 */` 并在 switch 内新增：
```c
        case CE_STEP_LOSS:          /* 未达速偏差：起点压挡块/卡死/域未对齐 */
            *mode = CM_FAULT;
            *reason = CR_STEP_LOSS;
            return true;
```
`CM_READY` 中 `CE_STALL` case 删除。
`params.h`：结构体删 `sgthrs`（行内注释「台位标定项」保留其余字段）。`params.c`：默认值删 `.sgthrs = 40,` 一行；`s_entries[]` 删 `{ "SGTHRS", ... }` 一行；文件头注释「（引擎加速度/看门狗/方向反转/堵转阈值）」去掉「/堵转阈值」。

Run: 两条宿主测试命令。
Expected: PASS（退出码 0；core 断言数按 Step 1 的删改变化，proto 恢复 49/49）。

- [ ] **Step 3: cartfsm.c 外壳重写**

按序对 `cartfsm.c` 做以下编辑（行号为当前 HEAD 锚点，以内容匹配为准）：

3a. 头部 `#include` 删 `"tmc2209.h"` 与 `"driver/gpio.h"`（本文件再无引用）。
3b. 删 `DIAG_ACTIVE_HIGH` 宏块（43-48 行）、`s_stall_report`（56 行）、`DIAG_DEBOUNCE`/`s_diag_high_run`/`s_last_sg_log`（60-63 行），替换为：
```c
/* ---- HOMING 到位判定状态（homing-spec §2） ---- */
static long s_last_e_um = 0;       /* v_meas 毫秒差分基准 */
static long s_v_meas_mm_s = 0;     /* 最近实测速度（诊断用） */
static bool s_speed_latch = false; /* 达速锁存 */
static int  s_speed_run = 0;       /* 达速连续拍计数（2 拍去抖） */
```
3c. flowtest 注入区（66-69 行）追加一个变量：
```c
static volatile bool s_test_follow_cmd = false;  /* 判定用 E 以指令位置替代 */
static volatile int  s_test_speed_latch = -1;    /* -1 自然；0 强制清；1 强制置 */
```
3d. `reason_str` 删 `case CR_STALL:` 两行。
3e. `params_apply_one` 删 `SGTHRS` 分支（`else if (strcmp(name, "SGTHRS") ...` 三行）。
3f. `CART_CMD_HOME`：删 `tmc2209_set_stall_threshold((uint8_t)p->sgthrs);`，在 `s_home_start = xTaskGetTickCount();` 后补锁存复位：
```c
        s_last_e_um = (long)as5600_position_um();
        s_v_meas_mm_s = 0;
        s_speed_latch = false;
        s_speed_run = 0;
```
动作注释「EN 开 + 堵转阈值 + 以 −HOME_VEL 起步」改「EN 开 + 以 −HOME_VEL 起步」。
3g. `fsm_task` 循环开头整块删除 DIAG 轮询（313-341 行，自「DIAG1 堵转轮询」注释起至 `s_stall_report = false;` 止）。
3h. HOMING 分支整体替换（351-425 行）为：
```c
        if (mode == CM_HOMING) {
            /* 回零速度按拍续喂：引擎看门狗按命令流设计（WDT_MS 窗口
             * 断流即减速），HOME 是单条命令，不续喂则 100 ms 后运动
             * 停止、HOMING 干等超时。回零是固件自主运动，由监督任务
             * 承担命令流角色；总时长上限仍由 HOME_TIMEOUT 把关。
             * 持锁复查：与 exec 的 HOMING 中止（RESET→DISABLED）
             * 交错时不得再喂，否则中止后引擎被重新起步 */
            portENTER_CRITICAL(&s_lock);
            if (s_mode == CM_HOMING) {
                step_engine_vel_target(mmps_to_q16(-(long)p->home_vel));
            }
            portEXIT_CRITICAL(&s_lock);

            /* 实测速度差分与达速锁存。v_meas 用原始 E：flowtest 的
             * 偏置注入只进偏差判定，不得污染速度判据（偏置阶跃会把
             * 锁存误置位，负路径用例假绿）。 */
            long e_raw;
#ifdef CONFIG_CARTFSM_FLOW_TEST
            if (s_test_follow_cmd) {
                e_raw = cmd_steps_to_um(step_engine_position_steps());
            } else
#endif
            {
                e_raw = (long)as5600_position_um();
            }
            s_v_meas_mm_s = e_raw - s_last_e_um;   /* µm/ms == mm/s */
            s_last_e_um = e_raw;
            if (cm_speed_reached(s_v_meas_mm_s, p->home_vel)) {
                if (++s_speed_run >= 2) {
                    s_speed_latch = true;
                }
            } else {
                s_speed_run = 0;
            }
#ifdef CONFIG_CARTFSM_FLOW_TEST
            if (s_test_speed_latch >= 0) {
                s_speed_latch = (s_test_speed_latch > 0);
            }
#endif

            long e_um = e_raw;
#ifdef CONFIG_CARTFSM_FLOW_TEST
            e_um += s_test_loss_bias_um;
#endif
            cm_homing_in_t hin = {
                .a5600_ok = a5600_ok,
                .m6835_ok = m6835_ok,
                .speed_latch = s_speed_latch,
                .e_um = e_um,
                .c_um = cmd_steps_to_um(step_engine_position_steps()),
                .step_loss_um = p->step_loss_um,
                .timed_out = (xTaskGetTickCount() - s_home_start)
                             >= pdMS_TO_TICKS(p->home_timeout_ms),
            };
            cm_event_t ev = cm_homing_event(&hin);

            if (ev != 0) {
                cm_mode_t m = mode;
                cm_reason_t r = s_reason;
                if (cm_step(&m, &r, ev)) {
                    if (m == CM_READY) {
                        /* 到位 = 完成：急停 + E/C 同帧重定基到 HOME 原点
                         *（E、C 必须同时换到新坐标系，否则 READY 首拍
                         *   |E−C| 丢步判定立即误报 FAULT/STEP_LOSS） */
                        step_engine_stop();
                        as5600_set_origin(HOME_ORIGIN_MM);
                        step_engine_set_position(
                            um_to_cmd_steps((long)(HOME_ORIGIN_MM * 1000.0)));
                        portENTER_CRITICAL(&s_lock);
                        s_mode = CM_READY;
                        s_reason = CR_NONE;
                        s_last_cmd_v = 0;
                        portEXIT_CRITICAL(&s_lock);
                        push_mode(CM_READY, CR_NONE);
                    } else {
                        if (r == CR_HOME_TIMEOUT) {
                            /* 超时诊断：区分「从未达速」（电机未随动/
                             * 速度过低/起点即堵）与「达速后未撞挡」
                             *（传动脱离） */
                            ESP_LOGW(TAG, "回零超时：E=%ld µm，C=%ld µm，"
                                     "v_meas=%ld mm/s，达速锁存=%d",
                                     e_raw, hin.c_um, s_v_meas_mm_s,
                                     s_speed_latch ? 1 : 0);
                        } else if (r == CR_STEP_LOSS) {
                            ESP_LOGW(TAG, "回零未达速即偏差越限：起点压"
                                     "挡块/皮带卡死/域未对齐——挪车后重试");
                        }
                        enter_fault(r);
                    }
                }
            }
        } else if (mode == CM_READY) {
            if (!a5600_ok) {
                enter_fault(CR_AS5600);
            } else if (!m6835_ok) {
                enter_fault(CR_MT6835);
            } else {
                /* 丢步判定：µm 域 |E−C|（流程自测可注入 E 偏置）。
                 * 运行中堵转由它捕获；回零后抵挡块静止属正常——
                 * C 不前进、D 不增长，无误报 */
                long e_um = (long)as5600_position_um();
#ifdef CONFIG_CARTFSM_FLOW_TEST
                e_um += s_test_loss_bias_um;
#endif
                long c_um = cmd_steps_to_um(step_engine_position_steps());
                if (cm_step_loss(e_um, c_um, p->step_loss_um)) {
                    enter_fault(CR_STEP_LOSS);
                } else {
                    /* 软限位持续钳制（运行中收紧立即生效） */
                    long eff = cm_soft_limit_clamp(
                        (long)as5600_position_um(), p->x_limit_um,
                        s_last_cmd_v);
                    if (eff != s_last_cmd_v) {
                        step_engine_vel_halt();
                        s_last_cmd_v = 0;
                    }
                }
            }
        } else if (mode == CM_FAULT && s_fault_settling) {
```
（其后 FAULT settling 分支原样保留。）
3i. 删 `cartfsm_report_stall` 实现（482-485 行），`cartfsm_mode` 后新增：
```c
cm_reason_t cartfsm_reason(void)
{
    return s_reason;
}
```
flowtest 钩子区（488 行起）追加：
```c
void cartfsm_test_follow_cmd(bool enable)
{
    s_test_follow_cmd = enable;
}

void cartfsm_test_speed_latch(int state)
{
    s_test_speed_latch = state;
}
```

`cartfsm.h`：删 `cartfsm_report_stall` 声明及其注释块（31-35 行）；`cartfsm_mode` 声明后加 `cm_reason_t cartfsm_reason(void);`（注释「当前原因（枚举原子读）」）；flowtest 钩子区加两个声明（见 Interfaces）；文件头注释「回零完成/超时」改「回零到位判定（位置偏差+达速锁存）/超时」。

- [ ] **Step 4: selftest 与 flowtest 重写**

`cartfsm_self_test.c`：`test_transitions_spot` 中 `CE_STALL` → `CE_HOME_DONE`，消息「堵转完成→READY」改「到位完成→READY」；文件头注释「假 stall/时钟」改「假到位/时钟」。

`cartfsm_flow_test.c` 重写为以下用例集（`to_disabled`/`wait_mode`/`wait_fault_settled`/`en_is_off`/`RUN_TEST` 骨架保留原样）：

```c
/* 1. HOME → 注入到位（达速锁存 + 偏差） → READY，原点重置 */
static bool test_home_done_to_ready(void)
{
    to_disabled();
    cartfsm_test_follow_cmd(false);
    cartfsm_test_speed_latch(-1);

    cart_req_t home = { .cmd = CART_CMD_HOME };
    cartfsm_test_exec(&home);
    CHECK(wait_mode(CM_HOMING, 200), "应进入 HOMING");

    cartfsm_test_speed_latch(1);             /* 达速锁存置位 */
    cartfsm_test_inject_loss_bias(10000);    /* 偏差 10 mm > T 3 mm */
    CHECK(wait_mode(CM_READY, 500), "到位后应 READY");
    CHECK(cartfsm_reason() == CR_NONE, "READY 原因为 NONE");
    cartfsm_test_inject_loss_bias(0);

    vTaskDelay(pdMS_TO_TICKS(50));
    int64_t x = as5600_position_um();
    CHECK(x > -266000 && x < -264000, "原点应重置为 −265000 µm 附近");

    to_disabled();
    CHECK(cartfsm_mode() == CM_DISABLED, "RESET 回 DISABLED");
    CHECK(en_is_off(), "DISABLED 时 EN 关");
    return true;
}

/* 1b. HOME → 未达速偏差 → FAULT/STEP_LOSS（起点压挡块/卡死/域反向） */
static bool test_home_loss_before_speed(void)
{
    to_disabled();
    cartfsm_test_follow_cmd(false);
    cartfsm_test_speed_latch(-1);

    cart_req_t home = { .cmd = CART_CMD_HOME };
    cartfsm_test_exec(&home);
    CHECK(wait_mode(CM_HOMING, 200), "应进入 HOMING");

    cartfsm_test_speed_latch(0);             /* 锁存强制清零 */
    cartfsm_test_inject_loss_bias(10000);
    CHECK(wait_mode(CM_FAULT, 500), "未达速偏差应 FAULT");
    CHECK(cartfsm_reason() == CR_STEP_LOSS, "原因为 STEP_LOSS");
    cartfsm_test_inject_loss_bias(0);
    CHECK(wait_fault_settled(2000), "EN 关");

    to_disabled();
    return true;
}

/* 2. HOME → 超时 → FAULT/HOME_TIMEOUT（理想编码器跟随，不依赖 12V） */
static bool test_home_timeout(void)
{
    to_disabled();
    cartfsm_test_speed_latch(-1);
    CHECK(fw_params_set("HOME_TIMEOUT", 1000), "临时收紧超时 1 s");

    cart_req_t home = { .cmd = CART_CMD_HOME };
    cartfsm_test_exec(&home);
    CHECK(wait_mode(CM_HOMING, 200), "应进入 HOMING");
    cartfsm_test_follow_cmd(true);    /* E 跟随指令：D 恒 0，只可能超时 */

    /* 回零运动必须持续整个超时窗（监督任务续喂引擎看门狗——曾因
     * 断喂在 100 ms 后减速停车，超时断言无法暴露，此处锚定；
     * 步数为逻辑量，不依赖 12V） */
    vTaskDelay(pdMS_TO_TICKS(300));
    int32_t p1 = step_engine_position_steps();
    vTaskDelay(pdMS_TO_TICKS(300));
    int32_t p2 = step_engine_position_steps();
    CHECK(p2 < p1 - 100, "HOMING 应持续运动（监督任务喂狗）");

    CHECK(wait_mode(CM_FAULT, 3000), "1 s 超时后应 FAULT");
    CHECK(cartfsm_reason() == CR_HOME_TIMEOUT, "原因为 HOME_TIMEOUT");
    CHECK(wait_fault_settled(2000), "FAULT 停机次序完成（EN 关）");

    cartfsm_test_follow_cmd(false);
    fw_params_set("HOME_TIMEOUT", 30000);
    to_disabled();
    return true;
}

/* 3. HOMING 中注入 AS5600 失联 → FAULT/AS5600 优先于偏差与超时 */
static bool test_homing_encoder_fail_priority(void)
{
    to_disabled();
    cartfsm_test_speed_latch(-1);

    cart_req_t home = { .cmd = CART_CMD_HOME };
    cartfsm_test_exec(&home);
    CHECK(wait_mode(CM_HOMING, 200), "应进入 HOMING");
    cartfsm_test_follow_cmd(true);

    TickType_t t0 = xTaskGetTickCount();
    cartfsm_test_inject_encoder_fail(1);  /* AS5600 失联 */
    CHECK(wait_mode(CM_FAULT, 100), "失联后应立即 FAULT（先于超时）");
    CHECK(cartfsm_reason() == CR_AS5600, "原因为 AS5600");
    cartfsm_test_inject_encoder_fail(0);
    cartfsm_test_follow_cmd(false);
    CHECK(wait_fault_settled(2000), "EN 关");
    ESP_LOGI(TAG, "  失联→FAULT+EN 关 耗时 %u ms（实测，含调度）",
             (unsigned)(xTaskGetTickCount() - t0));

    to_disabled();
    return true;
}

/* 3b. READY 中注入 |E−C| 偏差 → FAULT/STEP_LOSS */
static bool test_ready_step_loss(void)
{
    to_disabled();
    cartfsm_test_follow_cmd(false);
    cartfsm_test_speed_latch(-1);

    cart_req_t home = { .cmd = CART_CMD_HOME };
    cartfsm_test_exec(&home);
    CHECK(wait_mode(CM_HOMING, 200), "进入 HOMING");
    cartfsm_test_speed_latch(1);
    cartfsm_test_inject_loss_bias(10000);
    CHECK(wait_mode(CM_READY, 500), "回零完成进 READY");
    cartfsm_test_inject_loss_bias(0);

    cartfsm_test_inject_loss_bias(10000);   /* 10 mm > STEP_LOSS_UM 3000 */
    CHECK(wait_mode(CM_FAULT, 500), "偏差超阈值应 FAULT");
    CHECK(cartfsm_reason() == CR_STEP_LOSS, "原因为 STEP_LOSS");
    cartfsm_test_inject_loss_bias(0);
    CHECK(wait_fault_settled(2000), "EN 关");

    to_disabled();
    return true;
}

/* 4. FAULT 后 RESET → 可重新 HOME 全流程 */
static bool test_reset_then_rehome(void)
{
    to_disabled();
    cartfsm_test_follow_cmd(false);
    cartfsm_test_speed_latch(-1);

    cart_req_t home = { .cmd = CART_CMD_HOME };
    cartfsm_test_exec(&home);
    CHECK(wait_mode(CM_HOMING, 200), "重新 HOME 生效");
    cartfsm_test_speed_latch(1);
    cartfsm_test_inject_loss_bias(10000);
    CHECK(wait_mode(CM_READY, 500), "再次回零完成");
    cartfsm_test_inject_loss_bias(0);
    to_disabled();
    return true;
}
```

`cartfsm_flow_test_run` 的 RUN 列表与文件头注释同步：
```c
    RUN_TEST(test_home_done_to_ready);
    RUN_TEST(test_home_loss_before_speed);
    RUN_TEST(test_home_timeout);
    RUN_TEST(test_homing_encoder_fail_priority);
    RUN_TEST(test_ready_step_loss);
    RUN_TEST(test_reset_then_rehome);
```
头部注释用例清单改为：「1. HOME→注入到位→READY 且原点 −265000 µm；1b. HOME→未达速偏差→FAULT/STEP_LOSS；2. HOME 超时→FAULT/HOME_TIMEOUT（理想编码器跟随）；3. HOMING 中 AS5600 失联→FAULT 优先；3b. READY 偏差→FAULT/STEP_LOSS；4. RESET→DISABLED 后可重新 HOME。」

- [ ] **Step 5: 全部门禁**

Run: 两条宿主测试命令 + 三构建命令。
Expected: 宿主 PASS 退出码 0；build（正式）成功——注意正式构建无 `CONFIG_CARTFSM_FLOW_TEST`，`#ifdef` 块内代码不编译；build-selftest / build-flowtest 成功。

- [ ] **Step 6: Commit**（固件仓库）

```bash
cd tmp/PD-STEP && git add components/cartfsm host_tests
git commit -m "cartfsm：回零完成信号切换为位置偏差判据（达速锁存+决策表接入外壳；CE_STALL/CR_STALL/SGTHRS/cartfsm_report_stall 退役；flowtest 六用例重写）"
```

---

### Task 3: tmc2209 StallGuard 接口与 bsp DIAG 引脚退役

Task 2 之后固件已无任何调用方，本任务删除接口与引脚配置本身。

**Files:**
- Modify: `tmp/PD-STEP/components/tmc2209/include/tmc2209.h`（删三个声明及注释块）
- Modify: `tmp/PD-STEP/components/tmc2209/tmc2209.c`（删三个函数、相关寄存器定义与写入）
- Modify: `tmp/PD-STEP/components/bsp/bsp.c` + `include/bsp.h`（删 DIAG 引脚）
- Modify: `tmp/PD-STEP/components/bsp/Kconfig`、`tmp/PD-STEP/components/cartfsm/Kconfig`
- Modify: `tmp/PD-STEP/sdkconfig`、`build-selftest/sdkconfig`、`build-flowtest/sdkconfig`（删存量符号行）

**Interfaces:**
- Consumes: 无（纯删除）。
- Produces: `tmc2209.h` 对外只剩电流/细分/使能/寄存器层/状态层/纯函数；`BSP_PIN_DIAG` 不存在。

- [ ] **Step 1: tmc2209 删除**

`tmc2209.h`：删除 `tmc2209_set_stall_threshold`（108-117 行区域）、`tmc2209_sg_result`（119-123）、`tmc2209_tstep`（125-129）三段声明及 doc 注释。
`tmc2209.c`：
1. 寄存器定义区删 `REG_TSTEP`、`REG_TCOOLTHRS`、`REG_SGTHRS` 三行及 50-52 行的 SGTHRS/SG_RESULT 注释块。
2. 删三个函数实现（446-476 行区域）。
3. `tmc2209_init` 中删 TCOOLTHRS 写入块（633-637 行，含注释）。
4. GCONF 注释（585-587 行）改为：
```c
    /* GCONF: pdn_disable=1 (UART mode), mstep_reg_select=1,
     * en_spreadcycle=1（固定 SpreadCycle：全速域统一 chopper，
     * TPWMTHRS 不参与模式切换）, shaft=0 */
```
（`REG_TPWMTHRS` 的写 0 与注释保留。）

- [ ] **Step 2: bsp 与 Kconfig 删除**

`bsp.c`：删 `#define BSP_PIN_DIAG CONFIG_BSP_DIAG_GPIO`（30 行）；删 DIAG 的 `gpio_config` 块（97-106 行，含开漏上拉注释）；GPIO 日志行改为不含 DIAG：
```c
    ESP_LOGI(TAG, "GPIO: STEP=%d (OUT LOW), DIR=%d (OUT LOW), EN=%d (OUT HIGH=disabled)",
             BSP_PIN_STEP, BSP_PIN_DIR, BSP_PIN_EN);
```
`bsp_init` 内注释「GPIO (STEP / DIR / DIAG)」改「GPIO (STEP / DIR / EN)」。
`bsp.h` 40 行注释「STEP output LOW, DIR output LOW, DIAG input.」改「STEP output LOW, DIR output LOW.」。
`bsp/Kconfig`：菜单标题行「Stepper signals (STEP / DIR / EN / DIAG)」去「 / DIAG」；删 `config BSP_DIAG_GPIO` 整块（66 行起至该 config 结束）。
`cartfsm/Kconfig`：删 `CARTFSM_DIAG_ACTIVE_HIGH` 整块（10-18 行）；`CARTFSM_FLOW_TEST` help 文本中「HOME→注入堵转→READY」改「HOME→注入到位→READY；未达速偏差→FAULT/STEP_LOSS」。

- [ ] **Step 3: 三份 sdkconfig 清理**（598c18d 教训：删 Kconfig 选项必须清存量）

```bash
cd tmp/PD-STEP
sed -i '/BSP_DIAG_GPIO/d; /CARTFSM_DIAG_ACTIVE_HIGH/d' sdkconfig build-selftest/sdkconfig build-flowtest/sdkconfig
```
（当前行位：`sdkconfig` 3850/3877，`build-selftest/sdkconfig` 3849/3875，`build-flowtest/sdkconfig` 3850/3876。）

- [ ] **Step 4: 门禁与残留扫描**

Run: 三构建命令；然后
```bash
cd tmp/PD-STEP && grep -rn "STALL\|SGTHRS\|sgthrs\|SG_RESULT\|TCOOLTHRS\|DIAG" components/ main/ host_tests/ --include=*.c --include=*.h | grep -v "install"
```
Expected: 三构建成功；grep 无命中（`install` 误匹配已滤除）。

- [ ] **Step 5: Commit**（固件仓库）

```bash
cd tmp/PD-STEP && git add components sdkconfig build-selftest/sdkconfig build-flowtest/sdkconfig
git commit -m "tmc2209/bsp：StallGuard 接口与 DIAG 引脚退役（SGTHRS/SG_RESULT/TSTEP/TCOOLTHRS、GPIO16 配置、Kconfig 与三份 sdkconfig 清理）"
```

---

### Task 4: 端侧 edge 枚举同步

**Files:**
- Modify: `deploy/v2/edge/proto.h`（25 行删 `FW_REASON_STALL,`）
- Modify: `deploy/v2/edge/proto.c`（75-76 行删 parse 分支；238-239 行删 name case）

**Interfaces:**
- Consumes: 无。
- Produces: `fw_reason_t` 终集 = `FW_REASON_NONE/STEP_LOSS/AS5600/MT6835/HOME_TIMEOUT`（与 protocol.md reason 表一一对应）。

- [ ] **Step 1: 删除枚举与解析/命名两处**

`proto.h` 枚举删 `FW_REASON_STALL,` 一行；`proto.c` 删
```c
    } else if (word_eq(w, len, "STALL")) {
        *out = FW_REASON_STALL;
```
与
```c
    case FW_REASON_STALL:
        return "STALL";
```

- [ ] **Step 2: 测试与残留扫描**

Run: `wsl make -C deploy/v2/edge test`（在 cartpole 仓库根）。
Expected: proto_test / policy_test / console_test 全过（`proto_test.c` 无 STALL 断言，无需改动；若 make 报错先看枚举是否漏删）。
```bash
grep -rn "STALL\|SGTHRS" deploy/v2/edge/
```
Expected: 无命中。

- [ ] **Step 3: Commit**（cartpole 仓库）

```bash
git add deploy/v2/edge/proto.h deploy/v2/edge/proto.c
git commit -m "端侧：reason 枚举移除 STALL（随固件判据更换，协议同步）"
```

---

### Task 5: 协议、清单与规格文档修订

**Files:**
- Modify: `deploy/v2/protocol.md`
- Modify: `deploy/v2/onboard-checklist.md`
- Modify: `deploy/v2/homing-spec.md`（两处与实现校准）
- Modify: `tmp/PD-STEP/README.md`

**Interfaces:**
- Consumes: Task 2-4 已固化的固件行为。
- Produces: 文档与固件一致；端侧/台位操作者以此为准。

- [ ] **Step 1: protocol.md 八处修订**（行号为当前版本锚点）

1. 39 行模式表 READY 行进入方式「HOMING 堵转触发，回零完成」→「HOMING 到位判定，回零完成」。
2. 42 行转换表引言「堵转在 HOMING 是预期的完成信号，在 READY 跟踪中才是故障：」→「回零完成信号是位置偏差判据——达速（实测速度达到回零方向 80%，单向锁存）后 |指令位置−实测位置| 越过 `STEP_LOSS_UM` 判到位；未达速时的偏差越限是故障（起点压挡块、皮带卡死或域未对齐）。判定细则见 `homing-spec.md`：」
3. 47 行转换行「| HOMING | 堵转触发 | READY（触发点记为 `x = −265000 µm`） |」替换为两行：
   ```
   | HOMING | 到位判定（达速后偏差越限） | READY（触发点记为 `x = −265000 µm`） |
   | HOMING | 偏差越限且未达速 | FAULT（`STEP_LOSS`） |
   ```
4. 51 行 READY 行「丢步 / 堵转（运动中） / AS5600 或 MT6835 读取失败」→「丢步 / AS5600 或 MT6835 读取失败」。
5. 66 行 reason 表删「| `STALL` | 堵转（READY 跟踪中） |」行。
6. 115 行参数表删 SGTHRS 行；116 行 `STEP_LOSS_UM` 含义列改为「丢步判定阈值：指令位置与 AS5600 实测偏差超过即 FAULT；兼作 HOMING 到位判定阈值（确认延迟 = 阈值/`HOME_VEL`）」。
7. 118 行「表中后五项为台位标定量」→「表中后四项为台位标定量」；同段「`SGTHRS` 与 `STEP_LOSS_UM` 须实机调校」→「`STEP_LOSS_UM` 须实机调校」。
8. 146 行未决事项 StallGuard 阈值调校条删除（判据已换，撤销）；145 行「回零重复性实测（堵转触发点散布，预期 ±1–2 mm）」改「回零重复性实测（到位判定散布，预期 ±1–2 mm）」。

- [ ] **Step 2: onboard-checklist.md 六处修订**

1. C1 行改为：
   ```
   | C1 | 到位判定验证 | `py home` 后小车走到挡块停住 | `MODE READY NONE`、x_um ≈ −265000；再验负路径：起点压挡块时 `py home` → FAULT STEP_LOSS（日志含挪车提示） | |
   ```
2. 删除 C2 行（SGTHRS 扫描）。
3. B1 预期列「注入堵转→READY+原点 −265000 µm；超时→FAULT；AS5600 失联→FAULT（日志含耗时 ms，设计 ≤15）；丢步注入→FAULT；RESET 后重 HOME」→「注入到位→READY+原点 −265000 µm；未达速偏差→FAULT/STEP_LOSS；超时→FAULT；AS5600 失联→FAULT（日志含耗时 ms，设计 ≤15）；丢步注入→FAULT；RESET 后重 HOME」（已回填的记录列不动）。
4. 阶段 D「`POLE_ZERO_URAD / POLE_INV / X_INV / SGTHRS`（及实测微调的 `STEP_LOSS_UM`）」→「`POLE_ZERO_URAD / POLE_INV / X_INV`（及实测微调的 `STEP_LOSS_UM`）」。
5. 未决②条改为：「StallGuard 低速可靠性 → 已裁决改位置偏差判据（`homing-spec.md`），本项关闭；C1 验证新判据。」
6. 未决③条改为：「`STEP_LOSS_UM` 初值（兼到位阈值）→ C1/C3 实测回填。」

- [ ] **Step 3: homing-spec.md 两处校准**（与实现核对后的修正，非行为变更）

1. §4 端侧行「`config.h` 推送集移除 `SGTHRS`」→「参数推送表（`rig.c` 的 `CFG_FW_PARAMS`）本不含 `SGTHRS`，无需改动」。
2. §6 注入点列表改为三项：`loss_bias`（既有，驱动 `D`）；`follow_cmd`（判定用 E 以指令位置替代——理想编码器替身，使超时/失联用例不依赖 12V）；`speed_latch` 覆写（-1 自然 / 0 强制清 / 1 强制置）。

- [ ] **Step 4: 固件 README 四处修订**

1. 6 行「StallGuard 堵转回零」→「位置偏差判据回零」。
2. 17 行硬件表 TMC2200 行：删「DIAG1=16、」与「+ StallGuard」。
3. 42 行「tmc2209/ 驱动配置（电流/细分/StallGuard 阈值）」→「tmc2209/ 驱动配置（电流/细分）」。
4. 92-93 行「上板验收按 `docs/reviews/T8-acceptance.md`（回零散布/StallGuard 标定/DIAG 极性）与 …」→「上板验收按 `deploy/v2/onboard-checklist.md`（C 阶段）与 `docs/reviews/T10-acceptance.md`（端到端五用例）」。

- [ ] **Step 5: Commit**（两个仓库各一次）

```bash
# cartpole 仓库
git add deploy/v2/protocol.md deploy/v2/onboard-checklist.md deploy/v2/homing-spec.md
git commit -m "协议/清单/规格：回零完成信号改位置偏差判据的文档同步（转换表两行、STALL reason 与 SGTHRS 参数退役、清单 C1/C2 与未决出口改写）"
# 固件仓库
cd tmp/PD-STEP && git add README.md
git commit -m "README：回零判据与验收指引同步（StallGuard/DIAG 字样清理）"
```

---

### Task 6: 上板验收（需台位操作者执行；Agent 准备命令与判读，用户跑）

**Files:**
- Modify（回填记录）: `deploy/v2/onboard-checklist.md`

**Interfaces:**
- Consumes: Task 2/3 产出的 flowtest 与正式固件；Task 5 修订后的清单。
- Produces: 验收记录回填；未决②正式关闭。

- [ ] **Step 1: flowtest 上板**（PowerShell）

```powershell
idf.py -B build-flowtest -DSDKCONFIG=build-flowtest/sdkconfig -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;tools/flowtest.defaults" flash monitor
```
Expected: `cartfsm 流程自测` 六用例 `SELF-TEST DONE: 6/6 PASS`；UART1 抓 B2 的 MODE 序列与六用例一一对应（新序列含未达速偏差用例的 `MODE FAULT STEP_LOSS`）。

- [ ] **Step 2: 正式固件上板**（PowerShell `idf.py -B build flash`，host 工具 `py` 同清单）

| 操作 | 预期 |
|---|---|
| `py home` | 小车走到挡块干脆停住 → `MODE READY NONE`，静止 `x_um` ≈ −265000 |
| 负路径：手把小车推到 −x 挡块压住 → `py reset` → `py home` | FAULT/STEP_LOSS，UART0 日志含「挪车后重试」；挪车 ≥5 mm 后重新 `home` 正常完成 |
| 静置 1 min（READY 抵挡块） | 无 FAULT |
| `py home` ×5（间隔 `py reset`）散布 | ≤ 2000 µm（清单 C3） |
| `py vel 50 --hold 3` | x_um 增大、无 STEP_LOSS 误报（C3b，判据前提） |
| READY 手推小车 ≥3 mm | FAULT/STEP_LOSS |

- [ ] **Step 3: 记录回填与提交**

C1/C3/C3b 记录回填 `onboard-checklist.md`，未决②关闭；cartpole 仓库提交「清单：C1–C3 位置偏差判据验收记录回填，未决②关闭」。

---

## 自审记录（写计划时已核对）

- 规格覆盖：§1 信号定义→Task 2 外壳；§2 决策表→Task 1 纯核心 + Task 2 接入；§3 READY 修订→Task 2；§4 协议修订→Task 4/5；§5 驱动退役→Task 3；§6 测试验收→Task 1/2/6；§7 局限为记录性，无任务；§8 实现范围与任务划分一致；§9 未决留给 Task 6 回填。
- 占位符：无 TBD/「适当处理」类步骤。
- 类型一致性：`cm_homing_in_t` 字段、`cartfsm_test_speed_latch(int)`/`cartfsm_test_follow_cmd(bool)`、`cartfsm_reason(void)` 在 Task 1/2/6 间签名一致。
- flowtest 的 12V 无关性：超时/失联用例经 `follow_cmd` 注入实现；到位/负路径用例经 `speed_latch` 覆写实现（Review Focus 第 4 条的污染路径在 Step 3 的外壳代码里以「v_meas 用原始 E」封死）。
