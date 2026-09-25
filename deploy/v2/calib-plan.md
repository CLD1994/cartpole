# 手动两点标定实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 回零方式改为手动两点标定（MARK LEFT/MARK RIGHT/ENABLE，中点为 0，X_MARGIN_UM 软限位），模式机三态化，被否决的位置偏差回零代码全量退役。

**Architecture:** 标定状态机（CAL_NONE/CAL_LEFT/CAL_DONE）与命令执行全部在 cartfsm 外壳（MARK 不改模式，无纯核心事件）；纯核心只做三态转换表、L 合法性与软限位窗口钳位（宿主可测）。cartproto 扩展 MARK/ENABLE 词法；端侧同步推送集、console 流程与 reason/mode 枚举。

**Tech Stack:** ESP-IDF v6.1（三构建门禁）、C11 宿主测试（WSL gcc）、端侧 C（WSL make）、Python host 工具。

**Spec:** `deploy/v2/calib-spec.md`（行为契约权威）。执行前先通读。

## Global Constraints

- 权威：行为以 `deploy/v2/calib-spec.md` 为准，线协议文本以 `deploy/v2/protocol.md` 为准；计划与规格冲突时停止上报。
- 仓库：固件 `tmp/PD-STEP`（分支 cartpole）；文档/端侧在 cartpole 仓库（master）。提交信息中文「域：描述」。
- 宿主测试（在 `tmp/PD-STEP` 下，已实证；wsl 的 NAT localhost 警告无害）：
  ```bash
  wsl gcc -std=c11 -Wall -Wextra -Werror -Ihost_tests/stub \
    -Icomponents/cartfsm/include host_tests/cartfsm_core_test.c \
    components/cartfsm/cartfsm_core.c components/cartfsm/params.c \
    -o host_tests/.build_core_test && wsl ./host_tests/.build_core_test \
    && wsl rm -f host_tests/.build_core_test
  ```
  （当前基线 111/111，退出码 0 = 全过）
  ```bash
  wsl gcc -std=c11 -Wall -Wextra -Werror -Ihost_tests/stub \
    -Icomponents/cartproto/include host_tests/cartproto_parse_test.c \
    components/cartproto/cartproto_parse.c -o host_tests/.build_proto_test \
    && wsl ./host_tests/.build_proto_test && wsl rm -f host_tests/.build_proto_test
  ```
  （当前基线 49/49）
- 固件三构建门禁：PowerShell（非 Git Bash）里先 `$env:MSYSTEM=$null` 再 dot `C:\esp\v6.1\esp-idf\export.ps1`（否则 idf_tools 报 MSys 不支持），依次：
  ```powershell
  idf.py -B build build
  idf.py -B build-selftest -DSDKCONFIG=build-selftest/sdkconfig -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;tools/selftest.defaults" build
  idf.py -B build-flowtest -DSDKCONFIG=build-flowtest/sdkconfig -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;tools/flowtest.defaults" build
  ```
  建议 Task 2 Step 0 重建脚本 `.superpowers/sdd/calib-plan/build_gates.ps1`（内容 = 上述顺序 + 每步 `$LASTEXITCODE` 检查 + 末行 `ALL GATES PASSED`），之后一条命令跑三门禁。
- 端侧测试（cartpole 仓库根）：`wsl make -C deploy/v2/edge test`（当前全过）。
- 回执名是单 token：`MARK_LEFT` / `MARK_RIGHT` / `ENABLE`（规格 §2，OK/ERR 行两字段语法）。
- L 合法窗常量：`100000 µm ≤ L ≤ 1000000 µm`（规格 §2，硬编码，非参数）。
- 参数注册表终集九项：VEL_MAX/ACC_MAX/WDT_MS/WDT_DEC/X_MARGIN_UM/POLE_ZERO_URAD/POLE_INV/X_INV/STEP_LOSS_UM（规格 §5）。
- 测试全绿不能替代规格覆盖检查；发现冲突停下列出。

## Review Focus

规格隐含但任务测试未必覆盖、最可能咬人的五类（每条钉到 owning task）：

1. **标定完成到 ENABLE 之间手挪车** → C 重锚必须先行，否则 READY 首拍 STEP_LOSS 误报——Task 2 flowtest `test_calib_and_enable` 的「ENABLE 后 200 ms 无 FAULT」断言钉住；上板复核。
2. **域反向（先右后左标）** → L 为负必须拒且状态保持——Task 2 宿主 `test_calib_valid` 负值向量 + flowtest `test_calib_invalid_l`。
3. **ENABLE 时小车越出软限位窗口** → 重锚照做、向外钳 0 向内放行——Task 2 宿主 `test_soft_limit`（窗口边界双侧）+ 上板软限位行。
4. **断电丢标定（RAM）** → 上电即 ENABLE 必须拒——Task 2 flowtest `test_enable_uncalibrated`（配 `cartfsm_test_reset_calib` 钩子模拟上电）+ 上板断电复测。
5. **回执名单 token 一致性** → 固件发 `OK MARK_LEFT …` 而端侧按 `OK <name> <value>` 解析——Task 3 `proto_test` 的 OK 行解析向量（若现有向量覆盖 name 字段则改词，否则补一条 `OK MARK_LEFT 123` 解析断言）。

---

### Task 1: cartproto 词法扩展（纯增量）

**Files:**
- Modify: `tmp/PD-STEP/components/cartproto/include/cartproto.h:61-68`
- Modify: `tmp/PD-STEP/components/cartproto/cartproto_parse.c`
- Test: `tmp/PD-STEP/host_tests/cartproto_parse_test.c`

**Interfaces:**
- Consumes: 无。
- Produces: `cart_cmd_t` 新增 `CART_CMD_MARK_LEFT / CART_CMD_MARK_RIGHT / CART_CMD_ENABLE`（本任务**保留** `CART_CMD_HOME`，Task 2 一并退役，保证本任务可编译）。

- [ ] **Step 1: 写失败测试**——`host_tests/cartproto_parse_test.c` 的合法集（`test_legal`）中 `RESET` 行后追加：

```c
    CHECK(p("ENABLE") && req.cmd == CART_CMD_ENABLE, "ENABLE");
    CHECK(p("MARK LEFT") && req.cmd == CART_CMD_MARK_LEFT, "MARK LEFT");
    CHECK(p("MARK RIGHT") && req.cmd == CART_CMD_MARK_RIGHT, "MARK RIGHT");
```

非法集（`test_illegal`）末尾追加：

```c
    CHECK(!p("MARK"), "MARK 缺方向词");
    CHECK(!p("MARK UP"), "MARK 未知方向词");
    CHECK(!p("MARK LEFT extra"), "MARK 多余参数");
    CHECK(!p("MARK  LEFT"), "MARK 双空格分隔");
    CHECK(!p("MARKLEFT"), "MARK 粘连");
    CHECK(!p("ENABLE 1"), "ENABLE 带参数");
```

- [ ] **Step 2: 跑红**

Run: Global Constraints 的 proto 宿主测试命令。
Expected: 编译失败——`CART_CMD_MARK_LEFT` 等未定义。

- [ ] **Step 3: 实现**

`cartproto.h` 枚举 `CART_CMD_HOME,` 行后追加三行：
```c
    CART_CMD_MARK_LEFT,      /* MARK LEFT */
    CART_CMD_MARK_RIGHT,     /* MARK RIGHT */
    CART_CMD_ENABLE,
```

`cartproto_parse.c` 无参命令段（`cmd_len == len` 分支）：`RESET` 判定块后追加：
```c
        if (cmd_len == 6 && line[0] == 'E' && line[1] == 'N'
            && line[2] == 'A' && line[3] == 'B' && line[4] == 'L'
            && line[5] == 'E') {
            out->cmd = CART_CMD_ENABLE;
            return true;
        }
```

带参命令段（`size_t rest = cmd_len + 1;` 之后、VEL 解析之前）插入：
```c
    /* MARK <LEFT|RIGHT>：命令词 + 字面方向词（规格 §2） */
    if (cmd_len == 4 && line[0] == 'M' && line[1] == 'A'
        && line[2] == 'R' && line[3] == 'K') {
        size_t rl = len - rest;
        if (rl == 4 && line[rest] == 'L' && line[rest + 1] == 'E'
            && line[rest + 2] == 'F' && line[rest + 3] == 'T') {
            out->cmd = CART_CMD_MARK_LEFT;
            return true;
        }
        if (rl == 5 && line[rest] == 'R' && line[rest + 1] == 'I'
            && line[rest + 2] == 'G' && line[rest + 3] == 'H'
            && line[rest + 4] == 'T') {
            out->cmd = CART_CMD_MARK_RIGHT;
            return true;
        }
        return false;
    }
```

- [ ] **Step 4: 跑绿 + 三构建**

Run: proto 宿主测试（Expected: PASS，断言数 49+9=58 级）；三构建门禁。
Expected: 全绿（纯增量，exec 未消费新枚举值不报警告——枚举 switch 无 default 时未覆盖值在 -Wswitch 下仅对 switch 语句报警；`exec_cmd` 的 switch 无 default 分支则需确认编译通过，若 -Werror 报未覆盖枚举值，给 switch 加 `default: break;`）。

- [ ] **Step 5: Commit**（固件仓库）

```bash
cd tmp/PD-STEP && git add components/cartproto host_tests/cartproto_parse_test.c
git commit -m "cartproto：命令词法扩展 MARK LEFT/MARK RIGHT/ENABLE（回执名单 MARK_LEFT/MARK_RIGHT 单 token；HOME 退役在下一提交）"
```

---

### Task 2: 三态化与标定原子切换（核心+外壳+参数+flowtest）

`CM_HOMING`/`CE_CMD_HOME`/`CE_HOME_DONE`/`CE_HOME_TIMEOUT`/`CR_HOME_TIMEOUT`/`home_vel`/`home_timeout_ms`/`x_limit_um` 在 cartfsm.c、selftest、flowtest 三处引用必须一次切换，保证提交可编译。

**Files:**
- Modify: `tmp/PD-STEP/components/cartfsm/include/cartfsm_core.h` + `cartfsm_core.c`
- Modify: `tmp/PD-STEP/components/cartfsm/include/params.h` + `params.c`
- Modify: `tmp/PD-STEP/components/cartfsm/cartfsm.c` + `include/cartfsm.h`
- Modify: `tmp/PD-STEP/components/cartfsm/cartfsm_self_test.c` + `cartfsm_flow_test.c`
- Modify: `tmp/PD-STEP/components/cartproto/include/cartproto.h` + `cartproto_parse.c`（HOME 退役）
- Test: `tmp/PD-STEP/host_tests/cartfsm_core_test.c` + `cartproto_parse_test.c`

**Interfaces:**
- Consumes: Task 1 的 `CART_CMD_MARK_LEFT/MARK_RIGHT/ENABLE`。
- Produces（后续任务/上板依赖，签名不得改）:
  ```c
  bool cm_calib_valid(long l_um);                 /* 100000 ≤ l ≤ 1000000 */
  long cm_soft_limit_clamp(long x_um, long x_min_um, long x_max_um, long v_mm_s);
  cm_mode_t cartfsm_mode(void);                   /* 三态：无 CM_HOMING */
  /* 仅 CONFIG_CARTFSM_FLOW_TEST 编译： */
  void cartfsm_test_reset_calib(void);            /* 模拟上电：CAL_NONE */
  calib_state_t cartfsm_calib(void);              /* CAL_NONE/CAL_LEFT/CAL_DONE，flowtest 断言用 */
  ```

- [ ] **Step 0: 重建构建门禁脚本**

Write `.superpowers/sdd/calib-plan/build_gates.ps1`（cartpole 仓库，内容见 Global Constraints：`$env:MSYSTEM=$null` → export.ps1 → 三次 idf.py build 各查 `$LASTEXITCODE` → `ALL GATES PASSED`）。

- [ ] **Step 1: 宿主向量先行（红）**

`host_tests/cartfsm_core_test.c`：
1. `test_transitions` 整体重写为三态表：
```c
static void test_transitions(void)
{
    /* DISABLED + ENABLE → READY/NONE */
    m = CM_DISABLED; r = CR_NONE;
    CHECK(step(CE_CMD_ENABLE) && m == CM_READY && r == CR_NONE,
          "DISABLED+ENABLE→READY");

    /* DISABLED 忽略其余命令/故障事件 */
    m = CM_DISABLED;
    CHECK(!step(CE_CMD_RESET), "DISABLED+RESET 忽略");
    CHECK(!step(CE_STEP_LOSS), "DISABLED+STEP_LOSS 忽略");
    CHECK(!step(CE_AS5600_FAIL), "DISABLED+AS5600 忽略");
    CHECK(!step(CE_MT6835_FAIL), "DISABLED+MT6835 忽略");

    /* READY 故障三源 */
    m = CM_READY;
    CHECK(step(CE_STEP_LOSS) && m == CM_FAULT && r == CR_STEP_LOSS,
          "READY+STEP_LOSS→FAULT");
    m = CM_READY;
    CHECK(step(CE_AS5600_FAIL) && m == CM_FAULT && r == CR_AS5600,
          "READY+AS5600→FAULT");
    m = CM_READY;
    CHECK(step(CE_MT6835_FAIL) && m == CM_FAULT && r == CR_MT6835,
          "READY+MT6835→FAULT");

    /* READY 忽略 ENABLE/RESET */
    m = CM_READY;
    CHECK(!step(CE_CMD_ENABLE), "READY+ENABLE 忽略");
    CHECK(!step(CE_CMD_RESET), "READY+RESET 忽略");

    /* FAULT + RESET → DISABLED/NONE */
    m = CM_FAULT; r = CR_STEP_LOSS;
    CHECK(step(CE_CMD_RESET) && m == CM_DISABLED && r == CR_NONE,
          "FAULT+RESET→DISABLED");

    /* FAULT 忽略其余 */
    m = CM_FAULT; r = CR_NONE;
    CHECK(!step(CE_CMD_ENABLE), "FAULT+ENABLE 忽略");
    CHECK(!step(CE_STEP_LOSS), "FAULT+STEP_LOSS 忽略");
    CHECK(!step(CE_AS5600_FAIL), "FAULT+AS5600 忽略");
}
```
2. 新增两个用例并注册进 `main`：
```c
/* ---- 标定 L 合法性（规格 §2：100–1000 mm，正号兼域对齐） ---- */
static void test_calib_valid(void)
{
    CHECK(!cm_calib_valid(99999), "L=99999 拒（下界−1）");
    CHECK(cm_calib_valid(100000), "L=100000 收（下界）");
    CHECK(cm_calib_valid(1000000), "L=1000000 收（上界）");
    CHECK(!cm_calib_valid(1000001), "L=1000001 拒（上界+1）");
    CHECK(!cm_calib_valid(0), "L=0 拒");
    CHECK(!cm_calib_valid(-500000), "L 负拒（域反向）");
}

/* ---- 软限位窗口钳位（规格 §4：向外钳 0，向内/界内不受限） ---- */
static void test_soft_limit(void)
{
    /* 下界 −250000、上界 +250000 的窗 */
    CHECK(cm_soft_limit_clamp(260000, -250000, 250000, 100) == 0,
          "越上界且向外 → 0");
    CHECK(cm_soft_limit_clamp(260000, -250000, 250000, -100) == -100,
          "越上界但向内 → 不钳");
    CHECK(cm_soft_limit_clamp(-260000, -250000, 250000, -100) == 0,
          "越下界且向外 → 0");
    CHECK(cm_soft_limit_clamp(-260000, -250000, 250000, 100) == 100,
          "越下界但向内 → 不钳");
    CHECK(cm_soft_limit_clamp(0, -250000, 250000, 100) == 100, "界内不钳");
    CHECK(cm_soft_limit_clamp(250000, -250000, 250000, 100) == 100,
          "恰在界上不钳");
    CHECK(cm_soft_limit_clamp(-250000, -250000, 250000, 0) == 0,
          "零目标不动");
    /* 运行中收紧余量（窗口收窄到 ±240000）立即生效 */
    CHECK(cm_soft_limit_clamp(250000, -240000, 240000, 100) == 0,
          "收紧后立即钳");
}
```
（原 `test_soft_limit` 删除，替换为上文窗口版。）
3. `test_params_ranges`：删 `home_vel/home_timeout_ms/x_limit_um` 的默认值断言与边界块；新增：
```c
    CHECK(p->x_margin_um == 10000, "X_MARGIN_UM 默认 10000");
    CHECK(fw_params_set("X_MARGIN_UM", 1000)
          && fw_params_set("X_MARGIN_UM", 50000), "X_MARGIN_UM 界内");
    CHECK(!fw_params_set("X_MARGIN_UM", 999)
          && !fw_params_set("X_MARGIN_UM", 50001), "X_MARGIN_UM 界外拒绝");
```
4. `test_registry_enum`：`fw_params_count() == 9`；名字表改为九项顺序 `{"VEL_MAX","ACC_MAX","WDT_MS","WDT_DEC","X_MARGIN_UM","POLE_ZERO_URAD","POLE_INV","X_INV","STEP_LOSS_UM"}`；越界下标同步改 8/9；循环上界 9。
5. `host_tests/cartproto_parse_test.c`：合法集的 `CHECK(p("HOME") && req.cmd == CART_CMD_HOME, "HOME");` 移到非法集（`CHECK(!p("HOME"), "HOME 已退役");`）。

Run: 两条宿主测试命令。
Expected: 编译失败——`CM_HOMING`/`CE_CMD_HOME`/`cm_calib_valid` 等未定义（红）。

- [ ] **Step 2: 纯核心实现**

`cartfsm_core.h`：
- `cm_mode_t`：删 `CM_HOMING,`。
- `cm_reason_t`：删 `CR_HOME_TIMEOUT,`。
- `cm_event_t`：删 `CE_CMD_HOME`/`CE_HOME_DONE`/`CE_HOME_TIMEOUT`，改为：
```c
typedef enum {
    CE_CMD_ENABLE = 1,    /* DISABLED 才生效（标定完成后） */
    CE_CMD_RESET,         /* FAULT 清除 */
    CE_AS5600_FAIL,
    CE_MT6835_FAIL,
    CE_STEP_LOSS,         /* READY 丢步 */
} cm_event_t;
```
- 删 `cm_homing_in_t`/`cm_homing_event`/`cm_speed_reached` 声明与 §注释块；`cm_soft_limit_clamp` 声明与 doc 改为窗口版；新增 `bool cm_calib_valid(long l_um);`；文件头注释「回零完成信号是位置偏差判据…」段改为「坐标由手动两点标定建立（calib-spec.md）：MARK LEFT/MARK RIGHT 完成后中点为 x=0；ENABLE 是 DISABLED→READY 的唯一路径。」

`cartfsm_core.c`：`cm_step` 重写为三态：
```c
bool cm_step(cm_mode_t *mode, cm_reason_t *reason, cm_event_t ev)
{
    if (mode == NULL || reason == NULL) {
        return false;
    }

    switch (*mode) {
    case CM_DISABLED:
        if (ev == CE_CMD_ENABLE) {
            *mode = CM_READY;
            *reason = CR_NONE;
            return true;
        }
        return false;

    case CM_READY:
        switch (ev) {
        case CE_STEP_LOSS:
            *mode = CM_FAULT;
            *reason = CR_STEP_LOSS;
            return true;
        case CE_AS5600_FAIL:
            *mode = CM_FAULT;
            *reason = CR_AS5600;
            return true;
        case CE_MT6835_FAIL:
            *mode = CM_FAULT;
            *reason = CR_MT6835;
            return true;
        default:
            return false;           /* ENABLE/RESET 忽略 */
        }

    case CM_FAULT:
        if (ev == CE_CMD_RESET) {
            *mode = CM_DISABLED;
            *reason = CR_NONE;
            return true;
        }
        return false;

    default:
        return false;
    }
}
```
`cm_soft_limit_clamp` 替换为窗口版；`cm_homing_event`/`cm_speed_reached` 删除；文件末尾新增：
```c
bool cm_calib_valid(long l_um)
{
    return l_um >= 100000L && l_um <= 1000000L;
}
```

`cartproto.h`：`CART_CMD_HOME,` 行删除。`cartproto_parse.c`：无参段 `HOME` 判定块删除。

Run: 两条宿主测试命令。
Expected: PASS（core 断言数按新向量变化，proto 恢复 58 级，退出码 0）。

- [ ] **Step 3: 参数注册表**

`params.h`：结构体删 `home_vel, home_timeout_ms, x_limit_um`，增 `x_margin_um`（放 WDT_DEC 之后一行 `long x_margin_um;`，注释「软限位两端余量 µm」）；文件头「十二项」改「九项」的注释若存在则同步。
`params.c`：默认值删 `.home_vel/.home_timeout_ms/.x_limit_um` 三行，增 `.x_margin_um = 10000,   /* 1000 – 50000 µm        */`；`s_entries[]` 删 HOME_VEL/HOME_TIMEOUT/X_LIMIT 三行，按九项顺序增 `{ "X_MARGIN_UM",    &s_params.x_margin_um,     1000,    50000   },`（置于 WDT_DEC 之后）。

- [ ] **Step 4: cartfsm.c 外壳**

按序编辑（行号为当前 HEAD 锚点，以内容匹配为准）：

4a. 静态量区：删「HOMING 到位判定状态」四行（`s_last_e_um/s_v_meas_mm_s/s_speed_latch/s_speed_run`）与 `s_home_start`；新增：
```c
/* ---- 手动两点标定状态（calib-spec §1/§2，RAM，断电回 CAL_NONE） ---- */
typedef enum { CAL_NONE, CAL_LEFT, CAL_DONE } calib_state_t;
static calib_state_t s_calib = CAL_NONE;
static long s_e_left_um = 0;   /* 左标 E（µm） */
static long s_calib_l_um = 0;  /* 标定行程 L（CAL_DONE 后有效） */
```
flowtest 注入区删 `s_test_follow_cmd/s_test_speed_latch` 两行。
4b. `HOME_ORIGIN_MM` 宏删除。
4c. 新增窗口辅助（`params_apply_one` 前）：
```c
/* 软限位窗口（calib-spec §4）：[-L/2+m, +L/2-m] */
static void soft_window(const fw_params_t *p, long *min_um, long *max_um)
{
    *min_um = -s_calib_l_um / 2 + p->x_margin_um;
    *max_um = s_calib_l_um / 2 - p->x_margin_um;
}
```
4d. `params_apply_one` 的 `VEL_MAX` 分支内 `cm_soft_limit_clamp((long)as5600_position_um(), p->x_limit_um, v)` 改为：
```c
            long xmin, xmax;
            soft_window(p, &xmin, &xmax);
            long eff = cm_soft_limit_clamp((long)as5600_position_um(),
                                           xmin, xmax, v);
```
4e. `exec_cmd` 的 `CART_CMD_HOME` case 整体替换为三个新 case：
```c
    case CART_CMD_MARK_LEFT: {
        if (s_mode != CM_DISABLED) {
            cartproto_send_err("MARK_LEFT");   /* 规格 §2 回执原则 */
            return;
        }
        long e = (long)as5600_position_um();
#ifdef CONFIG_CARTFSM_FLOW_TEST
        e += s_test_loss_bias_um;
        s_test_loss_bias_um = 0;   /* 一次性注入：标记读数消费即清 */
#endif
        s_e_left_um = e;           /* 重复标记即覆盖重开（规格 §2） */
        s_calib = CAL_LEFT;
        cartproto_send_ok("MARK_LEFT", e);
        break;
    }

    case CART_CMD_MARK_RIGHT: {
        if (s_mode != CM_DISABLED) {
            cartproto_send_err("MARK_RIGHT");
            return;
        }
        long e = (long)as5600_position_um();
#ifdef CONFIG_CARTFSM_FLOW_TEST
        e += s_test_loss_bias_um;
        s_test_loss_bias_um = 0;
#endif
        long l = e - s_e_left_um;
        if (s_calib != CAL_LEFT || !cm_calib_valid(l)) {
            cartproto_send_err("MARK_RIGHT");  /* 状态保持（规格 §2） */
            return;
        }
        s_calib_l_um = l;
        s_calib = CAL_DONE;
        /* E/C 同帧重定基：两标中点为 x=0（引擎停止时整定） */
        long mid_um = s_e_left_um + l / 2;
        as5600_set_origin(mid_um / 1000.0);
        step_engine_set_position(um_to_cmd_steps(mid_um));
        cartproto_send_ok("MARK_RIGHT", l);
        break;
    }

    case CART_CMD_ENABLE: {
        if (s_mode != CM_DISABLED) {
            return;   /* READY/FAULT 忽略（规格 §5 矩阵） */
        }
        if (s_calib != CAL_DONE) {
            cartproto_send_err("ENABLE");
            return;
        }
        cm_mode_t m = s_mode;
        cm_reason_t r = CR_NONE;
        if (!cm_step(&m, &r, CE_CMD_ENABLE)) {
            return;
        }
        /* C 重锚到 E：先于模式发布——监督任务见 READY 即按拍判定，
         * 携旧 C 会在首拍 STEP_LOSS 误报（标定后手挪过的窗口） */
        step_engine_set_position(um_to_cmd_steps((long)as5600_position_um()));
        bsp_tmc_en_set(true);
        portENTER_CRITICAL(&s_lock);
        s_mode = CM_READY;
        s_reason = CR_NONE;
        s_last_cmd_v = 0;
        portEXIT_CRITICAL(&s_lock);
        push_mode(CM_READY, CR_NONE);
        break;
    }
```
4f. `CART_CMD_VEL` 分支的 clamp 同 4d 改窗口版。
4g. `fsm_task`：整段删除 `if (mode == CM_HOMING) { … }` 分支（续喂/latch/`cm_homing_in_t`/完成重定基/超时诊断，自 `if (mode == CM_HOMING) {` 起至 `} else if (mode == CM_READY) {` 前）；READY 分支内 clamp 改：
```c
                    /* 软限位持续钳制（运行中收紧立即生效） */
                    long xmin, xmax;
                    soft_window(p, &xmin, &xmax);
                    long eff = cm_soft_limit_clamp(
                        (long)as5600_position_um(), xmin, xmax,
                        s_last_cmd_v);
```
4h. 测试钩子区：删 `cartfsm_test_follow_cmd/cartfsm_test_speed_latch` 两函数，新增：
```c
void cartfsm_test_reset_calib(void)
{
    s_calib = CAL_NONE;   /* 模拟上电：RAM 标定丢失 */
    s_e_left_um = 0;
    s_calib_l_um = 0;
}
```
公开 API 区新增：
```c
calib_state_t cartfsm_calib(void)
{
    return s_calib;
}
```
`cartfsm.h`：删 `cartfsm_test_follow_cmd/cartfsm_test_speed_latch` 声明；flowtest 钩子区增 `void cartfsm_test_reset_calib(void);`；`calib_state_t` 的 typedef 移到 `cartfsm.h`（外壳与公开 API 共用，`cartfsm.c` 里不再重复定义），声明 `calib_state_t cartfsm_calib(void);`；文件头注释「回零到位判定（位置偏差+达速锁存）/超时」改「手动两点标定状态与使能」。

- [ ] **Step 5: selftest 与 flowtest 重写**

`cartfsm_self_test.c`：`test_transitions_spot` 改三态向量：
```c
static bool test_transitions_spot(void)
{
    cm_mode_t m = CM_DISABLED;
    cm_reason_t r = CR_NONE;
    CHECK(cm_step(&m, &r, CE_CMD_ENABLE) && m == CM_READY && r == CR_NONE,
          "ENABLE→READY");
    CHECK(cm_step(&m, &r, CE_STEP_LOSS) && m == CM_FAULT && r == CR_STEP_LOSS,
          "丢步→FAULT");
    CHECK(cm_step(&m, &r, CE_CMD_RESET) && m == CM_DISABLED, "RESET→DISABLED");
    CHECK(cm_step(&m, &r, CE_CMD_ENABLE) && m == CM_READY, "再次 ENABLE");
    return true;
}
```
`test_soft_limit_spot` 改：
```c
static bool test_soft_limit_spot(void)
{
    CHECK(cm_soft_limit_clamp(260000, -250000, 250000, 100) == 0, "向外钳 0");
    CHECK(cm_soft_limit_clamp(-260000, -250000, 250000, 100) == 100,
          "向内放行");
    return true;
}
```
新增 `test_calib_spot`（RUN_TEST 注册）：
```c
static bool test_calib_spot(void)
{
    CHECK(!cm_calib_valid(99999) && cm_calib_valid(100000), "L 下界");
    CHECK(cm_calib_valid(1000000) && !cm_calib_valid(1000001), "L 上界");
    CHECK(!cm_calib_valid(-1), "L 负拒");
    return true;
}
```

`cartfsm_flow_test.c` 整体重写（骨架 `CHECK/en_is_off/wait_mode/wait_fault_settled/to_disabled/RUN_TEST` 保留原样，`to_disabled` 的 HOMING 分支删除——三态下 READY 经注入失联制造 FAULT 后 RESET）：
```c
/* 1. 标定正路径：两 MARK + ENABLE → READY，重定基后无误报 */
static bool test_calib_and_enable(void)
{
    to_disabled();
    cartfsm_test_reset_calib();
    cartfsm_test_inject_loss_bias(0);

    cart_req_t ml = { .cmd = CART_CMD_MARK_LEFT };
    cartfsm_test_exec(&ml);                       /* E_left = E + 0 */
    CHECK(cartfsm_calib() == CAL_LEFT, "MARK LEFT 后 CAL_LEFT");

    cartfsm_test_inject_loss_bias(500000);        /* 右标在 +500 mm */
    cart_req_t mr = { .cmd = CART_CMD_MARK_RIGHT };
    cartfsm_test_exec(&mr);
    CHECK(cartfsm_calib() == CAL_DONE, "MARK RIGHT 后 CAL_DONE");

    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    CHECK(wait_mode(CM_READY, 200), "ENABLE 后 READY");
    vTaskDelay(pdMS_TO_TICKS(200));
    CHECK(cartfsm_mode() == CM_READY, "重定基后 200 ms 无 STEP_LOSS 误报");
    to_disabled();
    return true;
}

/* 1b. 无左标 / L 非法：拒且状态保持 */
static bool test_calib_invalid(void)
{
    to_disabled();
    cartfsm_test_reset_calib();

    cart_req_t mr = { .cmd = CART_CMD_MARK_RIGHT };
    cartfsm_test_exec(&mr);
    CHECK(cartfsm_calib() == CAL_NONE, "无左标 MARK RIGHT 拒");

    cart_req_t ml = { .cmd = CART_CMD_MARK_LEFT };
    cartfsm_test_exec(&ml);
    cartfsm_test_inject_loss_bias(0);             /* 两点重合 → L=0 */
    cartfsm_test_exec(&mr);
    CHECK(cartfsm_calib() == CAL_LEFT, "L 非法保持 CAL_LEFT");

    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    CHECK(cartfsm_mode() == CM_DISABLED, "未完成标定 ENABLE 拒");
    to_disabled();
    return true;
}

/* 1c. 断电语义：标定丢失后 ENABLE 拒 */
static bool test_enable_after_reset_calib(void)
{
    to_disabled();
    cartfsm_test_reset_calib();
    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    CHECK(cartfsm_mode() == CM_DISABLED && cartfsm_calib() == CAL_NONE,
          "上电未标定 ENABLE 拒");
    return true;
}

/* 2. 重新标定：CAL_DONE 后 MARK LEFT 覆盖重开 */
static bool test_recalib(void)
{
    to_disabled();
    cartfsm_test_reset_calib();

    cart_req_t ml = { .cmd = CART_CMD_MARK_LEFT };
    cartfsm_test_exec(&ml);
    cartfsm_test_inject_loss_bias(400000);
    cart_req_t mr = { .cmd = CART_CMD_MARK_RIGHT };
    cartfsm_test_exec(&mr);
    CHECK(cartfsm_calib() == CAL_DONE, "第一段标定完成");

    cartfsm_test_exec(&ml);                       /* 覆盖重开 */
    CHECK(cartfsm_calib() == CAL_LEFT, "重复 MARK LEFT 重开");
    cartfsm_test_inject_loss_bias(500000);
    cartfsm_test_exec(&mr);
    CHECK(cartfsm_calib() == CAL_DONE, "第二段标定完成");

    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    CHECK(wait_mode(CM_READY, 200), "新标定后 ENABLE");
    to_disabled();
    return true;
}

/* 3. READY 注入 |E−C| 偏差 → FAULT/STEP_LOSS（既有保护） */
static bool test_ready_step_loss(void)
{
    to_disabled();
    cartfsm_test_reset_calib();
    cart_req_t ml = { .cmd = CART_CMD_MARK_LEFT };
    cartfsm_test_exec(&ml);
    cartfsm_test_inject_loss_bias(500000);
    cart_req_t mr = { .cmd = CART_CMD_MARK_RIGHT };
    cartfsm_test_exec(&mr);
    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    CHECK(wait_mode(CM_READY, 200), "进入 READY");

    cartfsm_test_inject_loss_bias(10000);   /* 一次性：> STEP_LOSS_UM */
    CHECK(wait_mode(CM_FAULT, 500), "偏差超阈值应 FAULT");
    CHECK(cartfsm_reason() == CR_STEP_LOSS, "原因为 STEP_LOSS");
    CHECK(wait_fault_settled(2000), "EN 关");
    to_disabled();
    return true;
}

/* 4. READY 注入 AS5600 失联 → FAULT/AS5600（复用既有口径） */
static bool test_ready_encoder_fail(void)
{
    to_disabled();
    cartfsm_test_reset_calib();
    cart_req_t ml = { .cmd = CART_CMD_MARK_LEFT };
    cartfsm_test_exec(&ml);
    cartfsm_test_inject_loss_bias(500000);
    cart_req_t mr = { .cmd = CART_CMD_MARK_RIGHT };
    cartfsm_test_exec(&mr);
    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    CHECK(wait_mode(CM_READY, 200), "进入 READY");

    TickType_t t0 = xTaskGetTickCount();
    cartfsm_test_inject_encoder_fail(1);
    CHECK(wait_mode(CM_FAULT, 100), "失联应 FAULT");
    CHECK(cartfsm_reason() == CR_AS5600, "原因为 AS5600");
    cartfsm_test_inject_encoder_fail(0);
    CHECK(wait_fault_settled(2000), "EN 关");
    ESP_LOGI(TAG, "  失联→FAULT+EN 关 耗时 %u ms（实测，含调度）",
             (unsigned)(xTaskGetTickCount() - t0));
    to_disabled();
    return true;
}

/* 5. FAULT → RESET 回 DISABLED 后标定仍在，直接 ENABLE 恢复 */
static bool test_fault_reset_recovery(void)
{
    to_disabled();
    cartfsm_test_reset_calib();
    cart_req_t ml = { .cmd = CART_CMD_MARK_LEFT };
    cartfsm_test_exec(&ml);
    cartfsm_test_inject_loss_bias(500000);
    cart_req_t mr = { .cmd = CART_CMD_MARK_RIGHT };
    cartfsm_test_exec(&mr);
    cart_req_t en = { .cmd = CART_CMD_ENABLE };
    cartfsm_test_exec(&en);
    CHECK(wait_mode(CM_READY, 200), "进入 READY");
    cartfsm_test_inject_loss_bias(10000);
    CHECK(wait_mode(CM_FAULT, 500), "制造 FAULT");
    cartfsm_test_inject_loss_bias(0);

    cart_req_t rs = { .cmd = CART_CMD_RESET };
    cartfsm_test_exec(&rs);
    CHECK(cartfsm_mode() == CM_DISABLED, "RESET 回 DISABLED");
    CHECK(cartfsm_calib() == CAL_DONE, "标定保留");
    cartfsm_test_exec(&en);
    CHECK(wait_mode(CM_READY, 200), "直接 ENABLE 恢复");
    to_disabled();
    return true;
}
```
RUN 列表与文件头注释同步（六用例）。`to_disabled` 改写：
```c
static void to_disabled(void)
{
    if (cartfsm_mode() == CM_READY) {
        cartfsm_test_inject_encoder_fail(1);
        if (!wait_fault_settled(2000)) {
            ESP_LOGE(TAG, "to_disabled：FAULT 停机超时");
        }
        cartfsm_test_inject_encoder_fail(0);
    } else if (cartfsm_mode() == CM_FAULT) {
        if (!wait_fault_settled(2000)) {
            ESP_LOGE(TAG, "to_disabled：FAULT 停机超时");
        }
    }
    cart_req_t req = { .cmd = CART_CMD_RESET };
    cartfsm_test_exec(&req);
    for (int t = 0; t < 100 && cartfsm_mode() != CM_DISABLED; t += 5) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
```

- [ ] **Step 6: 全部门禁**

Run: 两条宿主测试命令 + `build_gates.ps1`。
Expected: 宿主 PASS 退出码 0；三构建成功（正式构建无 FLOW_TEST 宏块）。

- [ ] **Step 7: 残留扫描与提交**

```bash
cd tmp/PD-STEP && grep -rn "CM_HOMING\|CE_CMD_HOME\|CE_HOME_DONE\|CE_HOME_TIMEOUT\|CR_HOME_TIMEOUT\|HOME_VEL\|HOME_TIMEOUT\|X_LIMIT\|x_limit_um\|home_vel\|home_timeout\|cm_homing_event\|cm_speed_reached\|HOME_ORIGIN" components/ main/ host_tests/ tools/ --include=*.c --include=*.h
```
Expected: 零命中（`sdkconfig` 与历史文档不在扫描面）。
```bash
git add components host_tests
git commit -m "cartfsm：手动两点标定切换（MARK/ENABLE、三态模式机、X_MARGIN_UM 软限位窗口；CM_HOMING/CE_HOME*/HOME 参数族与位置偏差回零判定全量退役；flowtest 六用例重写）"
```

---

### Task 3: 端侧 edge 同步

**Files:**
- Modify: `deploy/v2/edge/proto.h`（mode/reason 枚举）
- Modify: `deploy/v2/edge/proto.c`（词法/命名）
- Modify: `deploy/v2/edge/config.h` + `rig.c`（推送集、wait_ready 退役）
- Modify: `deploy/v2/edge/console.h/.c`（calib/mark/enable 命令）
- Modify: `deploy/v2/edge/main.c`（run 流程）
- Test: `deploy/v2/edge/proto_test.c` + `console_test.c`

**Interfaces:**
- Consumes: 协议新词法 `MARK LEFT/MARK RIGHT/ENABLE`、`OK MARK_LEFT/MARK_RIGHT` 回执、四值 reason。
- Produces: console 命令 `calib`（交互引导）/ `mark-left` / `mark-right` / `enable`（一次性，等回执）。

- [ ] **Step 1: 枚举与词法**

`proto.h`：`fw_mode_t` 删 `FW_HOMING,`；`fw_reason_t` 删 `FW_REASON_HOME_TIMEOUT,`。
`proto.c`：mode 词法删 `HOMING` 分支与命名 case；reason 词法删 `HOME_TIMEOUT` 分支与命名 case。
`proto_test.c`：grep `HOMING\|HOME_TIMEOUT` 改造对应断言（MODE 行解析向量改 READY/FAULT 两态；补一条 OK 行 `OK MARK_LEFT 123` → `m.type == PROTO_OK && strcmp(m.param,"MARK_LEFT")==0 && m.value==123`，钉 Review Focus 第 5 条）。

- [ ] **Step 2: 推送集与 rig**

`config.h`：`CFG_FW_PARAM_COUNT 7` → `4`。
`rig.c`：`CFG_FW_PARAMS` 删 HOME_VEL/HOME_TIMEOUT/X_LIMIT 三行（余 VEL_MAX/ACC_MAX/WDT_MS/WDT_DEC）；`rig_wait_ready` 函数整体删除；`rig_goto_center` 的注释「回零完成后…」改「标定 ENABLE 后把小车从端点移回行程中点（协议操作流程）」。

- [ ] **Step 3: console 命令**

`console.h`：`CONSOLE_CMD_HOME` 改为 `CONSOLE_CMD_CALIB`，追加 `CONSOLE_CMD_MARK_LEFT, CONSOLE_CMD_MARK_RIGHT, CONSOLE_CMD_ENABLE`。
`console.c`：
1. 解析：`home` 词删除，`calib/mark-left/mark-right/enable` 进无参命令组（`strcmp` 链同步）。
2. 执行（替换 `CONSOLE_CMD_HOME` case）：
```c
    case CONSOLE_CMD_CALIB: {
        int known;
        fw_mode_t m = rig_mode(&known, NULL);
        if (known && m != FW_DISABLED) {
            fprintf(stderr, "[edge] calib requires DISABLED (current %s)\n",
                    proto_mode_name(m));
            return -1;
        }
        printf("[edge] push cart to LEFT end, then press Enter...");
        if (wait_enter() != 0) return -1;
        if (rig_send_simple("MARK LEFT") != 0 || rig_wait_reply(1.0) != 0) {
            fprintf(stderr, "[edge] MARK LEFT rejected/failed\n");
            return -1;
        }
        printf("[edge] push cart to RIGHT end, then press Enter...");
        if (wait_enter() != 0) return -1;
        if (rig_send_simple("MARK RIGHT") != 0 || rig_wait_reply(1.0) != 0) {
            fprintf(stderr, "[edge] MARK RIGHT rejected (L invalid? no left mark?)\n");
            return -1;
        }
        if (rig_send_simple("ENABLE") != 0) return -1;
        if (rig_wait_mode(FW_READY, 2.0) != 0) {
            fprintf(stderr, "[edge] ENABLE failed (see MODE events)\n");
            return -1;
        }
        printf("[edge] calibrated & READY; use 'center' to move to mid-track\n");
        return 0;
    }
    case CONSOLE_CMD_MARK_LEFT:
        return send_and_await("MARK LEFT", "MARK_LEFT");
    case CONSOLE_CMD_MARK_RIGHT:
        return send_and_await("MARK RIGHT", "MARK_RIGHT");
    case CONSOLE_CMD_ENABLE:
        return send_and_await("ENABLE", "ENABLE");
```
配套小助手（console.c 内 static）：
```c
static int wait_enter(void)
{
    int c = getchar();
    while (c != '\n' && c != EOF) {
        c = getchar();
    }
    return c == EOF ? -1 : 0;
}

static int send_and_await(const char *cmd, const char *reply_name)
{
    (void)reply_name;   /* rig_wait_reply 只区分 OK/ERR，名字经控制台回显核对 */
    if (rig_send_simple(cmd) != 0) {
        return -1;
    }
    if (rig_wait_reply(1.0) != 0) {
        fprintf(stderr, "[edge] %s rejected/failed\n", cmd);
        return -1;
    }
    return 0;
}
```
3. `print_help` 的 `home` 行替换为：
```c
           "  calib                 interactive 2-point calibration (MARK LEFT/RIGHT + ENABLE)\n"
           "  mark-left             send MARK LEFT (cart at left end)\n"
           "  mark-right            send MARK RIGHT (cart at right end)\n"
           "  enable                ENABLE after calibration, wait READY\n"
```
`console_test.c`：`expect_ok("home", CONSOLE_CMD_HOME)` 改 `expect_ok("calib", CONSOLE_CMD_CALIB)`；补 `expect_ok("mark-left", CONSOLE_CMD_MARK_LEFT); expect_ok("mark-right", CONSOLE_CMD_MARK_RIGHT); expect_ok("enable", CONSOLE_CMD_ENABLE);`；非法组 `expect_err("home")`（未知命令）与 `expect_err("home extra")` 删除/替换为 `expect_err("calib extra")`。

- [ ] **Step 4: main.c run 流程**

`main.c:178-193` 的 home 块替换为：
```c
    if (mode != FW_READY) {
        fprintf(stderr, "[edge] firmware in %s; calibrate first "
                "(console 'calib'), or RESET a FAULT\n",
                proto_mode_name(mode));
        return 1;
    }
    if (rig_goto_center(15.0) != 0) {
        fprintf(stderr, "[edge] move-to-center failed\n");
        return 1;
    }
    printf("[edge] ready at track center; lift the pole near upright to engage the policy\n");
```
并删除 `do_home` 变量与 `--no-home` 参数（grep `do_home\|no-home` 全部出现点同步清理）。

- [ ] **Step 5: 测试与提交**

Run: `wsl make -C deploy/v2/edge test`。
Expected: proto/policy/console 全过。残留 `grep -rn "HOMING\|HOME_TIMEOUT\|HOME_VEL\|X_LIMIT\|rig_wait_ready\|do_home" deploy/v2/edge/ --include=*.c --include=*.h` 零命中。
```bash
git add deploy/v2/edge
git commit -m "端侧：标定流程同步（calib/mark-left/mark-right/enable 命令、四值 reason、推送集四项、run 流程改为要求先标定）"
```

---

### Task 4: host 工具子命令

**Files:**
- Modify: `tmp/PD-STEP/tools/cartpole_host.py`

**Interfaces:**
- Consumes: 新线协议命令。
- Produces: `py mark-left` / `py mark-right` / `py enable` 子命令（台位操作用）；`py home` 删除。

- [ ] **Step 1: 替换 cmd_home**

`cmd_home` 整体替换为（`send_line` 无返回值，回执经 `pump` 填充的 `last_reply` 元组 `("OK", name, value)` / `("ERR", name, None)` 判定；先 grep `last_reply` 确认 `__init__` 里有初值 `None`，没有则补）：
```python
def cmd_mark(rig, args, word):
    rig.send_line(word)
    rig.pump(1.0)
    r = rig.last_reply
    if r and r[0] == "OK":
        print("%s → %d" % (r[1], r[2]))
        return 0
    print("%s 被拒（%s）" % (word, r[1] if r else "无回执"))
    return 1


def cmd_mark_left(rig, args):
    return cmd_mark(rig, args, "MARK LEFT")


def cmd_mark_right(rig, args):
    return cmd_mark(rig, args, "MARK RIGHT")


def cmd_enable(rig, args):
    rig.send_line("ENABLE")
    m = rig.wait_mode("READY", args.timeout)
    if m is None:
        print("超时未到 READY（当前 %s）" % (rig.last_mode,))
        return 1
    return 0
```
子命令注册：删 `home` 的 `add_parser` 与 dispatch 表项，增：
```python
    sub.add_parser("mark-left")
    sub.add_parser("mark-right")
    p = sub.add_parser("enable"); p.add_argument("--timeout", type=float, default=5.0)
```
dispatch 表同步 `"mark-left": cmd_mark_left, "mark-right": cmd_mark_right, "enable": cmd_enable`；文件头部用法示例行 `home` 改 `mark-left / mark-right / enable`。

- [ ] **Step 2: 语法自检与提交**

Run: `wsl python3 -m py_compile tmp/PD-STEP/tools/cartpole_host.py`（或 `python -m py_compile`，按本机 Python 可用形式）。
Expected: 无输出（编译通过）。
```bash
cd tmp/PD-STEP && git add tools/cartpole_host.py
git commit -m "host 工具：mark-left/mark-right/enable 子命令替换 home（标定台位操作）"
```

---

### Task 5: 文档重写

**Files:**
- Modify: `deploy/v2/protocol.md`（大修）
- Modify: `deploy/v2/onboard-checklist.md`（B1 预期与 C 段规程）
- Modify: `tmp/PD-STEP/README.md`（字样）

**Interfaces:**
- Consumes: Task 2/3 已固化的行为。
- Produces: 文档与实现一致。

- [ ] **Step 1: protocol.md**

1. 开头叙述与模式机表：三态（DISABLED/READY/FAULT），READY 进入方式「DISABLED 收到 `ENABLE`（两点标定完成后）」；删 HOMING 行。
2. 转换表整体替换为规格 §5 的四行表（MARK 在 DISABLED 内部完成、ENABLE→READY、READY 三故障源、FAULT→RESET），引言改标定语义并指向 `calib-spec.md`。
3. 命令清单：删 `HOME` 行，增三行 `MARK LEFT` / `MARK RIGHT` / `ENABLE`（含回执说明）。
4. 命令×模式矩阵替换为规格 §5 的矩阵（注意 CONFIG 与 CONFIG? 分行）。
5. reason 表删 `HOME_TIMEOUT` 行。
6. 参数注册表删 HOME_VEL/HOME_TIMEOUT/X_LIMIT 三行，增 `X_MARGIN_UM | 10000 | 1000 – 50000 | µm | 软限位两端余量（窗口 [−L/2+m, +L/2−m]）`；`STEP_LOSS_UM` 含义去掉回零角色；「后四项为台位标定量」段保持四项（POLE_ZERO_URAD/POLE_INV/X_INV/STEP_LOSS_UM）措辞核对。
7. 软限位段（`X_LIMIT` 描述处）改为窗口语义；「回零后小车停在 −265 mm 处于限外」句删除。
8. `x_um` 字段含义改「原点 = 两标中点，由手动两点标定建立；+x 方向见符号约定」；数据面有效性句「最后已知模式为 READY 后」改「标定完成后」。
9. 操作流程（演示当天）重写：
```
1. 上电（DISABLED，摆杆自然下垂最安全）。
2. 端侧开串口，发 `QUERY` 同步模式；可选 `CONFIG` 推送参数。
3. 手推小车到最左端 → `MARK LEFT`；推到最右端 → `MARK RIGHT`（回显 L，
   肉眼核对 ≈530 mm）。
4. `ENABLE` 等 `MODE READY NONE`。
5. 端侧以小速度 `VEL` 把小车从右端移到 0 附近。
6. 人手把摆杆扶到直立附近，策略接管。
7. FAULT 后：`RESET` 回 DISABLED（标定保留）→ `ENABLE` 恢复。
```
10. 未决事项：删「回零重复性实测」条，增「标定重复性实测（同一端点重复标记读数散布）」与「L 合法窗是否按实测行程收紧」（规格 §9）。

- [ ] **Step 2: onboard-checklist.md**

1. B1 预期列改：`SELF-TEST DONE: 6/6 PASS`（标定正路径/无左标与 L 非法/上电未标定拒/重开标定/READY 偏差/失联+恢复）；记录列不动。
2. C 段表整体重写：
```
| C1 | 标定正路径 | 推最左 `py mark-left` → 推最右 `py mark-right` → `py enable` | 两 `OK` 回显（L ≈ 530 mm）；`MODE READY NONE`；`py state` 静止 x_um ≈ +L/2，回中后 ≈ 0 |
| C2 | 标定负路径 | 先右后左再标、两点重合、未标定即 `py enable` | 均回 `ERR`，状态不变 |
| C3 | 域对齐 | `py vel 50 --hold 1` 记实际方向 = +x | x_um 增大（L 为正已保证 E 域；此处核指令域，反了改 X_INV） |
| C4 | 符号约定 | 同向手掰摆顶看 theta | theta 同向增大；`VEL −50` 反向 |
| C5 | 摆角零位 | 手扶摆杆机械直立读 STATE theta | 读数 θ₀；标定值 = −θ₀ |
| C6 | 标定落地 | `py config POLE_ZERO_URAD <−θ₀>`（及 POLE_INV/X_INV）后复核 | 直立 theta≈0；VEL 50 方向与 theta 符号符合约定 |
| C7 | 软限位 | 右端 `py vel 200` / `py vel -200`；左端对称 | 向外钳制不动、向内可动 |
| C8 | 标定重复性 | 同一端点重复标记 3 次读回显 | 散布 ≤ 100 µm |
| C9 | 看门狗 | `py wdt-test` | ~100 ms 减速到 0；QUERY 回 READY NONE；恢复发送继续跟踪 |
| C10 | 拔线/长稳 | READY 拔 AS5600（再拔 MT6835）；`py state --secs 600` | ≤15 ms FAULT 对应 reason、EN 关、STATE 流不断；200 Hz 无 reboot |
```
（原 C3b/C8/C9/C10 内容并入上表对应行；编号重排。）
3. 阶段 D 段：`POLE_ZERO_URAD / POLE_INV / X_INV`（及实测微调的 `STEP_LOSS_UM`、`X_MARGIN_UM` 如需）。
4. 未决出口：②③改写为「已随方案更换关闭（自动回零废弃）」；新增标定重复性出口指向 C8。
5. 头部「协议经两轮…堵转在 HOMING…」等历史叙述如有残留字样（grep `堵转\|回零\|HOMING\|SGTHRS`）逐处改写为标定语义。

- [ ] **Step 3: 固件 README 与提交**

`tmp/PD-STEP/README.md`：`位置偏差判据回零` → `手动两点标定`；硬件表/结构说明里若再 grep 到 `回零/STALL/SGTHRS` 字样同步清理。
```bash
# cartpole 仓库
git add deploy/v2/protocol.md deploy/v2/onboard-checklist.md deploy/v2/calib-spec.md
git commit -m "协议/清单：手动两点标定全面落地（三态模式机、MARK/ENABLE 命令集、X_MARGIN_UM、标定规程）"
# 固件仓库
cd tmp/PD-STEP && git add README.md
git commit -m "README：手动两点标定字样同步"
```

---

### Task 6: 上板验收（用户台位执行）

**Files:**
- Modify（回填记录）: `deploy/v2/onboard-checklist.md`

- [ ] **Step 1: flowtest 上板**（PowerShell）

```powershell
idf.py -B build-flowtest -DSDKCONFIG=build-flowtest/sdkconfig -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;tools/flowtest.defaults" flash monitor
```
Expected: `SELF-TEST DONE: 6/6 PASS`（六用例对应 Task 2 Step 5）。

- [ ] **Step 2: 正式固件标定验收**

按清单新 C 段逐项执行（C1–C10）；重点判读：C1 的 L 回显与理论行程差、C2 三条负路径的 ERR 行、C7 双向钳制、C8 重复标记散布。**前置**：电机相线排查完成（原地抖动未解则 C1 的 `py enable` 后 `py vel` 无法验收——标定本身不受影响）。

- [ ] **Step 3: 记录回填**

C1–C10 记录回填清单并提交「清单：两点标定验收记录回填」。

---

## 自审记录（写计划时已核对）

- 规格覆盖：§1 信号→Task 2；§2 决策表→Task 2（exec MARK 两 case + `cm_calib_valid` 宿主向量 + flowtest 1b）；§3 ENABLE→Task 2 Step 4e；§4 READY→Task 2 Step 4g + 宿主窗口向量；§5 协议修订→Task 1/2（词法与退役）/Task 3（端侧）/Task 5（文本）；§6 实现范围→Task 2/3/5 与之一致；§7 测试验收→Task 2（宿主+flowtest）/Task 6（上板）；§8 局限为记录性；§9 未决→Task 5 Step 1 第 10 条 + Task 6 回填。
- 占位符：无 TBD；文档重写类步骤均给出目标文本或逐条替换规则。
- 类型一致性：`calib_state_t` 定义于 cartfsm.h（Task 2 4h）；`cm_soft_limit_clamp` 四参签名在宿主向量/selftest/外壳三处一致；`cartfsm_test_reset_calib/cartfsm_calib` 声明与实现一致；`send_and_await/wait_enter` 为 console.c 内 static，声明与定义同文件。
- Review Focus 五条分别钉在 Task 2（1/2/3/4）与 Task 3（5）。
- 风险标注：Task 2 Step 4e 的 `as5600_set_origin(mid_um / 1000.0)` 依赖其 mm 浮点参数契约（与原 HOME_ORIGIN_MM 用法同构）；`-Wswitch` 对 `exec_cmd` 未覆盖新枚举的处理已给出预案（加 default）。
