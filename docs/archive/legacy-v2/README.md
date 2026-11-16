# 旧 V2 固件历史资料

这里保留旧 PD-STEP 台位的不可替代规格与实测证据，不是当前固件操作入口。当前固件的接口与职责见[部署协议](../../deployment/protocol.md)。旧记录中的 12 V、AS5600、200 Hz、RAM 标定及历史参数只适用于其当时版本，不得覆盖现有设备档案。

## 保留的材料

- [旧两点标定规格](calib-spec.md)：解释仓库仍保留的旧 C/PC 参考控制器的标定假设、模式与回执语义；不维护为当前固件规范。
- [旧台位验收记录](onboard-checklist.md)：保留 2026-09-26 实测行程、方向、时序和故障验证记录及未测项目。记录中的步骤只用于理解历史证据，不能作为现设备的执行清单。

## 已退出工作树的实施计划

以下计划属于旧固件逐步迁移任务，包含已经过时的实现命令、代码片段和未勾选任务。它们不再作为待办或当前规范维护；完整内容可从固定版本恢复：

- [旧固件改造实施计划](https://github.com/CLD1994/cartpole/blob/3648061c7eb65ab7e4e1505ed7e67ff6d898ca00/deploy/v2/firmware-plan.md)
- [旧两点标定实施计划](https://github.com/CLD1994/cartpole/blob/3648061c7eb65ab7e4e1505ed7e67ff6d898ca00/deploy/v2/calib-plan.md)

旧 V1 `S`/`P` 协议仍随其参考实现保存在 [deploy/protocol.md](../../../deploy/protocol.md)。保留历史代码不表示其已适配当前装置。
