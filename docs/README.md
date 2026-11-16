# 文档导航与维护边界

文档按使用目的组织；操作指南描述当前主线，历史证据单独归档。根 README 是项目入口，源码、配置和固件仓库各自保持权威，本文不复制参数表。

## 训练与模型

1. [Isaac Lab 训练工作流](training/isaaclab-workflow.md)：环境安装、训练、筛选衔接、回放与导出。
2. [仿真模型与执行器契约](training/sim2real-model.md)：物理来源、动作映射、观测、随机化、时间尺度和未建模边界。
3. [400 Hz / Acc9 配方](training/400hz-acc9.md)：可选训练任务及部署匹配。
4. [回合指标语义](training/episode-metrics.md)：成功率的分母、窗口与完整回合。
5. [摆杆几何与验证](training/pole-model.md)：几何约束、诊断命令和已有证据，区分静态检查、后端读回及现场确认。
6. [候选产生与筛选](../scripts/evaluation/README.md)：训练时提名、筛选模式、独立 holdout 及报告。

## 部署与诊断

- [当前固件协议入口](deployment/protocol.md)：cartpole 消费者接口和外部固件权威文档。
- [Python 单动作运行器](../deploy/v2/device/README.md)：合同、会话、400/500 Hz、停车及可选事件处理。
- [C 适配接口](../deploy/v2/device/edge/README.md)：接收与会话层的实现范围，不等于完整 NPU 控制器。
- [速度与节拍诊断](../scripts/diagnostics/README.md)：离线和仿真诊断工具。

## 参考实现与历史证据

[旧 V2 固件资料](archive/legacy-v2/README.md)保留旧标定规格和实测验收，不混入当前操作指南。旧 V1、V2 C/PC 参考代码的局部说明仍在对应实现旁；双动作等实验不由主线文档宣称完成。

## 维护规则

- 同一事实只保留一个主要说明位置；工具参数留在工具旁 README，跨流程指南通过链接引用。
- 完成的实施计划不长期冒充待办；独有设计约束并入正式说明，独有实测证据归档，其余由 Git 历史追溯。
- 文档移动、合并或删除时同步修改 Markdown 链接及源码中的文档引用。
- 不把配置值当作测量值，不把单元测试或现场外观确认扩大为未执行的物理/硬件验证。
