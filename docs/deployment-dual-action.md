# 双动作真机适配（实验分支）

旧四观测/单动作模型仍用原合同、VEL。双动作明确使用 model-contract-dual.example.json；不可只把旧合同改成双动作复用旧模型。
新输出严格为 float32 [1,2]，观测仍 [x,theta,xdot,thetadot]。模型需在400Hz、16ms观测速度窗口、执行器tau0、加速度能力9m/s²的双动作任务重新训练。

复制现场 profile，设置 training.control_hz=400、configured.acceleration_mm_s2=9000，保留当前标定行程及其余真实配置。不要修改训练基线profile或仅改合同假装改变时钟。
固件必须已烧录 VELA/1；主机启动会查询能力并核对 ACC_MAX=9000。主机不自动配置、使能或烧录。
当前标定坐标、模型主文件及外置权重哈希需填入新合同；模板默认未签认，不能直接运行。

```powershell
& $python .\deploy\v2\device\run_policy.py --port COM11 --onnx $onnx --contract $contract --profile $sessionProfile --output $report --continuous --start-confirmed
```

先使用 --diagnostic-timing --seconds 5 替代 --continuous --start-confirmed，可在DISABLED运行同路径只读诊断；这不验证执行器、线缆负载和闭环稳定性。
报告为新文件，不复用路径。Ctrl+C沿既有停机流程处理；VEL 0恢复固定ACC_MAX，停稳后DISABLE，不用模型的小加速度停车。
不恢复已按现场要求关闭的上位机位置/摆角门限；保留元数据、起始速度、有效性、FAULT、新鲜度和通信期限检查。
400Hz使用实际2.5ms绝对栅格，旧500Hz仍为2ms。4/6ms通信预算、4ms观测新鲜度、2ms推理预算保持不变，不因降低频率放宽故障保护。

本分支仅基础离线核验；尚无双动作真机验收。不能以只读诊断或模拟串口成功替代现场证据。
