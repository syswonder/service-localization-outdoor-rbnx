# Outdoor localization service for Robonix

独立定位 service：从 G70 GNSS 与 WIT IMU 原语的 ROS 能力读取测量，输出导航所需的 Odometry、map 位姿、TF 和就绪状态。

```text
primitive-g70-gnss-rbnx  ── fix / RTK status / receiver velocity ──┐
                                                               ├─ 本 service ── odom / pose / TF / status ── navigation service
primitive-wit-imu-rbnx ── Imu / gyro / status ────────────────────┘
```

运行不依赖旧 outdoorNav 工程，不打开串口、不处理 NMEA/UBX/WIT 字节、不连接 NTRIP，不订阅山猫 `/ODOM`，不订阅控制指令。旧 `legacy` 模式不积分加速度；新 `eskf` 模式明确输出惯性估计速度，不将其标为接收机实测速度。Scene 通过导航服务读取融合状态和计时，定位服务只提供数据。

## 配套版本状态

本仓需要配套的 G70 速度／GST 扩展、WIT 独立 gyro 扩展，以及导航服务的 localization 契约接入修改。

- 导航服务对接已发布到 main：`18c14136774eaf091080662eae0bcbf7728a2f33`。
- G70 扩展已发布到 main：`1bc5c5d38e8cb2b55c2fc95bee87b860550e3de9`。
- WIT 扩展已发布到 main：`2df8f8537a17ac86366009b74076c19c9471707c`。

当前验证报告对应上述配套版本；上述配套扩展现已发布；真实部署仍须配置实际设备、安装与精度参数，不能把虚拟验证视为实机验收。
原始依赖 commit 与验证源码指纹见 `docs/validation/source-manifest.json`。

## 输入契约

从 `provider_ids.gnss` 同一实例解析：

- `robonix/primitive/gnss/fix`：NavSatFix，天线 WGS84 位置与接收机报告的协方差。
- `robonix/primitive/gnss/rtk_status`：RTK 质量、GGA/RMC 时刻、数据龄期和关联消息时间戳。
- `robonix/primitive/gnss/velocity`：TwistStamped，ENU 水平地速，`gnss_enu` frame。

从 `provider_ids.imu` 同一实例解析：

- `robonix/primitive/imu/imu`：设备解算姿态，尚未对齐地图。
- `robonix/primitive/imu/angular_velocity`：Vector3Stamped，传感器坐标系中独立新鲜的 gyro（rad/s）。
- `robonix/primitive/imu/status`：串口、主 IMU 有效性与龄期状态。

可用 `topic_remap` 显式绑定同样的角色，但正式部署应使用 Atlas；缺少原语能力时 INIT 返回 Deferred，不回退读设备或旧仓库。

## 原语扩展与数据真实性

G70 原语需要本轮扩展：解析接收机已有的 RMC 输出水平地速，按 GST 的同一测量时刻填位置标准差。新 capability 在原语包内定义，不修改框架全局能力目录。

- 不声称 RMC 一定来自多普勒；这里叫“接收机报告地速”。
- 不将 HDOP 随意换成米；没有 GST/等价协方差元数据时保持未知，本 service 不误报就绪。
- RMC 不含速度精度。`velocity_accuracy_mps` 只能来自独立验证的部署侧界限，默认缺失且 `velocity_accuracy_confirmed=false`；不能当作接收机报告的 sAcc。
- 当前 UBX NAV-PVT 接收机的速度不确定度来自本机 `sAcc`，不是 NTRIP 基站返回的速度。该数值用于状态显示和融合权重，不设就绪/停车上限。旧配置键 `gnss_speed_accuracy_mps` 仅兼容读取，已无拦截作用；缺失、非有限或非正的不确定度仍视为无效数据。
- 标准 Imu 已带角速度；新增独立 gyro 是为了姿态帧断流时，仍能用实际角速度判断停车。旧 Imu 的发布/单位语义不变。
- 不自动发送设备配置命令。实际硬件是否已输出 RMC/GST、是否兼容安装与输出量程，需要后续单独验证。

## 输出契约

- `robonix/service/localization/odom`：nav_msgs/Odometry，连续局部位姿和实测速度反馈，frame `odom`、child `base_link`。
- `robonix/service/localization/pose`：PoseWithCovarianceStamped，地图中的 base_link 位姿。
- `robonix/service/localization/status`：String JSON，就绪、RTK 等级、航向/速度有效性、故障与来源。
- 独占动态 TF：`map→odom` 和 `odom→base_link`。TF 话题由部署配置绑定，不与其他定位器重复发布。

地图校正由接受的 GNSS 位置与已修正 IMU 航向提供；旧模式局部里程积分接收机地速和 gyro；新模式见下文。停用时就绪状态立即撤回；测量仍可用于停车反馈。TF 重复所有者与异常跳变会停止全局就绪。

## 配置与部署

`origin` 必须是部署的固定 WGS84 datum，不能每次启动用第一帧位置自动替换。
`antenna_xy_m` 为 base_link 到天线杆臂；`imu.base_from_imu_rotation` 为传感器向机身的安装旋转。地理北向修正、安装、姿态质量和速度界限必须显式确认，默认不假定已标定。

原语只管设备，定位服务只管计算，导航服务只管参考路线执行与避障。完整示例见 `config/deployment.example.yaml`；不能将示例的未确认项直接改成 true 来跳过测量。

```bash
source /opt/ros/humble/setup.bash
rbnx validate .
bash scripts/build.sh
# 在整机部署目录使用对应 manifest：
rbnx build -f robonix_manifest.yaml
rbnx boot -f robonix_manifest.yaml
```

可用 `LOCALIZATION_PYTHON` 指定 ROS-compatible Python。依赖 Robonix API/codegen、NumPy、ROS 2 Humble，不生成自定义 ROS IDL overlay。start 只起 bridge，Driver INIT 解析 provider 并启动计算 worker；activate 允许就绪，不等于所有输入已满足质量要求。

## 验证

`tests/test_inputs.py` 使用 ROS-shaped 数据验证转换、时间一致性、缺失精度拒绝、未知北向拒绝、姿态断流时继续保留 gyro 停车反馈。
`tests/full_stack.py` 在独立 Docker 中运行真实 `rbnx build/boot` 和 Atlas，两个真实原语驱动从伪串口读合成字节，再由本服务与真实 Nav2 完成虚拟导航。不挂载或导入旧 outdoorNav，不修改现有网页测试服务。

完整独立运行验证与未验证硬件项见 [VALIDATION.md](docs/VALIDATION.md)。本地兼容 manifest 适配所验证 SDK 的命名空间 Driver；部署只选择一种生命周期入口。

## Loaded outdoor map reference

Deploy with `require_map_reference: true` and an unbound placeholder origin. The worker emits status but no odometry or TF until Scene supplies `map_id`, `map_version` and the saved map origin through Driver INIT, then ACTIVATE. Changed configuration restarts the worker. Loading a Beijing map therefore uses its own origin; no Hangzhou bundle is required. Navigation checks the live reference before accepting a route.

## RTK＋惯导融合

部署设 `estimator_mode: eskf`，IMU 原语设 `device_profile: inertial` 和 `serial.baud: 115200`。
必须提供新 inertial 能力；不会缺失时自动退回旧算法。磁航向的安装、北向和质量仍需确认，
不能用 `course_heading` 演示开关替代；未确认时保持初始化状态，状态会说明原因。

内部使用三维 15 状态误差滤波器。IMU 加速度去重力后积分，RTK 水平位置/速度校正，
磁航向辅助校正。初始连续静止 5 秒且有效固定/浮点解稳定 2 秒后输出；移动则重新采样。
IMU 位置用 `fusion.imu_xyz_m`（base_link 坐标中的三维米值），天线沿用 `antenna_xy_m`。
第一版不使用 GNSS 高程或气压，不声称楼层定位；旧地图无需迁移。

导航水平速度只取 ENU 水平分量并按航向转换，避免机身倾斜时把竖直漂移混入前进速度。
速度误差与 ROS Odometry 使用同一套水平误差传播，包含航向和安装偏移的影响。
RTK 测速、角速度和加速度共同确认静止后，按 `fusion.stationary_s` 等待并每 0.5 秒
进行一次三维零速修正；其强度由 `fusion.zupt_velocity_std_mps`（默认 0.05 m/s）控制。
这只是滤波内部校正，不增加导航准入条件。静止判定随历史状态一起回放，GNSS 失效时停止使用。

固定解/浮点解按报告误差融合；失锁后阶段为 inertial，持续输出估计且误差增长，无固定失锁秒数。
固定解恢复后对齐位置和有效速度，保留姿态与偏差；浮点解恢复按权重纠正。保留数据时间和精度
检查，不因与漂移惯导相距较远拒绝固定解。延迟数据最多回放 3 秒，重复/过期数据不重新积分。
IMU 短暂断流、重连、过期包和队列积压不清空已标定状态；默认新鲜度容忍 3 秒。
缺 IMU 时停止积分旧加速度/角速度，改用匀速预测并接受有效固定/浮点 RTK 的位置和速度校正。
短时保持最近航向；超过 3 秒，前向运动且接收机测速足以辨认方向时，用行进方向缓慢修正航向。
单天线 RTK 静止时无法感知转头：长期缺少 IMU 且无可靠行进方向时保留状态、等待航向恢复，
不伪造航向。IMU 恢复后直接续算；只有非有限数值等真正滤波故障才重新初始化。

`fusion` 状态含 stage、estimate_valid、navigation_ready、uncertainty、outage_s、recovery_id。
rtk_grade 单独报告新鲜接收机固定/浮点解；inertial_elapsed_s 为本次惯导时长。
fusion_elapsed_s 从最近一次接受的固定解计时，每条新固定解归零；尚无固定解时从初始化计时。
估计仍可用而误差超限时继续发 pose/TF，但 ready=false；默认水平 2σ≤3m、航向 2σ≤30°。
位置/速度协方差来自滤波器，速度来源正常为 inertial_estimate，缺 IMU 时为 rtk_aided_coast，并报告 imu_degraded/heading_source。噪声密度和偏差随机游走
默认值仅为初始参数，必须用实际运动记录校验；不是设备精度承诺。

现有导航只允许纯惯导继续已有任务；新任务等 RTK，惯导到终点先停车等 RTK 确认，
超出现有恢复等待时间则任务失败。恢复校正先撤回控制、释放旧 FollowPath，再重规划。
保留旧模式：`estimator_mode: legacy`，并同步将 IMU 改回 legacy 设备档位。
