# Outdoor localization service for Robonix

独立定位 service：从 G70 GNSS 与 WIT IMU 原语的 ROS 能力读取测量，输出导航所需的 Odometry、map 位姿、TF 和就绪状态。

```text
primitive-g70-gnss-rbnx  ── fix / RTK status / receiver velocity ──┐
                                                               ├─ 本 service ── odom / pose / TF / status ── navigation service
primitive-wit-imu-rbnx ── Imu / gyro / status ────────────────────┘
```

运行不依赖旧 outdoorNav 工程，不打开串口、不处理 NMEA/UBX/WIT 字节、不连接 NTRIP，不订阅山猫 `/ODOM`，不订阅控制指令，不通过积分加速度或相减 GNSS 位置冒充速度。当前网页保持在原业务侧，未来由 Scene 消费定位和导航能力，本文不实现新前端。

## 配套版本状态

本仓需要配套的 G70 速度／GST 扩展、WIT 独立 gyro 扩展，以及导航服务的 localization 契约接入修改。

- 导航服务对接已发布到 main：`18c14136774eaf091080662eae0bcbf7728a2f33`。
- G70 扩展已发布到 main：`1bc5c5d38e8cb2b55c2fc95bee87b860550e3de9`。
- WIT 扩展本地提交：`1f56b1260abc344c454a9dfc01e0106b895b0d86`，尚未发布。

当前验证报告对应上述配套版本；WIT 原语扩展发布前，不能直接按示例的远端 main 组合运行。
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
- 标准 Imu 已带角速度；新增独立 gyro 是为了姿态帧断流时，仍能用实际角速度判断停车。旧 Imu 的发布/单位语义不变。
- 不自动发送设备配置命令。实际硬件是否已输出 RMC/GST、是否兼容安装与输出量程，需要后续单独验证。

## 输出契约

- `robonix/service/localization/odom`：nav_msgs/Odometry，连续局部位姿和实测速度反馈，frame `odom`、child `base_link`。
- `robonix/service/localization/pose`：PoseWithCovarianceStamped，地图中的 base_link 位姿。
- `robonix/service/localization/status`：String JSON，就绪、RTK 等级、航向/速度有效性、故障与来源。
- 独占动态 TF：`map→odom` 和 `odom→base_link`。TF 话题由部署配置绑定，不与其他定位器重复发布。

地图校正由接受的 GNSS 位置与已修正 IMU 航向提供；局部里程仅积分接收机地速和 gyro。停用时就绪状态立即撤回；测量仍可用于停车反馈。TF 重复所有者与异常跳变会停止全局就绪。

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
