# 周期导航配对定位接口

状态 ROS 2 topic 的 JSON 新增可选 `navigation_sample`。只有定位 ready 且有有效估计样本时提供对象，否则为 null。旧消费者可以忽略该字段。

对象字段：schema_version=1；stamp_s 为 ROS 时间下的采样时刻（不是状态重复发布时刻）；sample_id 为本次估计的稳定标识；epoch 为定位实例及缺口恢复序号；reference_point=base_link；global_frame=map；local_frame=odom；map_pose/odom_pose 各含 x、y、yaw。

两个 pose 来自同一份估计样本，导航用它们转换目标点，再在固定窗口内去重求平均。不要将不同时间的两个 TF 查询拼成一份配对样本。定位重启或缺口恢复后必须丢弃旧窗口和导航任务。

新增字段不改变定位器的 GNSS/IMU 融合、杆臂补偿和 TF 发布权属，不把地图目标反写成定位观测，也不声称能消除 GNSS 偏差。
