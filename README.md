# 相机外参标定工具使用说明

## 1. 简介
本工具用于标定相机相对于机器人基座（Robot Base）的外参 ($T_{robot\_cam}$)。
原理：通过识别已知位姿的棋盘格标定板，利用 PnP 算法解算相机位姿。

## 2. 准备工作
1. **录制数据**：
   - 将标定板固定在机器人工作空间内。
   - 移动机器人或相机，录制包含棋盘格图像的 rosbag。
   - 确保图像清晰，棋盘格完整可见。
2. **准备内参**：
   - 准备标准 ROS 格式的相机内参文件 (`camera_info.yaml`)。
3. **测量位姿**：
   - 准确测量或从数模获取标定板在机器人基座坐标系下的位姿 ($T_{robot\_board}$)。
   - **注意**：默认标定板坐标系原点位于棋盘格的**第一个内角点**（左上角）标定板坐标系为右下前，
机器人坐标系（R） 

X轴：向前
Y轴：向左
Z轴：向上
原点：机器人底盘中心地面投影点。
标定板坐标系（B）

原点：标定板左上角第一个角点。


## 3. 配置文件
修改 `config/extrinsic_config.yaml`：

## 4. 运行
```bash
# 编译
colcon build --packages-select cam_extrinsic_calibtool
source install/setup.bash

# 运行
ros2 run cam_extrinsic_calibtool calib_node share/cam_extrinsic_calibtool/config/extrinsic_config.yaml
```

## 5. 结果
标定结果将保存为 YAML 文件，包含平移向量和旋转四元数/欧拉角。
