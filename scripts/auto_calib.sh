#!/bin/bash

# ================= 配置区域 =================
# 请在此处修改你的实际路径
PATH1="/root/mipi_ws/"      # 执行 ./exec.sh 的路径
PATH2="/root/tof1_image/"        # 图片保存路径 2
PATH3="/root/tof2_image/"       # 图片保存路径 3
PATH4="/root/cam_extrinsic_calibtool/install"         # setup.bash 所在路径
PATH5="/root/cam_extrinsic_calibtool/src/cam_extrinsic_calibtool/scripts/"          # python 脚本所在路径
PYTHON_SCRIPT_NAME="raw_to_calib_jpg.py"       # 具体 python 脚本的文件名
TARGET_IMG_COUNT=100                      # 目标图片数量
# ===========================================

echo "[Step 1] 准备开始..."

# 1. 清理旧数据 (可选，为了防止计数错误，建议先清空图片文件夹)
mkdir -p "$PATH2" "$PATH3" # 确保文件夹存在
rm -f "$PATH2"/* "$PATH3"/*
# echo "已清理旧图片数据."

# 2. 在路径1下执行 ./exec.sh
echo "[Step 2] 启动 exec.sh 程序..."
cd "$PATH1" || { echo "无法进入路径: $PATH1"; exit 1; }

# 在后台执行 exec.sh，并将 PID 保存下来以便后续终止
./exec.sh &
EXEC_PID=$!
echo "程序已在后台运行，PID: $EXEC_PID"

# 3. 循环监控路径2和3下的图片数量
echo "[Step 3] 开始监控图片数量 (目标: $TARGET_IMG_COUNT 张)..."

while true; do
    # 统计文件数量 (ls -1 | wc -l 统计文件行数)
    # 注意：如果文件夹不存在，这里可能会报错，所以加了 || echo 0
    count2=$(ls -1 "$PATH2" 2>/dev/null | wc -l)
    count3=$(ls -1 "$PATH3" 2>/dev/null | wc -l)

    echo -ne "当前数量 -> 路径2: $count2 | 路径3: $count3 \r"

    # 判断是否任意一个文件夹达到目标数量 (这里是用 OR 逻辑，如果需要两个都达到，将 -ge 改为逻辑判断)
    if [ "$count2" -ge "$TARGET_IMG_COUNT" ] || [ "$count3" -ge "$TARGET_IMG_COUNT" ]; then
        echo -e "\n达到目标图片数量！"
        break
    fi

    # 检查后台进程是否意外退出了
    if ! kill -0 $EXEC_PID 2>/dev/null; then
        echo -e "\n错误：exec.sh 进程意外停止了！"
        exit 1
    fi

    sleep 1
done

# ================= 替换原脚本 Step 4 =================
echo "[Step 4] 正在终止 exec.sh (PID: $EXEC_PID)..."

# 1. 尝试发送 SIGINT (Ctrl+C)
kill -SIGINT $EXEC_PID 2>/dev/null
# 等待最多 5 秒让它退出
for i in {1..3}; do
    if ! kill -0 $EXEC_PID 2>/dev/null; then
        break
    fi
    sleep 1
done

# 2. 如果还在运行，发送 SIGTERM
if kill -0 $EXEC_PID 2>/dev/null; then
    echo "警告: 进程未响应 Ctrl+C，尝试发送 SIGTERM..."
    kill -SIGTERM $EXEC_PID 2>/dev/null
    sleep 2
fi

# 3. 如果还在运行，发送 SIGKILL (强制杀死)
if kill -0 $EXEC_PID 2>/dev/null; then
    echo "警告: 进程顽固，执行强制杀死 (kill -9)..."
    kill -9 $EXEC_PID 2>/dev/null
fi

# 4. 确保子进程也被清理 (可选，如果 exec.sh 启动了很重的子进程)
# 假设 exec.sh 启动的名字里包含 "my_camera_app" 这样的关键字，可以取消下面注释
# pkill -9 -f "my_camera_app"

echo "exec.sh 已完全停止."
# ===================================================

# 5. Source install.bash
echo "[Step 5] Source 环境配置..."
SOURCE_FILE="$PATH4/setup.bash" # 通常 ros2 是 setup.bash，如果是 install.bash 请修改这里
if [ -f "$SOURCE_FILE" ]; then
    source "$SOURCE_FILE"
    echo "环境配置已加载: $SOURCE_FILE"
else
    # 尝试用户指定的 install.bash
    if [ -f "$PATH4/install.bash" ]; then
        source "$PATH4/install.bash"
        echo "环境配置已加载: $PATH4/install.bash"
    else
        echo "错误：找不到 install.bash 或 setup.bash 文件"
        exit 1
    fi
fi

# 6. 执行 Python 脚本
echo "[Step 6] 执行 Python 脚本..."
cd "$PATH5" || { echo "无法进入路径: $PATH5"; exit 1; }
python3 "$PYTHON_SCRIPT_NAME"

# 检查 Python 脚本执行结果
if [ $? -ne 0 ]; then
    echo "Python 脚本执行失败，终止后续步骤。"
    exit 1
fi

# 7. 执行 ROS2 节点
echo "[Step 7] 执行 ROS2 标定节点..."
# 此时应该已经在前面 source 过环境了
ros2 run cam_extrinsic_calibtool calib_node

echo "脚本执行完毕。"