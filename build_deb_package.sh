#!/bin/bash
set -e

# 获取当前脚本所在目录的绝对路径 (用于存放生成的 deb 包)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# 配置
PKG_NAME="cam-extrinsic-calibtool"
# 如果没有参数传入，使用当前终端所在的目录作为 WORKSPACE
WORKSPACE="${1:-$PWD}"  
VERSION="1.0.0"
ARCH="arm64"
# 将 deb 包输出到脚本所在目录
OUTPUT_DEB="${SCRIPT_DIR}/${PKG_NAME}_${ARCH}.deb"

# 验证路径是否存在
if [ ! -d "$WORKSPACE/install/cam_extrinsic_calibtool" ]; then
    echo "错误：安装目录不存在: $WORKSPACE/install/cam_extrinsic_calibtool"
    echo "请先运行 colcon build --packages-select cam_extrinsic_calibtool"
    exit 1
fi

echo "工作空间: $WORKSPACE"
echo "包名称: $PKG_NAME"
echo "输出路径: $OUTPUT_DEB"

# 编译 RawToBmpDemo_240719
echo "编译 RawToBmpDemo_240719..."
cd "$WORKSPACE/RawToBmpDemo_240719"
mkdir -p build
cd build
cmake ..
make
cd "$WORKSPACE"

# 清理并创建临时打包目录
# 在工作空间根目录创建临时文件夹 tmp_pkg
rm -rf "$WORKSPACE/tmp_pkg"
mkdir -p "$WORKSPACE/tmp_pkg/DEBIAN"
# 注意：这里保持目标设备上的安装路径为 /root/cam_extrinsic_calibtool 不变
mkdir -p "$WORKSPACE/tmp_pkg/root/cam_extrinsic_calibtool"

# 复制所有需要的内容
# 注意：这里假设 install/cam_extrinsic_calibtool 包含了 lib 和 share 目录
cp -r "$WORKSPACE/install/cam_extrinsic_calibtool/"* "$WORKSPACE/tmp_pkg/root/cam_extrinsic_calibtool/"

# 创建 control 文件
INSTALL_SIZE=$(du -sk "$WORKSPACE/tmp_pkg/root/cam_extrinsic_calibtool" | cut -f1)

cat > "$WORKSPACE/tmp_pkg/DEBIAN/control" << EOF
Package: ${PKG_NAME}
Version: ${VERSION}
Architecture: ${ARCH}
Maintainer: zzh <1016001513@qq.com>
Installed-Size: ${INSTALL_SIZE}
Depends: bash, python3
Description: Camera Extrinsic Calibration Tool
 A tool for camera extrinsic calibration.
Section: utils
Priority: optional
EOF

# 创建 postinst
cat > "$WORKSPACE/tmp_pkg/DEBIAN/postinst" << 'EOF'
#!/bin/bash
# 创建启动脚本
cat > /usr/local/bin/cam-extrinsic-calibtool << 'SCRIPT'
#!/bin/bash
# Source ROS2 environment (assuming Humble, adjust if needed)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/tros/setup.bash ]; then
    source /opt/tros/setup.bash
fi

# Source package environment
if [ -f /root/cam_extrinsic_calibtool/local_setup.bash ]; then
    source /root/cam_extrinsic_calibtool/local_setup.bash
fi

# Run the executable
exec /root/cam_extrinsic_calibtool/lib/cam_extrinsic_calibtool/calib_node "$@"
SCRIPT

chmod +x /usr/local/bin/cam-extrinsic-calibtool
echo "Camera Extrinsic Calibration Tool installed. Use 'cam-extrinsic-calibtool' to run."
EOF
chmod 755 "$WORKSPACE/tmp_pkg/DEBIAN/postinst"

# 创建 prerm
cat > "$WORKSPACE/tmp_pkg/DEBIAN/prerm" << 'EOF'
#!/bin/bash
rm -f /usr/local/bin/cam-extrinsic-calibtool
EOF
chmod 755 "$WORKSPACE/tmp_pkg/DEBIAN/prerm"

# 构建 deb 包
dpkg-deb --build "$WORKSPACE/tmp_pkg" "$OUTPUT_DEB"

# 编译后清理临时构建目录
rm -rf "$WORKSPACE/tmp_pkg"

echo "构建完成: $OUTPUT_DEB"
echo "安装: sudo dpkg -i $OUTPUT_DEB"
