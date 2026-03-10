#!/bin/bash
set -e

# 获取当前脚本所在目录的绝对路径 (用于存放生成的 deb 包)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# 配置
PKG_NAME="3dtof_calib"
# 如果没有参数传入，使用当前终端所在的目录作为 WORKSPACE
WORKSPACE="${1:-$PWD}"  
VERSION="1.0.0"
ARCH="arm64"
# 将 deb 包输出到脚本所在目录
OUTPUT_DEB="${SCRIPT_DIR}/${PKG_NAME}_${ARCH}.deb"

# 验证路径是否存在
if [ ! -d "$WORKSPACE/../../install/3dtof_calib" ]; then
    echo "错误：安装目录不存在: $WORKSPACE/install/3dtof_calib"
    echo "请先运行 colcon build --packages-select 3dtof_calib"
    exit 1
fi

echo "工作空间: $WORKSPACE"
echo "包名称: $PKG_NAME"
echo "输出路径: $OUTPUT_DEB"

# 编译 RawToBmpDemo_240719
echo "编译 RawToBmpDemo_240719..."

# 检测是否需要交叉编译
HOST_ARCH=$(uname -m)
SKIP_BUILD=false
CMAKE_ARGS=""

if [ "$SKIP_BUILD" = "false" ]; then
    cd "$WORKSPACE/RawToBmpDemo_240719"
    mkdir -p build
    cd build
    # 清理旧的构建文件以防万一，特别是如果之前是用不同编译器构建的
    if [ -f CMakeCache.txt ]; then
        rm CMakeCache.txt
    fi
    cmake .. $CMAKE_ARGS
    make
    cd "$WORKSPACE"
fi

# 清理并创建临时打包目录
# 在工作空间根目录创建临时文件夹 tmp_pkg
rm -rf "$WORKSPACE/tmp_pkg"
mkdir -p "$WORKSPACE/tmp_pkg/DEBIAN"
# 注意：这里保持目标设备上的安装路径为 /root/3dtof_calib 不变
mkdir -p "$WORKSPACE/tmp_pkg/root/3dtof_calib"

# 复制所有需要的内容
# 注意：这里假设 install/3dtof_calib 包含了 lib 和 share 目录
cp -r "$WORKSPACE/install/3dtof_calib/"* "$WORKSPACE/tmp_pkg/root/3dtof_calib/"

# 复制 RawToBmpDemo 可执行文件和依赖库
mkdir -p "$WORKSPACE/tmp_pkg/root/3dtof_calib/bin"
mkdir -p "$WORKSPACE/tmp_pkg/root/3dtof_calib/lib"

# 复制并重命名可执行文件，避免与启动脚本冲突
cp "$WORKSPACE/RawToBmpDemo_240719/build/rawToBmpDemo" "$WORKSPACE/tmp_pkg/root/3dtof_calib/bin/rawToBmpDemo_exec"

# 复制依赖的动态库
cp "$WORKSPACE/RawToBmpDemo_240719/third/librawToBmpLib.so" "$WORKSPACE/tmp_pkg/root/3dtof_calib/lib/"

# 创建启动脚本，设置 LD_LIBRARY_PATH
cat > "$WORKSPACE/tmp_pkg/root/3dtof_calib/bin/rawToBmpDemo" << 'SCRIPT'
#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# 设置库路径，指向 ../lib
export LD_LIBRARY_PATH=$SCRIPT_DIR/../lib:$LD_LIBRARY_PATH
# 执行重命名后的可执行文件
exec "$SCRIPT_DIR/rawToBmpDemo_exec" "$@"
SCRIPT
chmod +x "$WORKSPACE/tmp_pkg/root/3dtof_calib/bin/rawToBmpDemo"

# 创建 3dtof_calib 启动脚本
cat > "$WORKSPACE/tmp_pkg/root/3dtof_calib/bin/3dtof_calib" << 'SCRIPT'
#!/bin/bash
# Source ROS2 environment (assuming Humble, adjust if needed)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/tros/setup.bash ]; then
    source /opt/tros/setup.bash
fi

# Source package environment
if [ -f /root/3dtof_calib/local_setup.bash ]; then
    source /root/3dtof_calib/local_setup.bash
fi

# Run the executable
exec /root/3dtof_calib/lib/3dtof_calib/calib_node "$@"
SCRIPT
chmod +x "$WORKSPACE/tmp_pkg/root/3dtof_calib/bin/3dtof_calib"

# 创建 control 文件
INSTALL_SIZE=$(du -sk "$WORKSPACE/tmp_pkg/root/3dtof_calib" | cut -f1)

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
echo "3dtof_calib installed."
echo "Run /root/3dtof_calib/bin/3dtof_calib to start the calibration node."
echo "Run /root/3dtof_calib/bin/rawToBmpDemo to start the raw to bmp demo."
EOF
chmod 755 "$WORKSPACE/tmp_pkg/DEBIAN/postinst"

# 创建 prerm
cat > "$WORKSPACE/tmp_pkg/DEBIAN/prerm" << 'EOF'
#!/bin/bash
# No cleanup needed for self-contained package
EOF
chmod 755 "$WORKSPACE/tmp_pkg/DEBIAN/prerm"

# 构建 deb 包
dpkg-deb --build "$WORKSPACE/tmp_pkg" "$OUTPUT_DEB"

# 编译后清理临时构建目录
rm -rf "$WORKSPACE/tmp_pkg"

echo "构建完成: $OUTPUT_DEB"
echo "安装: sudo dpkg -i $OUTPUT_DEB"
