#!/usr/bin/env bash
set -eo pipefail
set +u

# 本脚本应在工作区根目录执行。
if [[ ! -f "third_party/genisom_L1_sdk/CMakeLists.txt" ]]; then
  echo "错误: 请在工作区根目录执行 scripts/build_all.sh" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
# ROS 环境脚本会读取未定义变量，加载完成后再恢复严格检查。
set -u

cmake -S third_party/genisom_L1_sdk -B build/official_sdk \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/official_sdk --parallel

# 清理本包旧安装目标，避免升级后残留历史可执行文件。
rm -rf -- install/genisom_l1_control
colcon build --symlink-install --packages-select genisom_l1_control

echo "构建完成。下一步执行: source install/setup.bash"
