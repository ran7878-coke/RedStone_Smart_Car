#!/bin/bash
# 脚本功能：cmake编译 -> 增量编译项目 -> 上传目标文件到远程设备

export PATH=/opt/ls_2k0300_env/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6/bin:$PATH

# ===================== 核心配置区（可根据需要修改，一目了然） =====================
WORK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"  # 脚本所在绝对路径（关键修复）
OUT_DIR="${WORK_DIR}/../out"                              # out目录绝对路径
USER_DIR="${WORK_DIR}/../user"                            # user目录绝对路径
REMOTE_IP="172.20.10.3"                                  # 远程设备IP
REMOTE_USER="root"                                        # 远程登录用户
REMOTE_PATH="/home/root/"                                 # 远程上传路径
MAKE_JOBS=$(nproc)                                        # make编译线程数

# ===================== 全局通用函数（复用性强，减少冗余代码） =====================
# 错误退出函数：打印错误信息 + 退出脚本（退出码1）
error_exit() {
    echo -e "\033[31m[ERROR] $1\033[0m"
    exit 1
}

# 成功提示函数：打印普通成功信息
info_echo() {
    echo -e "\033[32m[INFO] $1\033[0m"
}

# ===================== 脚本主逻辑 =====================
# 1. 进入out目录并校验
info_echo "准备进入目录: ${OUT_DIR}"
cd "${OUT_DIR}" || error_exit "无法进入 ${OUT_DIR} 目录，请检查目录是否存在！"

# 2. 执行cmake编译（首次全量生成，后续cmake/make自动检测增量）
info_echo "执行cmake编译: cmake ${USER_DIR}"
cmake "${USER_DIR}" || error_exit "cmake 编译失败，请检查CMakeLists.txt或编译依赖！"

# 3. 执行make多线程编译（增量编译，仅重编修改过的文件）
info_echo "cmake执行成功，开始执行 make -j${MAKE_JOBS} 增量编译..."
make -j${MAKE_JOBS} || error_exit "make 编译失败，编译日志如上！"

# 4. 获取上级目录名称
parent_dir_name=$(basename "$(dirname "$(pwd)")")
info_echo "待上传文件/目录：${parent_dir_name}"

# 5. SCP上传文件到远程设备，-O 兼容老版本openssh，增加上传校验
info_echo "开始上传文件到 ${REMOTE_USER}@${REMOTE_IP}:${REMOTE_PATH}"
scp -O "${parent_dir_name}" "${REMOTE_USER}@${REMOTE_IP}:${REMOTE_PATH}" || error_exit "文件上传失败，请检查网络/远程权限/文件是否存在！"

# 6. 全部执行完成
info_echo "✅ 所有操作执行完成：cmake编译 → make增量编译 → 文件上传 均成功！"
exit 0
