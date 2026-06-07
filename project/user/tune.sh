#!/bin/bash
# 热加载 PID 参数到小车
# 用法:
#   ./tune.sh Speed_l_Kp 500
#   ./tune.sh Speed_l_Kp 500 Speed_l_Ki 10 Speed_r_Kp 400   (批量)
#   ./tune.sh show                                        (查看当前参数)

REMOTE_IP="172.20.10.3"
REMOTE_USER="root"
REMOTE_PATH="/home/root/pid_tune.txt"

TMPFILE=$(mktemp)

if [ "$1" = "show" ]; then
    echo "当前 PID 参数（最后一次 tune 的值）:"
    ssh "${REMOTE_USER}@${REMOTE_IP}" "cat ${REMOTE_PATH} 2>/dev/null || echo '暂无记录'"
    rm -f "$TMPFILE"
    exit 0
fi

# 将参数对写入临时文件（每对一行）
while [ $# -ge 2 ]; do
    echo "$1 $2" >> "$TMPFILE"
    shift 2
done

if [ ! -s "$TMPFILE" ]; then
    echo "用法: $0 <Key1> <Value1> [<Key2> <Value2> ...]"
    echo "示例: $0 Speed_l_Kp 500 Speed_l_Ki 10"
    echo ""
    echo "支持的 Key:"
    echo "  Speed_l_Kp/Ki/Kd   Speed_r_Kp/Ki/Kd   Delta_Sp_Kp/Ki/Kd"
    echo "  Trace_Kp/Ki/Kd     Angle_Kp/Ki/Kd"
    rm -f "$TMPFILE"
    exit 1
fi

echo "即将发送以下参数到小车:"
cat "$TMPFILE"

scp -O "$TMPFILE" "${REMOTE_USER}@${REMOTE_IP}:${REMOTE_PATH}" && \
    echo "✅ 已发送，1秒内生效（串口会打印确认）"

rm -f "$TMPFILE"
