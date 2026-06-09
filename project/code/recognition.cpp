//-------------------------------------------------------------------------------------------------------------------
//  @brief      基于AI模型的红色触发区识别 + 目标板分类
//  @idea       1. 彩色摄像头检测 12cm×5cm 红色长方形区域（触发区）
//              2. 触发后在红色区域后方提取识别板ROI
//              3. 送入CNN模型进行三分类（物资/交通工具/武器）
//  @platform   龙芯 LS2K0300 + TensorFlow Lite Micro (int8量化)
//-------------------------------------------------------------------------------------------------------------------
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/c/common.h"
#include "zf_common_headfile.hpp"
#include <cstring>
#include <cstdio>
#include <cmath>
#include "recognition.hpp"

// ==================== 模型全局变量 ====================
static const tflite::Model* g_model = nullptr;
static tflite::MicroInterpreter* g_interpreter = nullptr;
static TfLiteTensor* g_input_tensor = nullptr;
static TfLiteTensor* g_output_tensor = nullptr;
static uint8_t* g_tensor_arena = nullptr;
static bool g_model_initialized = false;

// 模型类型标记（自动检测 float32 / int8）
static bool g_is_float_model = false;
// 量化参数（仅 int8 模型使用，自动从模型读取）
static float g_input_scale = 0.0f;
static int g_input_zero_point = 0;
static float g_output_scale = 0.0f;
static int g_output_zero_point = 0;

// ==================== 类别枚举 ====================
typedef enum {
    CATEGORY_SUPPLY = 0,    // 物资
    CATEGORY_VEHICLE = 1,   // 交通工具
    CATEGORY_WEAPON = 2,    // 武器
    CATEGORY_UNKNOWN = 3    // 未知
} ImageCategory;

// ==================== 红色矩形检测参数（12cm × 5cm）====================
// 物理尺寸 12cm×5cm，宽高比 = 12/5 = 2.4
// 在160×120图像中，根据摄像头高度和视角预估像素范围
#define RED_RECT_MIN_W          15      // 最小宽度（像素），放宽到15px
#define RED_RECT_MAX_W          130     // 最大宽度（像素）
#define RED_RECT_MIN_H          6       // 最小高度（像素），放宽到6px
#define RED_RECT_MAX_H          60      // 最大高度（像素）
#define RED_RECT_ASPECT_MIN     1.2f    // 最小宽高比（大幅放宽畸变容忍）
#define RED_RECT_ASPECT_MAX     4.0f    // 最大宽高比
#define RED_RECT_FILL_MIN       0.25f   // 矩形内红色填充率（降低到25%）

// RGB565 红色判定阈值（大幅放宽，优先保证能检测到）
// 策略：R明显高于G和B即为红色，不做过于严格的绝对阈值限制
#define RED_R_MIN               10      // R通道最低值（0-31），非常宽松
#define RED_G_MAX               25      // G通道最高值（0-63），放宽
#define RED_B_MAX               20      // B通道最高值（0-31），放宽
#define RED_R_G_RATIO           1.3f    // R至少是G的1.3倍（相对比值更鲁棒）
#define RED_R_B_RATIO           1.3f    // R至少是B的1.3倍

// 搜索区域：扫描图像中下部（红色触发区通常在地面上）
#define RED_SEARCH_Y_START      30      // 从第30行开始
#define RED_SEARCH_Y_END        118     // 扫描到倒数第2行

// 连续检测帧数和延迟
#define RED_DEBOUNCE_FRAMES     2       // 连续N帧检测到矩形才触发（降低到2）
#define TARGET_DELAY_FRAMES     3       // 触发后延迟N帧再识别
#define COOLDOWN_FRAMES         30      // 识别完成后冷却N帧

// 识别板ROI：在红色矩形的上方扩展区域
#define BOARD_ROI_EXTEND_UP     60      // 向上扩展像素数
#define BOARD_ROI_EXTEND_SIDE   20      // 左右扩展像素数

// 调试开关：每N帧打印一次红色像素统计
#define RED_DEBUG_INTERVAL      30

// ==================== 红色矩形检测结构体 ====================
typedef struct {
    int x, y;           // 左上角坐标
    int w, h;           // 宽度和高度
    int area;           // 内部红色像素数
    float fill_ratio;   // 填充率 = area / (w*h)
    float aspect;       // 宽高比 w/h
    int center_x;       // 中心X坐标
    int center_y;       // 中心Y坐标
} RedRect;

// ==================== 状态机 ====================
typedef enum {
    STATE_WAIT_RED = 0,         // 等待红色触发区
    STATE_RED_CONFIRMED,        // 红色矩形已确认
    STATE_APPROACH_TARGET,      // 接近目标板
    STATE_CLASSIFY,             // 执行分类
    STATE_COOLDOWN              // 冷却期，防止重复触发
} DetectState;

static DetectState g_detect_state = STATE_WAIT_RED;
static int g_red_debounce_cnt = 0;
static int g_target_delay_cnt = 0;
static int g_cooldown_cnt = 0;
static RedRect g_last_red_rect = {0};  // 记录最后一次检测到的红色矩形
static float g_last_supply_conf = 0.0f;
static float g_last_vehicle_conf = 0.0f;
static float g_last_weapon_conf = 0.0f;
static ImageCategory g_last_category = CATEGORY_UNKNOWN;

// ==================== 辅助函数：RGB565 → 单通道判断 ====================
static inline bool is_red_pixel(uint16_t pixel)
{
    uint8_t r = (pixel >> 11) & 0x1F;   // 5位R (0-31)
    uint8_t g = (pixel >> 5)  & 0x3F;   // 6位G (0-63)
    uint8_t b = pixel & 0x1F;            // 5位B (0-31)

    // 红色判定（简化版，适应偏橙/偏黄的赛道红色）
    // 核心条件：R是最高通道 + R不低于阈值 + B明显低于R
    if (r < RED_R_MIN) return false;        // R不能太暗
    if (r <= g) return false;               // R必须大于G（排除灰/白/黄）
    if (r <= b + 2) return false;           // R必须明显大于B（排除品红/紫）

    // G和B的绝对上限（排除过亮的非红色区域）
    if (g > RED_G_MAX && r < 18) return false;   // G很高时要求R也较高
    if (b > RED_B_MAX && r < 15) return false;   // B很高时要求R也较高

    return true;
}

// ==================== 红色矩形检测（基于游程编码+连通域分析）====================
// 原理：
//   1. 逐行扫描，将连续红色像素压缩为"游程"（run-length）
//   2. 合并相邻行的重叠游程，形成连通域
//   3. 对每个连通域计算边界框、面积、填充率、宽高比
//   4. 筛选最匹配 12cm×5cm（宽高比≈2.4）的矩形
//
// 返回：检测到的有效红色矩形数量
static int detect_red_rectangles(const uint16_t* rgb565_img,
                                  int img_w, int img_h,
                                  RedRect* results, int max_results)
{
    if (!rgb565_img || !results || max_results <= 0) return 0;

    // ------- 第一步：逐行游程编码 -------
    // 每行最多存储 N 个游程
    #define MAX_RUNS_PER_ROW    16
    typedef struct { int x_start, x_end, y; } Run;
    static Run all_runs[RED_SEARCH_Y_END - RED_SEARCH_Y_START][MAX_RUNS_PER_ROW];
    static int run_counts[RED_SEARCH_Y_END - RED_SEARCH_Y_START];

    int total_rows = 0;

    for (int y = RED_SEARCH_Y_START; y < RED_SEARCH_Y_END; y++) {
        int run_idx = 0;
        int x = 0;
        while (x < img_w && run_idx < MAX_RUNS_PER_ROW) {
            // 跳过非红色像素
            while (x < img_w && !is_red_pixel(rgb565_img[y * img_w + x])) {
                x++;
            }
            if (x >= img_w) break;

            int x_start = x;
            // 收集连续红色像素
            while (x < img_w && is_red_pixel(rgb565_img[y * img_w + x])) {
                x++;
            }
            int x_end = x - 1;

            // 过滤太短的游程（噪声）
            if (x_end - x_start >= 3) {
                all_runs[total_rows][run_idx].x_start = x_start;
                all_runs[total_rows][run_idx].x_end   = x_end;
                all_runs[total_rows][run_idx].y       = y;
                run_idx++;
            }
        }
        run_counts[total_rows] = run_idx;
        total_rows++;
    }

    // ------- 第二步：连通域分析（行间游程合并）-------
    #define MAX_BLOBS   16
    typedef struct {
        int x_min, x_max, y_min, y_max;
        int pixel_count;
        bool active;
    } Blob;
    static Blob blobs[MAX_BLOBS];
    int blob_count = 0;

    // 初始化：第一行的每个游程创建一个blob
    for (int r = 0; r < run_counts[0] && blob_count < MAX_BLOBS; r++) {
        Run* run = &all_runs[0][r];
        blobs[blob_count].x_min = run->x_start;
        blobs[blob_count].x_max = run->x_end;
        blobs[blob_count].y_min = run->y;
        blobs[blob_count].y_max = run->y;
        blobs[blob_count].pixel_count = run->x_end - run->x_start + 1;
        blobs[blob_count].active = true;
        blob_count++;
    }

    // 逐行处理：尝试将当前行游程与现有blob合并
    for (int row = 1; row < total_rows; row++) {
        int cur_y = RED_SEARCH_Y_START + row;
        int num_runs = run_counts[row];

        for (int r = 0; r < num_runs; r++) {
            Run* run = &all_runs[row][r];
            bool merged = false;

            // 尝试与现有活跃blob合并
            for (int b = 0; b < blob_count; b++) {
                if (!blobs[b].active) continue;

                // 检查垂直连续性（行间距不超过2行）
                if (cur_y - blobs[b].y_max > 3) {
                    blobs[b].active = false;  // 该blob不再增长
                    continue;
                }

                // 检查水平重叠（游程与blob的X范围有交集）
                if (run->x_start <= blobs[b].x_max + 4
                    && run->x_end >= blobs[b].x_min - 4)
                {
                    // 合并到现有blob
                    if (run->x_start < blobs[b].x_min) blobs[b].x_min = run->x_start;
                    if (run->x_end   > blobs[b].x_max) blobs[b].x_max = run->x_end;
                    blobs[b].y_max = cur_y;
                    blobs[b].pixel_count += run->x_end - run->x_start + 1;
                    merged = true;
                    break;
                }
            }

            // 未合并 → 新建blob
            if (!merged && blob_count < MAX_BLOBS) {
                blobs[blob_count].x_min = run->x_start;
                blobs[blob_count].x_max = run->x_end;
                blobs[blob_count].y_min = cur_y;
                blobs[blob_count].y_max = cur_y;
                blobs[blob_count].pixel_count = run->x_end - run->x_start + 1;
                blobs[blob_count].active = true;
                blob_count++;
            }
        }

        // 标记长时间未更新的blob为不活跃
        for (int b = 0; b < blob_count; b++) {
            if (blobs[b].active && cur_y - blobs[b].y_max > 3) {
                blobs[b].active = false;
            }
        }
    }

    // ------- 第三步：筛选有效矩形 -------
    int valid_count = 0;
    for (int b = 0; b < blob_count && valid_count < max_results; b++) {
        int bw = blobs[b].x_max - blobs[b].x_min + 1;
        int bh = blobs[b].y_max - blobs[b].y_min + 1;

        // 尺寸过滤
        if (bw < RED_RECT_MIN_W || bw > RED_RECT_MAX_W) continue;
        if (bh < RED_RECT_MIN_H || bh > RED_RECT_MAX_H) continue;

        // 宽高比过滤（12cm:5cm = 2.4:1）
        float aspect = (float)bw / (float)bh;
        if (aspect < RED_RECT_ASPECT_MIN || aspect > RED_RECT_ASPECT_MAX) continue;

        // 填充率过滤（矩形内部红色像素占比）
        float fill = (float)blobs[b].pixel_count / (float)(bw * bh);
        if (fill < RED_RECT_FILL_MIN) continue;

        // 填充
        results[valid_count].x         = blobs[b].x_min;
        results[valid_count].y         = blobs[b].y_min;
        results[valid_count].w         = bw;
        results[valid_count].h         = bh;
        results[valid_count].area      = blobs[b].pixel_count;
        results[valid_count].fill_ratio = fill;
        results[valid_count].aspect    = aspect;
        results[valid_count].center_x  = blobs[b].x_min + bw / 2;
        results[valid_count].center_y  = blobs[b].y_min + bh / 2;
        valid_count++;
    }

    return valid_count;
}

// ==================== 从红色矩形后方提取识别板ROI ====================
// 识别板位于红色矩形上方（物理位置在触发区后方）
// 提取的ROI被缩放到模型输入尺寸（40×40）
//
// 提取策略：
//   - 以红色矩形上边界为基准，向上扩展 BOARD_ROI_EXTEND_UP 像素
//   - 左右各扩展 BOARD_ROI_EXTEND_SIDE 像素
//   - 得到一个正方形区域用于分类
static bool extract_board_roi(const uint16_t* rgb565_img, int img_w, int img_h,
                               const RedRect* red_rect,
                               uint8_t* roi_rgb888, int roi_size)
{
    if (!rgb565_img || !red_rect || !roi_rgb888) return false;

    // 计算识别板ROI的边界（在红色矩形上方）
    // ROI高度 = 红色矩形宽度的2/3（假设识别板大小与触发区成比例）
    int roi_h = red_rect->w;
    if (roi_h > img_h / 2) roi_h = img_h / 2;  // 不超过图像一半

    int roi_y_start = red_rect->y - roi_h - 5;  // 红色矩形上方，留5像素间距
    if (roi_y_start < 0) roi_y_start = 0;

    int roi_y_end = red_rect->y - 5;
    if (roi_y_end > img_h - 1) roi_y_end = img_h - 1;

    int actual_roi_h = roi_y_end - roi_y_start;
    if (actual_roi_h < 10) {
        // 识别板太靠上，使用红色矩形上方的所有区域
        roi_y_start = 0;
        roi_y_end = red_rect->y - 3;
        actual_roi_h = roi_y_end - roi_y_start;
    }

    // ROI宽度 = 红色矩形宽度 + 两侧扩展
    int roi_x_start = red_rect->x - BOARD_ROI_EXTEND_SIDE;
    if (roi_x_start < 0) roi_x_start = 0;

    int roi_x_end = red_rect->x + red_rect->w + BOARD_ROI_EXTEND_SIDE;
    if (roi_x_end >= img_w) roi_x_end = img_w - 1;

    int actual_roi_w = roi_x_end - roi_x_start;
    if (actual_roi_w < 10) return false;

    // 双线性插值缩放到 roi_size × roi_size
    float x_ratio = (float)actual_roi_w / roi_size;
    float y_ratio = (float)actual_roi_h / roi_size;

    for (int y = 0; y < roi_size; y++) {
        for (int x = 0; x < roi_size; x++) {
            // 计算源图像中的浮点坐标
            float src_x_f = roi_x_start + x * x_ratio;
            float src_y_f = roi_y_start + y * y_ratio;

            int src_x = (int)src_x_f;
            int src_y = (int)src_y_f;

            // 边界裁剪
            if (src_x < 0) src_x = 0;
            if (src_x >= img_w - 1) src_x = img_w - 2;
            if (src_y < 0) src_y = 0;
            if (src_y >= img_h - 1) src_y = img_h - 2;

            // 最近邻采样（嵌入式平台优先速度）
            uint16_t pixel = rgb565_img[src_y * img_w + src_x];

            uint8_t r5 = (pixel >> 11) & 0x1F;
            uint8_t g6 = (pixel >> 5)  & 0x3F;
            uint8_t b5 = pixel & 0x1F;

            int base = y * roi_size * 3 + x * 3;
            roi_rgb888[base + 0] = (r5 << 3) | (r5 >> 2);  // R (5→8 bit)
            roi_rgb888[base + 1] = (g6 << 2) | (g6 >> 4);  // G (6→8 bit)
            roi_rgb888[base + 2] = (b5 << 3) | (b5 >> 2);  // B (5→8 bit)
        }
    }

    return true;
}

// ==================== 模型初始化（int8版本）====================
int model_init(const char* model_path)
{
    (void)model_path;

    // 清理旧资源
    if (g_interpreter) { delete g_interpreter; g_interpreter = nullptr; }
    if (g_tensor_arena) { free(g_tensor_arena); g_tensor_arena = nullptr; }
    g_model_initialized = false;

    // 加载模型
    g_model = tflite::GetModel(loong_cnn_model_simple_tflite);
    if (!g_model) {
        printf("❌ [识别] 模型加载失败\n");
        return -1;
    }

    if (g_model->version() != TFLITE_SCHEMA_VERSION) {
        printf("❌ [识别] 模型版本不匹配！需要：%d，当前：%d\n",
               TFLITE_SCHEMA_VERSION, g_model->version());
        return -2;
    }

    // ================================================================
    // 模型结构诊断：打印算子数量和类型（帮助定位缺失算子）
    // ================================================================
    {
        const tflite::SubGraph* subgraph = (*g_model->subgraphs())[0];
        const flatbuffers::Vector<flatbuffers::Offset<tflite::Operator>>* ops =
            subgraph->operators();
        const flatbuffers::Vector<flatbuffers::Offset<tflite::OperatorCode>>* opcodes =
            g_model->operator_codes();

        printf("   [识别] 模型包含 %u 个算子\n", ops->size());

        for (uint32_t i = 0; i < ops->size(); i++) {
            const tflite::Operator* op = ops->Get(i);
            uint32_t opcode_idx = op->opcode_index();
            if (opcode_idx < opcodes->size()) {
                const tflite::OperatorCode* opcode = opcodes->Get(opcode_idx);
                tflite::BuiltinOperator builtin = opcode->builtin_code();
                const char* op_name = "Unknown";
                switch (builtin) {
                    case tflite::BuiltinOperator_CONV_2D:          op_name = "Conv2D"; break;
                    case tflite::BuiltinOperator_MAX_POOL_2D:      op_name = "MaxPool2D"; break;
                    case tflite::BuiltinOperator_AVERAGE_POOL_2D:  op_name = "AvgPool2D"; break;
                    case tflite::BuiltinOperator_FULLY_CONNECTED:  op_name = "FullyConnected"; break;
                    case tflite::BuiltinOperator_RELU:             op_name = "Relu"; break;
                    case tflite::BuiltinOperator_SOFTMAX:          op_name = "Softmax"; break;
                    case tflite::BuiltinOperator_RESHAPE:          op_name = "Reshape"; break;
                    case tflite::BuiltinOperator_PAD:              op_name = "Pad"; break;
                    case tflite::BuiltinOperator_DEPTHWISE_CONV_2D: op_name = "DepthwiseConv2D"; break;
                    case tflite::BuiltinOperator_CONCATENATION:    op_name = "Concat"; break;
                    case tflite::BuiltinOperator_ADD:              op_name = "Add"; break;
                    default: {
                        static char buf[32];
                        snprintf(buf, sizeof(buf), "BuiltinOp#%d", (int)builtin);
                        op_name = buf;
                    }
                }
                printf("   [识别]   算子[%u]：%s\n", i, op_name);
            }
        }
    }

    // ================================================================
    // 算子注册：逐飞TFLM库支持的算子有限，按模型实际需要注册
    // 如果模型含 Reshape/Pad/AvgPool 等算子，需对应增加
    // ================================================================
    static tflite::MicroMutableOpResolver<8> resolver;

    // 基础算子（所有CNN模型必备）
    if (resolver.AddConv2D() != kTfLiteOk) {
        printf("⚠️ [识别] Conv2D 算子注册失败\n");
    }
    if (resolver.AddMaxPool2D() != kTfLiteOk) {
        printf("⚠️ [识别] MaxPool2D 算子注册失败\n");
    }
    if (resolver.AddFullyConnected() != kTfLiteOk) {
        printf("⚠️ [识别] FullyConnected 算子注册失败\n");
    }
    if (resolver.AddRelu() != kTfLiteOk) {
        printf("⚠️ [识别] Relu 算子注册失败\n");
    }
    if (resolver.AddSoftmax() != kTfLiteOk) {
        printf("⚠️ [识别] Softmax 算子注册失败\n");
    }

    // 扩展算子（int8量化模型可能隐含需要）
    // Reshape: 许多模型在 FC 层前需要展平
    resolver.AddReshape();
    // AveragePool2D: 部分模型用平均池化替代最大池化
    resolver.AddAveragePool2D();
    // Pad:  Conv2D "SAME" padding 在某些TFLM版本需要单独的 Pad op
    resolver.AddPad();

    printf("✅ [识别] 算子注册完成（8个槽位）\n");

    // ================================================================
    // 张量内存分配：使用头文件定义的 TENSOR_ARENA_SIZE（1MB 默认）
    // 如果 AllocateTensors 仍失败，说明模型架构超出预期，需进一步增大
    // ================================================================
    #define ACTUAL_TENSOR_SIZE  TENSOR_ARENA_SIZE
    int ret = posix_memalign((void**)&g_tensor_arena, 64, ACTUAL_TENSOR_SIZE);
    if (ret != 0 || g_tensor_arena == nullptr) {
        printf("❌ [识别] 动态内存分配失败（请求 %d MB）\n",
               ACTUAL_TENSOR_SIZE / 1024 / 1024);
        return -3;
    }
    memset(g_tensor_arena, 0, ACTUAL_TENSOR_SIZE);
    printf("✅ [识别] 动态分配张量池成功：%d MB\n",
           ACTUAL_TENSOR_SIZE / 1024 / 1024);

    // 创建解释器
    g_interpreter = new tflite::MicroInterpreter(
        g_model, resolver, g_tensor_arena, ACTUAL_TENSOR_SIZE);
    if (!g_interpreter) {
        printf("❌ [识别] 解释器创建失败\n");
        free(g_tensor_arena);
        g_tensor_arena = nullptr;
        return -4;
    }

    // 分配张量内存
    TfLiteStatus status = g_interpreter->AllocateTensors();
    if (status != kTfLiteOk) {
        // 打印诊断信息：报告模型需要的 Arena 大小
        size_t required = g_interpreter->arena_used_bytes();
        printf("❌ [识别] 张量内存分配失败，错误码：%d\n", status);
        printf("   [识别] 模型预估需要：%zu bytes (%zu KB)\n",
               required, required / 1024);
        printf("   [识别] 当前Arena大小：%d bytes (%d MB)\n",
               ACTUAL_TENSOR_SIZE, ACTUAL_TENSOR_SIZE / 1024 / 1024);
        printf("   [识别] 建议：在 recognition.hpp 增大 TENSOR_ARENA_SIZE\n");
        printf("   [识别] 或检查模型是否包含TFLM不支持的算子\n");
        delete g_interpreter;  g_interpreter = nullptr;
        free(g_tensor_arena);  g_tensor_arena = nullptr;
        return -5;
    }

    // 获取输入输出张量
    g_input_tensor = g_interpreter->input(0);
    g_output_tensor = g_interpreter->output(0);

    // ================================================================
    // 类型检测：自动适配 float32 / int8 模型
    // ================================================================
    TfLiteType input_type = g_input_tensor->type;
    TfLiteType output_type = g_output_tensor->type;

    const char* type_name = "Unknown";
    if (input_type == kTfLiteFloat32) {
        g_is_float_model = true;
        type_name = "float32";
    } else if (input_type == kTfLiteInt8) {
        g_is_float_model = false;
        type_name = "int8";
    } else {
        printf("❌ [识别] 不支持的张量类型：input=%d, output=%d\n",
               (int)input_type, (int)output_type);
        delete g_interpreter;  g_interpreter = nullptr;
        free(g_tensor_arena);  g_tensor_arena = nullptr;
        return -7;
    }

    // 读取量化参数（仅 int8 模型需要）
    if (!g_is_float_model) {
        g_input_scale = g_input_tensor->params.scale;
        g_input_zero_point = g_input_tensor->params.zero_point;
        g_output_scale = g_output_tensor->params.scale;
        g_output_zero_point = g_output_tensor->params.zero_point;
    }

    g_model_initialized = true;
    printf("✅ [识别] TFLM %s 模型初始化成功！\n", type_name);
    printf("   [识别] 输入维度：%dx%dx%d\n",
           g_input_tensor->dims->data[1],
           g_input_tensor->dims->data[2],
           g_input_tensor->dims->data[3]);
    printf("   [识别] 实际使用内存：%zu bytes\n", g_interpreter->arena_used_bytes());
    if (!g_is_float_model) {
        printf("   [识别] 输入量化：scale=%.6f, zero=%d\n", g_input_scale, g_input_zero_point);
    }
    printf("   [识别] 预处理：%s, 值域[0,255]\n",
           PRE_CHANNEL_ORDER == 0 ? "RGB" : "BGR");
    printf("   [识别] 等待红色触发区（12cm×5cm）...\n");
    return 0;
}

// ==================== 模型推理（使用ROI数据，自动适配float32/int8）====================
// 输入：RGB888格式的ROI数据（roi_size × roi_size × 3），已缩放到模型输入尺寸
// 输出：三个类别的置信度（0-100%）
static int model_inference_roi(const uint8_t* roi_rgb888, int roi_size,
                                float* supply_conf, float* vehicle_conf, float* weapon_conf)
{
    if (!g_model_initialized || !g_input_tensor || !g_output_tensor) return -1;
    if (!roi_rgb888 || !supply_conf || !vehicle_conf || !weapon_conf) return -2;

    // ---- 填充输入张量 ----
    for (int y = 0; y < MODEL_INPUT_HEIGHT; y++) {
        for (int x = 0; x < MODEL_INPUT_WIDTH; x++) {
            int src_idx = y * roi_size * 3 + x * 3;

            uint8_t r = roi_rgb888[src_idx + 0];
            uint8_t g = roi_rgb888[src_idx + 1];
            uint8_t b = roi_rgb888[src_idx + 2];

            // 归一化到 [0, 1]
            float r_norm = r / 255.0f;
            float g_norm = g / 255.0f;
            float b_norm = b / 255.0f;

            int base_idx = y * MODEL_INPUT_WIDTH * 3 + x * 3;

            if (g_is_float_model) {
                // float32 模型：直接填归一化浮点值
                float* input_data = g_input_tensor->data.f;
                input_data[base_idx + 0] = r_norm;
                input_data[base_idx + 1] = g_norm;
                input_data[base_idx + 2] = b_norm;
            } else {
                // int8 模型：量化后填入
                int8_t* input_data = g_input_tensor->data.int8;
                input_data[base_idx + 0] = (int8_t)(r_norm / g_input_scale + g_input_zero_point);
                input_data[base_idx + 1] = (int8_t)(g_norm / g_input_scale + g_input_zero_point);
                input_data[base_idx + 2] = (int8_t)(b_norm / g_input_scale + g_input_zero_point);
            }
        }
    }

    // ---- 执行推理 ----
    TfLiteStatus status = g_interpreter->Invoke();
    if (status != kTfLiteOk) return -3;

    // ---- 解析输出 ----
    float logits[3];
    if (g_is_float_model) {
        // float32 模型：读取输出（可能是 logits 或已 softmax 的结果）
        const float* output_data = g_output_tensor->data.f;
        logits[0] = output_data[0];
        logits[1] = output_data[1];
        logits[2] = output_data[2];
    } else {
        // int8 模型：反量化
        const int8_t* output_data = g_output_tensor->data.int8;
        logits[0] = (output_data[0] - g_output_zero_point) * g_output_scale;
        logits[1] = (output_data[1] - g_output_zero_point) * g_output_scale;
        logits[2] = (output_data[2] - g_output_zero_point) * g_output_scale;
    }

    // ---- 统一 Softmax + 百分比（兼容 logits 和概率两种输出）----
    // 使用数值稳定的 softmax：如果已是 softmax 概率，再 softmax 不改变排序
    float max_logit = logits[0];
    if (logits[1] > max_logit) max_logit = logits[1];
    if (logits[2] > max_logit) max_logit = logits[2];

    float exp_sum = expf(logits[0] - max_logit)
                  + expf(logits[1] - max_logit)
                  + expf(logits[2] - max_logit);

    *supply_conf  = expf(logits[0] - max_logit) / exp_sum * 100.0f;
    *vehicle_conf = expf(logits[1] - max_logit) / exp_sum * 100.0f;
    *weapon_conf  = expf(logits[2] - max_logit) / exp_sum * 100.0f;

    return 0;
}
// ==================== 全图模型推理（自动适配 float32/int8）====================
int model_inference(const void* image_data, int image_width, int image_height,
                    float* supply_conf, float* vehicle_conf, float* weapon_conf)
{
    if (!g_model_initialized || !g_input_tensor || !g_output_tensor) return -1;
    if (!image_data || !supply_conf || !vehicle_conf || !weapon_conf) return -2;

    const uint16_t* rgb565_img = (const uint16_t*)image_data;

    float x_ratio = (float)image_width / MODEL_INPUT_WIDTH;
    float y_ratio = (float)image_height / MODEL_INPUT_HEIGHT;

    // ---- 填充输入张量 ----
    for (int y = 0; y < MODEL_INPUT_HEIGHT; y++) {
        for (int x = 0; x < MODEL_INPUT_WIDTH; x++) {
            int src_x = (int)(x * x_ratio);
            int src_y = (int)(y * y_ratio);
            uint16_t pixel = rgb565_img[src_y * image_width + src_x];

            // RGB565 → RGB888
            uint8_t r5 = (pixel >> 11) & 0x1F;
            uint8_t g6 = (pixel >> 5)  & 0x3F;
            uint8_t b5 = pixel & 0x1F;
            uint8_t r = (r5 << 3) | (r5 >> 2);
            uint8_t g = (g6 << 2) | (g6 >> 4);
            uint8_t b = (b5 << 3) | (b5 >> 2);

            // 归一化（匹配 train.py 的 image_dataset_from_directory 设置）
            #if PRE_NORM_RANGE == 0
                float c0 = r / 255.0f;          // [0, 1]
                float c1 = g / 255.0f;
                float c2 = b / 255.0f;
            #elif PRE_NORM_RANGE == 1
                float c0 = r / 127.5f - 1.0f;   // [-1, 1]
                float c1 = g / 127.5f - 1.0f;
                float c2 = b / 127.5f - 1.0f;
            #else
                float c0 = (float)r;             // [0, 255] 原始值
                float c1 = (float)g;
                float c2 = (float)b;
            #endif

            int base_idx = y * MODEL_INPUT_WIDTH * 3 + x * 3;

            // 通道顺序
            #if PRE_CHANNEL_ORDER == 0
                // RGB (Keras load_img / tf.keras 默认)
                float ch0 = c0, ch1 = c1, ch2 = c2;
            #else
                // BGR (OpenCV imread 默认)
                float ch0 = c2, ch1 = c1, ch2 = c0;
            #endif

            if (g_is_float_model) {
                float* input_data = g_input_tensor->data.f;
                input_data[base_idx + 0] = ch0;
                input_data[base_idx + 1] = ch1;
                input_data[base_idx + 2] = ch2;
            } else {
                int8_t* input_data = g_input_tensor->data.int8;
                input_data[base_idx + 0] = (int8_t)(ch0 / g_input_scale + g_input_zero_point);
                input_data[base_idx + 1] = (int8_t)(ch1 / g_input_scale + g_input_zero_point);
                input_data[base_idx + 2] = (int8_t)(ch2 / g_input_scale + g_input_zero_point);
            }
        }
    }

    // ---- 执行推理 ----
    TfLiteStatus status = g_interpreter->Invoke();
    if (status != kTfLiteOk) return -3;

    // ---- 解析输出 ----
    float raw[3];
    if (g_is_float_model) {
        const float* output_data = g_output_tensor->data.f;
        raw[0] = output_data[0];
        raw[1] = output_data[1];
        raw[2] = output_data[2];
    } else {
        const int8_t* output_data = g_output_tensor->data.int8;
        raw[0] = (output_data[0] - g_output_zero_point) * g_output_scale;
        raw[1] = (output_data[1] - g_output_zero_point) * g_output_scale;
        raw[2] = (output_data[2] - g_output_zero_point) * g_output_scale;
    }

    // ---- 智能 Softmax：检测模型是否已经输出概率 ----
    float raw_sum = raw[0] + raw[1] + raw[2];
    bool already_softmax = (raw_sum > 0.9f && raw_sum < 1.1f)
                        && (raw[0] >= 0.0f && raw[0] <= 1.0f)
                        && (raw[1] >= 0.0f && raw[1] <= 1.0f)
                        && (raw[2] >= 0.0f && raw[2] <= 1.0f);

    static int softmax_diag = 0;
    if (softmax_diag < 3) {
        printf("🔬 [诊断] 原始输出: [%.4f, %.4f, %.4f] sum=%.4f → %s\n",
               raw[0], raw[1], raw[2], raw_sum,
               already_softmax ? "已是概率，跳过Softmax" : "是logits，执行Softmax");
        softmax_diag++;
    }

    if (already_softmax) {
        // 模型输出已是 Softmax 概率，直接 ×100 得百分比
        *supply_conf  = raw[0] * 100.0f;
        *vehicle_conf = raw[1] * 100.0f;
        *weapon_conf  = raw[2] * 100.0f;
    } else {
        // 模型输出是 logits，需要 Softmax
        float max_raw = raw[0];
        if (raw[1] > max_raw) max_raw = raw[1];
        if (raw[2] > max_raw) max_raw = raw[2];
        float exp_sum = expf(raw[0] - max_raw)
                      + expf(raw[1] - max_raw)
                      + expf(raw[2] - max_raw);
        *supply_conf  = expf(raw[0] - max_raw) / exp_sum * 100.0f;
        *vehicle_conf = expf(raw[1] - max_raw) / exp_sum * 100.0f;
        *weapon_conf  = expf(raw[2] - max_raw) / exp_sum * 100.0f;
    }

    return 0;
}

// ==================== 在屏幕上绘制红色矩形检测结果 ====================
static void draw_red_rect_debug(const RedRect* rect)
{
    if (!rect || rect->w <= 0) return;

    // 绘制红色矩形边界框（绿色边框表示检测到）
    uint16_t color = uesr_GREEN;

    // 上边和下边
    for (int x = rect->x; x <= rect->x + rect->w && x < IMAGE_W; x++) {
        ips200.draw_point(x, rect->y, color);
        if (rect->y + rect->h < IMAGE_H) {
            ips200.draw_point(x, rect->y + rect->h, color);
        }
    }
    // 左边和右边
    for (int y = rect->y; y <= rect->y + rect->h && y < IMAGE_H; y++) {
        ips200.draw_point(rect->x, y, color);
        if (rect->x + rect->w < IMAGE_W) {
            ips200.draw_point(rect->x + rect->w, y, color);
        }
    }

    // 绘制中心十字
    int cx = rect->center_x;
    int cy = rect->center_y;
    ips200.draw_point(cx - 3, cy, uesr_RED);
    ips200.draw_point(cx + 3, cy, uesr_RED);
    ips200.draw_point(cx, cy - 3, uesr_RED);
    ips200.draw_point(cx, cy + 3, uesr_RED);
}

// ==================== 在屏幕上绘制识别板ROI ====================
static void draw_board_roi_debug(const RedRect* red_rect)
{
    if (!red_rect || red_rect->w <= 0) return;

    int roi_h = red_rect->w;
    if (roi_h > IMAGE_H / 2) roi_h = IMAGE_H / 2;

    int roi_y_start = red_rect->y - roi_h - 5;
    if (roi_y_start < 0) roi_y_start = 0;

    int roi_x_start = red_rect->x - BOARD_ROI_EXTEND_SIDE;
    if (roi_x_start < 0) roi_x_start = 0;
    int roi_x_end = red_rect->x + red_rect->w + BOARD_ROI_EXTEND_SIDE;
    if (roi_x_end >= IMAGE_W) roi_x_end = IMAGE_W - 1;

    uint16_t color = uesr_BLUE;
    // 上边
    for (int x = roi_x_start; x <= roi_x_end && x < IMAGE_W; x++) {
        ips200.draw_point(x, roi_y_start, color);
    }
    // 左边
    for (int y = roi_y_start; y <= red_rect->y && y < IMAGE_H; y++) {
        ips200.draw_point(roi_x_start, y, color);
    }
    // 右边
    for (int y = roi_y_start; y <= red_rect->y && y < IMAGE_H; y++) {
        ips200.draw_point(roi_x_end, y, color);
    }
}

// ==================== 分类结果字符串生成 ====================
static void get_category_string(ImageCategory category, float max_conf,
                                 char* result_str, int str_size)
{
    const char* cat_name = "未知";
    switch (category) {
        case CATEGORY_SUPPLY:  cat_name = "物资";   break;
        case CATEGORY_VEHICLE: cat_name = "交通工具"; break;
        case CATEGORY_WEAPON:  cat_name = "武器";   break;
        default:               cat_name = "未知";   break;
    }

    if (max_conf >= 90.0f) {
        snprintf(result_str, str_size, "%s（识别成功 %.1f%%）", cat_name, max_conf);
    } else if (max_conf >= 60.0f) {
        snprintf(result_str, str_size, "%s（置信度 %.1f%%）", cat_name, max_conf);
    } else {
        snprintf(result_str, str_size, "%s（置信度不足 %.1f%%）", cat_name, max_conf);
    }
}

// ==================== 主识别流程 ====================
void run_recognition()
{
    // ---- 一次性模型初始化 ----
    static bool init_attempted = false;
    if (!init_attempted) {
        if (model_init(nullptr) != 0) {
            printf("❌ [识别] 模型初始化最终失败，关闭识别功能\n");
            init_attempted = true;
            return;
        }
        init_attempted = true;
    }
    if (!g_model_initialized) return;

    // ---- 获取彩色图像 ----
    const uint16_t* image_data = uvc_dev.get_rgb_image_ptr();
    if (image_data == nullptr) return;

    // ---- 状态机 ----
    switch (g_detect_state)
    {
        // =====================================================
        // 状态0：等待红色触发区（12cm × 5cm 红色矩形）
        // =====================================================
        case STATE_WAIT_RED:
        {
            // ---- 调试：每N帧打印一次红色像素统计 ----
            static int debug_frame = 0;
            debug_frame++;

            // 快速统计：扫描全图红色像素占比
            if (debug_frame % RED_DEBUG_INTERVAL == 0) {
                int total_scan = 0, red_count = 0;
                int max_r = 0, min_r = 31;
                // 记录"最红"像素的RGB值（R最高且G+B最低的）
                uint8_t best_r = 0, best_g = 0, best_b = 0;
                int best_score = -1;  // score = r*2 - g - b，越大越红

                for (int y = RED_SEARCH_Y_START; y < RED_SEARCH_Y_END; y++) {
                    for (int x = 0; x < IMAGE_W; x++) {
                        total_scan++;
                        uint16_t px = image_data[y * IMAGE_W + x];
                        uint8_t r = (px >> 11) & 0x1F;
                        uint8_t g = (px >> 5)  & 0x3F;
                        uint8_t b = px & 0x1F;
                        if (r > max_r) max_r = r;
                        if (r < min_r) min_r = r;

                        if (is_red_pixel(px)) red_count++;

                        // 找最红的像素
                        int score = (int)r * 2 - (int)g - (int)b;
                        if (score > best_score) {
                            best_score = score;
                            best_r = r; best_g = g; best_b = b;
                        }
                    }
                }
                float ratio = (float)red_count / total_scan * 100.0f;
                printf("🔍 [调试] 红色像素: %d/%d (%.1f%%) | R范围:[%d,%d]\n",
                       red_count, total_scan, ratio, min_r, max_r);
                printf("   [调试] 最红像素 RGB565: R=%d G=%d B=%d (score=%d)\n",
                       best_r, best_g, best_b, best_score);
                printf("   [调试] 阈值: R>=%d G<=%d B<=%d R>G*%.1f R>B*%.1f\n",
                       RED_R_MIN, RED_G_MAX, RED_B_MAX,
                       RED_R_G_RATIO, RED_R_B_RATIO);

                // 简单阈值检测作为参考
                if (ratio >= 5.0f) {
                    printf("   [调试] ⚠️ 红色占比 %.1f%% >= 5%%，但连通域未匹配矩形\n", ratio);
                    printf("   [调试] 可能原因：尺寸/宽高比/填充率不满足，检查 RED_RECT_* 参数\n");
                } else if (ratio < 1.0f) {
                    printf("   [调试] ⚠️ 红色占比不足1%%，阈值可能太严或红色区域不在视野内\n");
                }
            }

            // 检测红色矩形（连通域分析）
            static RedRect detected_rects[4];
            int rect_count = detect_red_rectangles(image_data, IMAGE_W, IMAGE_H,
                                                    detected_rects, 4);

            // ---- 简单回退：如果连通域没找到，检查底部ROI红色占比 ----
            bool red_triggered = false;
            if (rect_count == 0) {
                // 快速ROI扫描：图像底部40行 × 中间80列
                int roi_x = 40, roi_w = 80, roi_y = IMAGE_H - 45, roi_h = 40;
                int roi_red = 0, roi_total = roi_w * roi_h;
                for (int y = roi_y; y < roi_y + roi_h && y < IMAGE_H; y++) {
                    for (int x = roi_x; x < roi_x + roi_w && x < IMAGE_W; x++) {
                        if (is_red_pixel(image_data[y * IMAGE_W + x])) roi_red++;
                    }
                }
                float roi_ratio = (float)roi_red / roi_total;
                if (roi_ratio >= 0.08f) {  // 底部区域红色占比 >= 8%
                    red_triggered = true;
                    // 构造虚拟矩形用于后续ROI提取
                    g_last_red_rect.x = roi_x;
                    g_last_red_rect.y = roi_y;
                    g_last_red_rect.w = roi_w;
                    g_last_red_rect.h = roi_h;
                    g_last_red_rect.center_x = roi_x + roi_w / 2;
                    g_last_red_rect.center_y = roi_y + roi_h / 2;
                    g_last_red_rect.aspect = (float)roi_w / roi_h;
                    g_last_red_rect.fill_ratio = roi_ratio;
                    if (debug_frame % RED_DEBUG_INTERVAL == 0) {
                        printf("   [调试] 回退模式触发：底部ROI红色占比=%.1f%%\n", roi_ratio * 100.0f);
                    }
                }
            }

            if (rect_count > 0 || red_triggered) {
                // 找到红色矩形 → 记录并消抖
                if (rect_count > 0) {
                    g_last_red_rect = detected_rects[0];
                }

                g_red_debounce_cnt++;
                if (g_red_debounce_cnt == 1) {
                    printf("🔴 [识别] 检测到红色区域：(%d,%d) %dx%d "
                           "填充率=%.1f%% 宽高比=%.1f\n",
                           g_last_red_rect.x, g_last_red_rect.y,
                           g_last_red_rect.w, g_last_red_rect.h,
                           g_last_red_rect.fill_ratio * 100.0f,
                           g_last_red_rect.aspect);
                }

                // 绘制检测框
                draw_red_rect_debug(&g_last_red_rect);

                if (g_red_debounce_cnt >= RED_DEBOUNCE_FRAMES) {
                    printf("✅ [识别] 红色矩形确认！12cm×5cm触发区已识别\n");
                    printf("   [识别] 矩形位置：(%d,%d) 尺寸：%d×%d 宽高比：%.2f\n",
                           g_last_red_rect.x, g_last_red_rect.y,
                           g_last_red_rect.w, g_last_red_rect.h,
                           g_last_red_rect.aspect);
                    g_detect_state = STATE_RED_CONFIRMED;
                    g_red_debounce_cnt = 0;
                }
            } else {
                // 未检测到 → 重置消抖计数
                if (g_red_debounce_cnt > 0) {
                    g_red_debounce_cnt = 0;
                }
            }
            break;
        }

        // =====================================================
        // 状态1：红色矩形已确认，提取识别板ROI
        // =====================================================
        case STATE_RED_CONFIRMED:
        {
            // 继续跟踪红色矩形（可能因车辆移动而位移）
            static RedRect tracked_rects[4];
            int rect_count = detect_red_rectangles(image_data, IMAGE_W, IMAGE_H,
                                                    tracked_rects, 4);
            if (rect_count > 0) {
                g_last_red_rect = tracked_rects[0];
                draw_red_rect_debug(&g_last_red_rect);
            }

            // 绘制识别板ROI预览
            draw_board_roi_debug(&g_last_red_rect);

            // 进入接近目标板阶段
            g_detect_state = STATE_APPROACH_TARGET;
            g_target_delay_cnt = 0;
            printf("🎯 [识别] 正在接近目标板，延迟%d帧后分类...\n", TARGET_DELAY_FRAMES);
            break;
        }

        // =====================================================
        // 状态2：接近目标板（延迟等待车辆靠近识别板）
        // =====================================================
        case STATE_APPROACH_TARGET:
        {
            g_target_delay_cnt++;

            // 继续跟踪红色矩形
            static RedRect tracked_rects2[4];
            int rect_count = detect_red_rectangles(image_data, IMAGE_W, IMAGE_H,
                                                    tracked_rects2, 4);
            if (rect_count > 0) {
                g_last_red_rect = tracked_rects2[0];
                draw_red_rect_debug(&g_last_red_rect);
            }
            draw_board_roi_debug(&g_last_red_rect);

            if (g_target_delay_cnt >= TARGET_DELAY_FRAMES) {
                g_detect_state = STATE_CLASSIFY;
                printf("🔍 [识别] 开始分类识别板...\n");
            }
            break;
        }

        // =====================================================
        // 状态3：执行分类（全图推理 + 多帧投票）
        // =====================================================
        case STATE_CLASSIFY:
        {
            // === 策略：直接用全图推理，不做ROI裁剪 ===
            // 模型训练时用的是全帧 160×120 → 40×40，ROI裁剪反而丢失信息
            // 识别板通常在图像上半部，全图推理能自动定位

            #define VOTE_FRAMES  5   // 多帧投票帧数
            static float vote_supply[VOTE_FRAMES];
            static float vote_vehicle[VOTE_FRAMES];
            static float vote_weapon[VOTE_FRAMES];
            static int vote_idx = 0;

            // 对当前帧推理
            float s = 0.0f, v = 0.0f, w = 0.0f;
            int ret = model_inference(image_data, IMAGE_W, IMAGE_H, &s, &v, &w);
            if (ret == 0) {
                vote_supply[vote_idx]  = s;
                vote_vehicle[vote_idx] = v;
                vote_weapon[vote_idx]  = w;
                vote_idx++;
            } else {
                printf("⚠️ [识别] 推理失败(帧%d)，跳过\n", vote_idx);
            }

            // 收集够VOTE_FRAMES帧后取平均
            if (vote_idx >= VOTE_FRAMES) {
                float supply_conf = 0.0f, vehicle_conf = 0.0f, weapon_conf = 0.0f;
                for (int i = 0; i < VOTE_FRAMES; i++) {
                    supply_conf  += vote_supply[i];
                    vehicle_conf += vote_vehicle[i];
                    weapon_conf  += vote_weapon[i];
                }
                supply_conf  /= VOTE_FRAMES;
                vehicle_conf /= VOTE_FRAMES;
                weapon_conf  /= VOTE_FRAMES;

                // 保存结果
                g_last_supply_conf  = supply_conf;
                g_last_vehicle_conf = vehicle_conf;
                g_last_weapon_conf  = weapon_conf;

                // 判定类别
                ImageCategory detect_result = CATEGORY_UNKNOWN;
                float max_conf = 0.0f;
                if (supply_conf >= vehicle_conf && supply_conf >= weapon_conf) {
                    max_conf = supply_conf;
                    detect_result = CATEGORY_SUPPLY;
                } else if (vehicle_conf >= supply_conf && vehicle_conf >= weapon_conf) {
                    max_conf = vehicle_conf;
                    detect_result = CATEGORY_VEHICLE;
                } else {
                    max_conf = weapon_conf;
                    detect_result = CATEGORY_WEAPON;
                }
                g_last_category = detect_result;

                // 输出结果
                char result_str[64] = {0};
                get_category_string(detect_result, max_conf, result_str, sizeof(result_str));

                printf("╔══════════════════════════════════╗\n");
                printf("║  🎯 识别结果：%-18s ║\n", result_str);
                printf("║  物资置信度：    %6.2f%%          ║\n", supply_conf);
                printf("║  交通工具置信度：%6.2f%%          ║\n", vehicle_conf);
                printf("║  武器置信度：    %6.2f%%          ║\n", weapon_conf);
                printf("╚══════════════════════════════════╝\n");

                // 入冷却期，重置投票计数器
                vote_idx = 0;
                g_detect_state = STATE_COOLDOWN;
                g_cooldown_cnt = 0;
                printf("⏳ [识别] 进入冷却期（%d帧）...\n", COOLDOWN_FRAMES);
            }
            // else: 继续收集投票帧，保持在 STATE_CLASSIFY
            break;
        }

        // =====================================================
        // 状态4：冷却期（避免重复触发）
        // =====================================================
        case STATE_COOLDOWN:
        {
            g_cooldown_cnt++;
            if (g_cooldown_cnt >= COOLDOWN_FRAMES) {
                g_detect_state = STATE_WAIT_RED;
                printf("🔄 [识别] 冷却结束，回到等待红色触发区状态\n");
            }
            break;
        }

        default:
            g_detect_state = STATE_WAIT_RED;
            break;
    }
}

// ==================== 获取最近一次识别结果（供其他模块查询）====================
void get_last_recognition_result(float* supply_conf, float* vehicle_conf,
                                  float* weapon_conf, int* category)
{
    if (supply_conf)  *supply_conf  = g_last_supply_conf;
    if (vehicle_conf) *vehicle_conf = g_last_vehicle_conf;
    if (weapon_conf)  *weapon_conf  = g_last_weapon_conf;
    if (category)     *category     = (int)g_last_category;
}
