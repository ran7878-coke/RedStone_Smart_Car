#ifndef _RECOGNITION_HPP
#define _RECOGNITION_HPP

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 模型参数
#define MODEL_INPUT_WIDTH   40
#define MODEL_INPUT_HEIGHT  40
#define MODEL_INPUT_CHANNELS 3
#define TENSOR_ARENA_SIZE   (1 * 1024 * 1024)   // 1MB，float32模型对小网络足够

// ================================================================
// ⚠️ 预处理配置：必须和 train.py 保持一致！
// ================================================================
// CHANNEL_ORDER:  0=RGB (Keras load_img/image_dataset_from_directory默认)
//                 1=BGR (OpenCV imread默认)
// NORM_RANGE:     0=[0,1]   (rescale=1./255)
//                 1=[-1,1]  (rescale=1./127.5, offset=-1)
//                 2=[0,255] (无rescale, image_dataset_from_directory默认)
// 改了这里后重新编译即可，无需重新训练
#define PRE_CHANNEL_ORDER   0    // 0=RGB, 1=BGR
#define PRE_NORM_RANGE      2    // 0=[0,1], 1=[-1,1], 2=[0,255]

// 接口函数
int model_init(const char* model_path);
int model_inference(const void* image_data, int width, int height,
                    float* supply_conf, float* vehicle_conf, float* weapon_conf);
void run_recognition();
void get_last_recognition_result(float* supply_conf, float* vehicle_conf,
                                  float* weapon_conf, int* category);

#ifdef __cplusplus
}
#endif

#endif  // ✅ 新增：闭合#ifndef