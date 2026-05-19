#include "aitask.h"
#include <chrono>

// ======================== 可配置参数宏定义 ========================
#define MODEL_INPUT_WIDTH         48      // 模型输入图像宽度
#define MODEL_INPUT_HEIGHT        48      // 模型输入图像高度
#define MODEL_INPUT_CHANNEL       3       // 模型输入图像通道数（RGB=3）
#define MODEL_OUTPUT_CLASS_NUM    3       // 模型输出类别数
#define TFLITE_OP_RESOLVER_MAX_NUM 20     // 算子解析器最大支持算子数
#define TENSOR_ARENA_SIZE         (256 * 1024)  // 张量空间大小（单位：字节）

const char* class_labels[] = {"weapon","materials","traffic"}; 

uint16_t img_ai[AI_H][AI_W] = {0};

// ======================== 静态全局变量 (TFLite核心组件) ========================
// 必须设为静态，确保生命周期贯穿整个程序运行过程，而不是随函数退出而销毁
static const tflite::Model* g_model = nullptr;
static tflite::MicroInterpreter* g_interpreter = nullptr;
// 内存池：16字节对齐，防止部分单片机架构下出现内存访问错误
static uint8_t g_tensor_arena[TENSOR_ARENA_SIZE] __attribute__((aligned(16))); 

// ======================== 日志回调函数 ========================
void debug_log_printf_callback(const char* s) 
{
    if (s == NULL)  return;
    printf("%s", s);
}

// ======================== 获取图像 ========================
void ai_fetch_new_frame(void) {
    ai_uvc_get_rgb_image(img_ai[0]); 
}

// ======================== 1. AI 初始化函数 (仅在程序启动时调用一次) ========================
int ai_init(void) {
    printf("===== 开始初始化 AI 模型 =====\n");
    
    // 1. 重定向TFLM的MicroPrintf日志输出
    RegisterDebugLogCallback(debug_log_printf_callback);

    // 2. tflite库初始化
    tflite::InitializeTarget();

    // 3. 导入模型
    g_model = ::tflite::GetModel(loong_cnn_model_simple_tflite);
    TFLITE_CHECK_EQ(g_model->version(), TFLITE_SCHEMA_VERSION);

    // 4. 构建算子解析器 (使用静态变量)
    static tflite::MicroMutableOpResolver<TFLITE_OP_RESOLVER_MAX_NUM> resolver;
    TF_LITE_ENSURE_STATUS(resolver.AddConv2D());
    TF_LITE_ENSURE_STATUS(resolver.AddDepthwiseConv2D());
    TF_LITE_ENSURE_STATUS(resolver.AddMaxPool2D());
    TF_LITE_ENSURE_STATUS(resolver.AddFullyConnected());
    TF_LITE_ENSURE_STATUS(resolver.AddRelu6());
    TF_LITE_ENSURE_STATUS(resolver.AddSoftmax());
    TF_LITE_ENSURE_STATUS(resolver.AddReshape());
    TF_LITE_ENSURE_STATUS(resolver.AddShape());
    TF_LITE_ENSURE_STATUS(resolver.AddQuantize());
    TF_LITE_ENSURE_STATUS(resolver.AddDequantize());
    TF_LITE_ENSURE_STATUS(resolver.AddCast());
    TF_LITE_ENSURE_STATUS(resolver.AddSqueeze());
    TF_LITE_ENSURE_STATUS(resolver.AddExpandDims());
    TF_LITE_ENSURE_STATUS(resolver.AddConcatenation());
    TF_LITE_ENSURE_STATUS(resolver.AddTranspose());
    TF_LITE_ENSURE_STATUS(resolver.AddStridedSlice());
    TF_LITE_ENSURE_STATUS(resolver.AddPack());
    TF_LITE_ENSURE_STATUS(resolver.AddLogistic());
    TF_LITE_ENSURE_STATUS(resolver.AddMean());
    TF_LITE_ENSURE_STATUS(resolver.AddAdd());

    // 5. 创建模型解析器并分配张量空间 (使用静态对象避免使用new/malloc)
    static tflite::MicroInterpreter static_interpreter(
        g_model, resolver, g_tensor_arena, TENSOR_ARENA_SIZE);
    
    // 将指针指向静态对象，供其他函数调用
    g_interpreter = &static_interpreter;
    
    // 分配内存
    g_interpreter->AllocateTensors();

    printf("模型输入数据类型: %d (1=Float32, 9=INT8)\n", g_interpreter->input(0)->type);
    printf("===== AI 模型初始化完成 =====\n");
    return 0;
}

// ======================== 2. AI 单次推理任务 (在主循环/定时器中定期调用) ========================
void ai_extract(void) {
    // 安全检查：防止未初始化就被调用
    if (g_interpreter == nullptr) {
        return; 
    }

    static int frame_count = 0;   // 静态帧计数器
    frame_count++;

    // 记录总耗时起点
    auto frame_start_time = std::chrono::high_resolution_clock::now();

    // 采集一帧图像
    ai_fetch_new_frame(); 

    // 计算缩放步长 (最近邻插值)
    const float scale_x = (float)AI_W / MODEL_INPUT_WIDTH;
    const float scale_y = (float)AI_H / MODEL_INPUT_HEIGHT;

    // ======================== 图像预处理 ========================
    float* input_data = g_interpreter->input(0)->data.f;
    int input_index = 0;

    for (int y = 0; y < MODEL_INPUT_HEIGHT; ++y) {
        for (int x = 0; x < MODEL_INPUT_WIDTH; ++x) {
            int src_x = (int)(x * scale_x);
            int src_y = (int)(y * scale_y);

            if(src_x >= AI_W) src_x = AI_W - 1;
            if(src_y >= AI_H) src_y = AI_H - 1;

            uint16_t pixel = img_ai[src_y][src_x];

            // 提取RGB (带低位补全，色彩还原更准一点点，不需要也可改回你的原版)
            uint8_t r = ((pixel & 0xF800) >> 8) | ((pixel & 0xF800) >> 13);
            uint8_t g = ((pixel & 0x07E0) >> 3) | ((pixel & 0x07E0) >> 9);
            uint8_t b = ((pixel & 0x001F) << 3) | ((pixel & 0x001F) >> 2);
            
            //是否归一化
            input_data[input_index++] = (float)r;
            input_data[input_index++] = (float)g;
            input_data[input_index++] = (float)b;
            // input_data[input_index++] = (float)r / 255.0f;
            // input_data[input_index++] = (float)g / 255.0f;
            // input_data[input_index++] = (float)b / 255.0f;
        }
    }

    // ======================== 模型推理 ========================
    auto infer_start_time = std::chrono::high_resolution_clock::now();
    
    g_interpreter->Invoke();
    
    auto infer_end_time = std::chrono::high_resolution_clock::now();

    // ======================== 模型输出解析 ========================
    float* output_data = g_interpreter->output(0)->data.f;
    float max_v = 0.0f;
    int max_i = 0;
    
    for (int n = 0; n < MODEL_OUTPUT_CLASS_NUM; n++) {
        float v = output_data[n];
        if (v > max_v) {
            max_v = v;
            max_i = n;
        }
    }

    // ======================== 耗时计算与打印 ========================
    auto infer_time_us = std::chrono::duration_cast<std::chrono::microseconds>(infer_end_time - infer_start_time).count();
    auto frame_end_time = std::chrono::high_resolution_clock::now();
    auto frame_time_us = std::chrono::duration_cast<std::chrono::microseconds>(frame_end_time - frame_start_time).count();
    float fps = 1000000.0f / (float)frame_time_us;

    // 为了防止刷屏太快，可以限制每10帧打印一次
        printf("[Frame %d] 结果: %s | 置信度: %.2f%% | 推理耗时: %.2f ms | FPS: %.1f\n",
               frame_count, 
               class_labels[max_i], 
               max_v * 100.0f,               
               infer_time_us / 1000.0f,      
               fps);
}