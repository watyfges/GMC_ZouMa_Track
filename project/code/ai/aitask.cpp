#include "aitask.h"
#include "aiuvc.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "net.h"

extern std::chrono::time_point<std::chrono::high_resolution_clock> rao_start,rao_end;

static constexpr int MODEL_INPUT_WIDTH = 40;
static constexpr int MODEL_INPUT_HEIGHT = 40;
static constexpr float AI_CONFIDENCE_THRESHOLD = 0.90f;
static constexpr int AI_IDLE_SLEEP_MS = 1;

static const char *MODEL_PARAM_CANDIDATES[] = {
    "tiny_classifier_v5_fp32.ncnn.param",
    "/home/root/tiny_classifier_v5_fp32.ncnn.param",
    "../model/tiny_classifier_v5_fp32.ncnn.param",
};

static const char *MODEL_BIN_CANDIDATES[] = {
    "tiny_classifier_v5_fp32.ncnn.bin",
    "/home/root/tiny_classifier_v5_fp32.ncnn.bin",
    "../model/tiny_classifier_v5_fp32.ncnn.bin",
};

static const std::vector<std::string> MODEL_LABELS = {
    "ambulance",
    "armoured_car",
    "binoculars",
    "gun",
    "hand_boom",
};

#define RED_H_LOW_1                    0
#define RED_H_HIGH_1                   10
#define RED_H_LOW_2                    170
#define RED_H_HIGH_2                   180
#define RED_S_MIN                      80
#define RED_V_MIN                      60
#define RED_S_MAX                      255
#define RED_V_MAX                      255
#define RED_MORPH_KERNEL_SIZE          5
#define RED_MIN_AREA_RATIO             0.0010
#define RED_MAX_AREA_RATIO             0.0800
#define RED_MARKER_ASPECT_MIN          0.20f
#define RED_MARKER_ASPECT_MAX          7.00f
#define RED_MARKER_FILL_MIN            0.05f
#define RED_QUAD_APPROX_EPSILON_RATIO  0.030
#define RED_SEARCH_TOP_RATIO           0.35f
#define RED_BOTTOM_PRIORITY_PIXELS     8
#define TARGET_SIDE_TO_MARKER_LONG_EDGE     0.70f
#define TARGET_BOTTOM_GAP_TO_MARKER_HEIGHT  0.00f
#define TARGET_CENTER_SHIFT_ALONG_LONG_EDGE 0.00f
#define TARGET_MIN_SIDE_PIXELS              18.0f

struct RedMarkerInfo
{
    std::vector<cv::Point2f> quad;
    cv::Point2f center;
    cv::Point2f long_edge_start;
    cv::Point2f long_edge_end;
    cv::Point2f long_edge_axis;
    cv::Point2f target_axis;
    float long_edge_length = 0.0f;
    float short_edge_length = 0.0f;
};

static bool file_exists(const std::string &path)
{
    std::ifstream file(path.c_str());
    return file.good();
}

static std::string first_existing_path(const char *const *paths, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        if (file_exists(paths[i])) {
            return paths[i];
        }
    }
    return paths[0];
}

static bool contains_blob_name(const std::vector<std::string> &names, const std::string &target)
{
    return std::find(names.begin(), names.end(), target) != names.end();
}

static bool parse_ncnn_param_blob_names(const std::string &param_path,
                                        std::string &input_name,
                                        std::string &output_name)
{
    std::ifstream file(param_path.c_str());
    if (!file.is_open()) {
        return false;
    }

    std::vector<std::string> produced_blobs;
    std::vector<std::string> consumed_blobs;
    std::string line;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream stream(line);
        std::vector<std::string> tokens;
        std::string token;
        while (stream >> token) {
            tokens.push_back(token);
        }

        if (tokens.size() < 4) {
            continue;
        }

        int bottom_count = 0;
        int top_count = 0;
        try {
            bottom_count = std::stoi(tokens[2]);
            top_count = std::stoi(tokens[3]);
        } catch (...) {
            continue;
        }

        const size_t first_bottom_index = 4;
        const size_t first_top_index = first_bottom_index + static_cast<size_t>(bottom_count);
        const size_t needed_size = first_top_index + static_cast<size_t>(top_count);
        if (tokens.size() < needed_size) {
            continue;
        }

        if (tokens[0] == "Input" && top_count > 0 && input_name.empty()) {
            input_name = tokens[first_top_index];
        }

        for (int i = 0; i < bottom_count; ++i) {
            const std::string &bottom = tokens[first_bottom_index + static_cast<size_t>(i)];
            consumed_blobs.push_back(bottom);
            if (input_name.empty() && !contains_blob_name(produced_blobs, bottom)) {
                input_name = bottom;
            }
        }

        for (int i = 0; i < top_count; ++i) {
            produced_blobs.push_back(tokens[first_top_index + static_cast<size_t>(i)]);
        }
    }

    for (auto it = produced_blobs.rbegin(); it != produced_blobs.rend(); ++it) {
        if (!contains_blob_name(consumed_blobs, *it)) {
            output_name = *it;
            break;
        }
    }

    if (output_name.empty() && !produced_blobs.empty()) {
        output_name = produced_blobs.back();
    }

    return !input_name.empty() || !output_name.empty();
}

class NcnnClassifier
{
public:
    bool init()
    {
        const std::string param_path = first_existing_path(
            MODEL_PARAM_CANDIDATES,
            sizeof(MODEL_PARAM_CANDIDATES) / sizeof(MODEL_PARAM_CANDIDATES[0]));
        const std::string bin_path = first_existing_path(
            MODEL_BIN_CANDIDATES,
            sizeof(MODEL_BIN_CANDIDATES) / sizeof(MODEL_BIN_CANDIDATES[0]));

        net_.opt.use_vulkan_compute = false;
        net_.opt.num_threads = 1;

        if (net_.load_param(param_path.c_str()) != 0) {
            std::printf("NCNN: 加载参数文件失败: %s\n", param_path.c_str());
            return false;
        }

        if (net_.load_model(bin_path.c_str()) != 0) {
            std::printf("NCNN: 加载模型文件失败: %s\n", bin_path.c_str());
            return false;
        }

        std::string parsed_input_name;
        std::string parsed_output_name;
        parse_ncnn_param_blob_names(param_path, parsed_input_name, parsed_output_name);
        if (!parsed_input_name.empty()) {
            input_name_ = parsed_input_name;
        }
        if (!parsed_output_name.empty()) {
            output_name_ = parsed_output_name;
        }

        initialized_ = true;
        std::printf("NCNN: 模型加载成功 (%s)\n", param_path.c_str());
        std::printf("NCNN: 输入blob=%s, 输出blob=%s\n", input_name_.c_str(), output_name_.c_str());
        return true;
    }

    bool infer(const cv::Mat &bgr_image, std::string &result, float &confidence)
    {
        if (!initialized_ || bgr_image.empty()) {
            return false;
        }

        cv::Mat resized;
        cv::resize(bgr_image, resized, cv::Size(MODEL_INPUT_WIDTH, MODEL_INPUT_HEIGHT));

        cv::Mat rgb;
        cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

        ncnn::Mat input = ncnn::Mat::from_pixels(
            rgb.data,
            ncnn::Mat::PIXEL_RGB,
            MODEL_INPUT_WIDTH,
            MODEL_INPUT_HEIGHT);

        float mean_vals[3] = {123.675f, 116.28f, 103.53f};
        float norm_vals[3] = {0.01712475f, 0.017507f, 0.01742919f};
        input.substract_mean_normalize(mean_vals, norm_vals);

        ncnn::Extractor extractor = net_.create_extractor();
        if (extractor.input(input_name_.c_str(), input) != 0) {
            return false;
        }

        ncnn::Mat logits;
        if (extractor.extract(output_name_.c_str(), logits) != 0) {
            return false;
        }

        int class_id = argmax(logits, confidence);
        if (class_id < 0) {
            return false;
        }

        if (class_id < static_cast<int>(MODEL_LABELS.size())) {
            result = MODEL_LABELS[class_id];
        } else {
            result = std::to_string(class_id);
        }
        return true;
    }

private:
    int argmax(const ncnn::Mat &logits, float &confidence)
    {
        if (logits.w <= 0) {
            return -1;
        }

        int best_index = 0;
        float best_value = -std::numeric_limits<float>::infinity();
        for (int i = 0; i < logits.w; ++i) {
            const float value = logits[i];
            if (value > best_value) {
                best_value = value;
                best_index = i;
            }
        }

        float sum = 0.0f;
        for (int i = 0; i < logits.w; ++i) {
            sum += std::exp(logits[i] - best_value);
        }

        confidence = sum > 0.0f ? 1.0f / sum : 0.0f;
        return best_index;
    }

private:
    ncnn::Net net_;
    std::string input_name_ = "in0";
    std::string output_name_ = "out0";
    bool initialized_ = false;
};

static NcnnClassifier g_classifier;
static bool g_classifier_ready = false;
static bool g_ai_worker_started = false;

struct PendingRecognition
{
    BypassDir_e dir = DIR_NONE;
    std::string result;
    float confidence = 0.0f;
    bool available = false;
};

static std::mutex g_pending_mutex;
static PendingRecognition g_pending_recognition;

static float point_distance(const cv::Point2f &a, const cv::Point2f &b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

static float cross_product(const cv::Point2f &a, const cv::Point2f &b)
{
    return a.x * b.y - a.y * b.x;
}

static cv::Point2f normalize_vector(const cv::Point2f &value, const cv::Point2f &fallback)
{
    const float length = std::sqrt(value.x * value.x + value.y * value.y);
    if (length < 0.001f) {
        return fallback;
    }
    return cv::Point2f(value.x / length, value.y / length);
}

static cv::Point2f upward_normal_from_edge_axis(const cv::Point2f &edge_axis)
{
    const cv::Point2f normal_a(-edge_axis.y, edge_axis.x);
    const cv::Point2f normal_b(edge_axis.y, -edge_axis.x);
    return normal_a.y < normal_b.y ? normal_a : normal_b;
}

static cv::Point2f intersect_lines_or_average(const cv::Point2f &a0,
                                              const cv::Point2f &a1,
                                              const cv::Point2f &b0,
                                              const cv::Point2f &b1)
{
    const cv::Point2f r = a1 - a0;
    const cv::Point2f s = b1 - b0;
    const float denom = cross_product(r, s);
    if (std::fabs(denom) < 0.001f) {
        return (a0 + a1 + b0 + b1) * 0.25f;
    }
    const float t = cross_product(b0 - a0, s) / denom;
    return a0 + r * t;
}

static std::vector<cv::Point2f> order_quad_points(const std::vector<cv::Point2f> &points)
{
    std::vector<cv::Point2f> ordered = points;
    if (ordered.size() != 4) {
        return std::vector<cv::Point2f>(4);
    }

    cv::Point2f center(0.0f, 0.0f);
    for (const auto &point : ordered) {
        center += point;
    }
    center *= 0.25f;

    std::sort(ordered.begin(), ordered.end(), [center](const cv::Point2f &a, const cv::Point2f &b) {
        return std::atan2(a.y - center.y, a.x - center.x) <
               std::atan2(b.y - center.y, b.x - center.x);
    });

    int first_index = 0;
    float min_sum = ordered[0].x + ordered[0].y;
    for (int i = 1; i < 4; ++i) {
        const float sum = ordered[i].x + ordered[i].y;
        if (sum < min_sum) {
            min_sum = sum;
            first_index = i;
        }
    }

    std::rotate(ordered.begin(), ordered.begin() + first_index, ordered.end());
    return ordered;
}

static bool contour_to_quad(const std::vector<cv::Point> &contour, std::vector<cv::Point2f> &quad)
{
    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);
    const double perimeter = cv::arcLength(hull, true);
    if (perimeter < 1.0) {
        return false;
    }

    std::vector<cv::Point> approx;
    cv::approxPolyDP(hull, approx, perimeter * RED_QUAD_APPROX_EPSILON_RATIO, true);
    if (approx.size() == 4 && cv::isContourConvex(approx)) {
        std::vector<cv::Point2f> points;
        points.reserve(4);
        for (const auto &point : approx) {
            points.push_back(cv::Point2f(static_cast<float>(point.x),
                                         static_cast<float>(point.y)));
        }
        quad = order_quad_points(points);
        return true;
    }

    cv::RotatedRect rect = cv::minAreaRect(contour);
    cv::Point2f rect_points[4];
    rect.points(rect_points);
    std::vector<cv::Point2f> points(rect_points, rect_points + 4);
    quad = order_quad_points(points);
    return true;
}

static bool fill_marker_info_from_quad(const std::vector<cv::Point2f> &quad, RedMarkerInfo &marker)
{
    if (quad.size() != 4) {
        return false;
    }

    struct EdgeCandidate
    {
        cv::Point2f start;
        cv::Point2f end;
        cv::Point2f center;
        float length;
    };

    std::vector<EdgeCandidate> edges;
    edges.reserve(4);
    for (int i = 0; i < 4; ++i) {
        const cv::Point2f start = quad[i];
        const cv::Point2f end = quad[(i + 1) % 4];
        edges.push_back({start, end, (start + end) * 0.5f, point_distance(start, end)});
    }

    std::sort(edges.begin(), edges.end(), [](const EdgeCandidate &a, const EdgeCandidate &b) {
        return a.length > b.length;
    });

    const float long_edge_length = (edges[0].length + edges[1].length) * 0.5f;
    const float short_edge_length = (edges[2].length + edges[3].length) * 0.5f;
    if (long_edge_length < 1.0f || short_edge_length < 1.0f) {
        return false;
    }

    EdgeCandidate target_edge = edges[0].center.y <= edges[1].center.y ? edges[0] : edges[1];
    if (target_edge.start.x > target_edge.end.x) {
        std::swap(target_edge.start, target_edge.end);
    }

    cv::Point2f short_axis_sum(0.0f, 0.0f);
    for (int i = 2; i < 4; ++i) {
        cv::Point2f short_axis = edges[i].end - edges[i].start;
        if (short_axis.y > 0.0f) {
            short_axis = -short_axis;
        }
        short_axis_sum += normalize_vector(short_axis, cv::Point2f(0.0f, -1.0f));
    }

    marker.quad = quad;
    marker.center = intersect_lines_or_average(quad[0], quad[2], quad[1], quad[3]);
    marker.long_edge_start = target_edge.start;
    marker.long_edge_end = target_edge.end;
    marker.long_edge_axis = normalize_vector(target_edge.end - target_edge.start, cv::Point2f(1.0f, 0.0f));
    marker.target_axis = normalize_vector(short_axis_sum, upward_normal_from_edge_axis(marker.long_edge_axis));
    marker.long_edge_length = target_edge.length;
    marker.short_edge_length = short_edge_length;
    return true;
}

static bool find_red_marker(const cv::Mat &image, RedMarkerInfo &marker)
{
    if (image.empty()) {
        return false;
    }

    cv::Mat hsv, low_mask, high_mask, red_mask;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
                cv::Scalar(RED_H_LOW_1, RED_S_MIN, RED_V_MIN),
                cv::Scalar(RED_H_HIGH_1, RED_S_MAX, RED_V_MAX),
                low_mask);
    cv::inRange(hsv,
                cv::Scalar(RED_H_LOW_2, RED_S_MIN, RED_V_MIN),
                cv::Scalar(RED_H_HIGH_2, RED_S_MAX, RED_V_MAX),
                high_mask);
    cv::bitwise_or(low_mask, high_mask, red_mask);

    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(RED_MORPH_KERNEL_SIZE, RED_MORPH_KERNEL_SIZE));
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double frame_area = image.cols * image.rows;
    const double min_area = frame_area * RED_MIN_AREA_RATIO;
    const double max_area = frame_area * RED_MAX_AREA_RATIO;
    const int search_top = cvRound(image.rows * RED_SEARCH_TOP_RATIO);
    int best_bottom = -1;
    double best_area = 0.0;
    RedMarkerInfo best_marker;
    bool found = false;

    for (const auto &contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < min_area || area > max_area) {
            continue;
        }

        const cv::Rect rect = cv::boundingRect(contour);
        if (rect.width <= 0 || rect.height <= 0 || rect.y < search_top) {
            continue;
        }

        std::vector<cv::Point2f> quad;
        if (!contour_to_quad(contour, quad)) {
            continue;
        }

        RedMarkerInfo current_marker;
        if (!fill_marker_info_from_quad(quad, current_marker)) {
            continue;
        }

        const float aspect = current_marker.long_edge_length / current_marker.short_edge_length;
        const float quad_area = static_cast<float>(std::fabs(cv::contourArea(quad)));
        const float fill_ratio = quad_area > 1.0f ? static_cast<float>(area) / quad_area : 0.0f;
        if (aspect < RED_MARKER_ASPECT_MIN ||
            aspect > RED_MARKER_ASPECT_MAX ||
            fill_ratio < RED_MARKER_FILL_MIN) {
            continue;
        }

        const int bottom = rect.y + rect.height;
        const bool lower_than_best = bottom > best_bottom + RED_BOTTOM_PRIORITY_PIXELS;
        const bool same_bottom_band = bottom + RED_BOTTOM_PRIORITY_PIXELS >= best_bottom;
        if (!found || lower_than_best || (same_bottom_band && area > best_area)) {
            best_marker = current_marker;
            best_bottom = bottom;
            best_area = area;
            found = true;
        }
    }

    if (found) {
        marker = best_marker;
    }
    return found;
}

static bool estimate_target_quad_from_marker(const RedMarkerInfo &marker,
                                             std::vector<cv::Point2f> &target_quad)
{
    if (marker.quad.size() != 4 ||
        marker.long_edge_length <= 0.0f ||
        marker.short_edge_length <= 0.0f) {
        return false;
    }

    float target_side = marker.long_edge_length * TARGET_SIDE_TO_MARKER_LONG_EDGE;
    if (target_side < TARGET_MIN_SIDE_PIXELS) {
        target_side = TARGET_MIN_SIDE_PIXELS;
    }

    const cv::Point2f edge_center = (marker.long_edge_start + marker.long_edge_end) * 0.5f;
    const cv::Point2f bottom_center =
        edge_center +
        marker.long_edge_axis * (marker.long_edge_length * TARGET_CENTER_SHIFT_ALONG_LONG_EDGE) +
        marker.target_axis * (marker.short_edge_length * TARGET_BOTTOM_GAP_TO_MARKER_HEIGHT);

    const cv::Point2f half_width = marker.long_edge_axis * (target_side * 0.5f);
    const cv::Point2f height_vec = marker.target_axis * target_side;
    const cv::Point2f bottom_left = bottom_center - half_width;
    const cv::Point2f bottom_right = bottom_center + half_width;
    const cv::Point2f top_left = bottom_left + height_vec;
    const cv::Point2f top_right = bottom_right + height_vec;

    target_quad.clear();
    target_quad.push_back(top_left);
    target_quad.push_back(top_right);
    target_quad.push_back(bottom_right);
    target_quad.push_back(bottom_left);
    return true;
}

static bool warp_target_image(const cv::Mat &image,
                              const std::vector<cv::Point2f> &target_quad,
                              cv::Mat &target_image)
{
    if (image.empty() || target_quad.size() != 4) {
        return false;
    }

    std::vector<cv::Point2f> dst_quad;
    dst_quad.push_back(cv::Point2f(0.0f, 0.0f));
    dst_quad.push_back(cv::Point2f(static_cast<float>(MODEL_INPUT_WIDTH - 1), 0.0f));
    dst_quad.push_back(cv::Point2f(static_cast<float>(MODEL_INPUT_WIDTH - 1),
                                   static_cast<float>(MODEL_INPUT_HEIGHT - 1)));
    dst_quad.push_back(cv::Point2f(0.0f, static_cast<float>(MODEL_INPUT_HEIGHT - 1)));

    const cv::Mat transform = cv::getPerspectiveTransform(target_quad, dst_quad);
    cv::warpPerspective(image,
                        target_image,
                        transform,
                        cv::Size(MODEL_INPUT_WIDTH, MODEL_INPUT_HEIGHT),
                        cv::INTER_LINEAR,
                        cv::BORDER_REPLICATE);
    return !target_image.empty();
}

static bool crop_target_image(const cv::Mat &frame, cv::Mat &target_image)
{
    RedMarkerInfo marker;
    if (!find_red_marker(frame, marker)) {
        return false;
    }

    std::vector<cv::Point2f> target_quad;
    if (!estimate_target_quad_from_marker(marker, target_quad)) {
        return false;
    }

    return warp_target_image(frame, target_quad, target_image);
}

static BypassDir_e recognition_result_to_dir(const std::string &result)
{
    if (result == "hand_boom" || result == "hand_gun" || result == "gun") {
        return DIR_LEFT;
    }
    if (result == "ambulance" || result == "armoured_car") {
        return DIR_RIGHT;
    }
    return DIR_NONE;
}

static const char *dir_to_text(BypassDir_e dir)
{
    switch (dir) {
        case DIR_LEFT:
            return "LEFT";
        case DIR_RIGHT:
            return "RIGHT";
        default:
            return "NONE";
    }
}

static void store_pending_recognition(BypassDir_e dir, const std::string &result, float confidence)
{
    if (dir != DIR_LEFT && dir != DIR_RIGHT) {
        return;
    }

    std::lock_guard<std::mutex> lock(g_pending_mutex);
    g_pending_recognition.dir = dir;
    g_pending_recognition.result = result;
    g_pending_recognition.confidence = confidence;
    g_pending_recognition.available = true;
}

static bool take_pending_recognition(BypassDir_e &dir, std::string &result, float &confidence)
{
    std::lock_guard<std::mutex> lock(g_pending_mutex);
    if (!g_pending_recognition.available) {
        return false;
    }

    dir = g_pending_recognition.dir;
    result = g_pending_recognition.result;
    confidence = g_pending_recognition.confidence;
    g_pending_recognition.available = false;
    return true;
}

static void apply_recognition_result(BypassDir_e dir, const std::string &result, float confidence)
{
    if (confidence < AI_CONFIDENCE_THRESHOLD || !track_state.allow_change_special_barrier) {
        return;
    }

    if (dir != DIR_LEFT && dir != DIR_RIGHT) {
        return;
    }

    track_state.special_barrier_dir = dir;
    track_state.is_special_barrier = true;
    track_state.allow_change_special_barrier = false;

    std::printf("NCNN special barrier dir: %s, result: %s, confidence: %.6f\n",
                dir_to_text(dir),
                result.c_str(),
                confidence);
}

static void ai_inference_thread_func()
{
    while (true) {
        bool did_work = false;

        cv::Mat frame;
        if (ai_uvc_get_bgr_frame(frame)) {
            cv::Mat target_image;
            if (crop_target_image(frame, target_image)) {
                did_work = true;

                std::string result;
                float confidence = 0.0f;
                const auto infer_start = std::chrono::high_resolution_clock::now();//模型开始计时
                rao_start = std::chrono::high_resolution_clock::now();//全局推理时长开始计时
                const bool ok = g_classifier.infer(target_image, result, confidence);
                const auto infer_end = std::chrono::high_resolution_clock::now();

                if (ok) {
                    const BypassDir_e dir = recognition_result_to_dir(result);
                    if (confidence >= AI_CONFIDENCE_THRESHOLD &&
                        (dir == DIR_LEFT || dir == DIR_RIGHT)) {
                        store_pending_recognition(dir, result, confidence);

                        const auto duration =
                            std::chrono::duration_cast<std::chrono::milliseconds>(infer_end - infer_start);
                        std::printf("推理结果: %s, 置信度: %.6f, 方向候选: %s, 推理耗时: %ld ms\n",
                                    result.c_str(),
                                    confidence,
                                    dir_to_text(dir),
                                    duration.count());
                    }
                }
            }
        }

        if (!did_work) {
            std::this_thread::sleep_for(std::chrono::milliseconds(AI_IDLE_SLEEP_MS));
        } else {
            std::this_thread::yield();
        }
    }
}

static void start_ai_inference_worker()
{
    if (g_ai_worker_started || !g_classifier_ready) {
        return;
    }

    std::thread worker(ai_inference_thread_func);
    worker.detach();
    g_ai_worker_started = true;
}

int ai_init(void)
{
    g_classifier_ready = g_classifier.init();
    start_ai_inference_worker();
    return g_classifier_ready ? 0 : -1;
}

void ai_extract(void)
{
    if (!g_classifier_ready) {
        return;
    }

    BypassDir_e dir = DIR_NONE;
    std::string result;
    float confidence = 0.0f;
    if (take_pending_recognition(dir, result, confidence)) {
        apply_recognition_result(dir, result, confidence);
    }
}
