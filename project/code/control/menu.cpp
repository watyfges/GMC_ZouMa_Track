#include "menu.h"
extern uint8_t img_bin[TRACK_H][TRACK_W];  // 巡线二值化图像
extern uint8_t Group;
extern float Speed;
extern int Max_01_value;

namespace {

// 按键为低电平按下；如果实车读出来刚好相反，只需要把这里改成 1。
constexpr int KEY_PRESSED_LEVEL = 0;
// 菜单项数量要和 MenuItem 枚举保持一致，后续新增菜单项时一起加。
constexpr int MENU_ITEM_COUNT = 3;
// 每个可调参数的上下限集中放在这里，防止按键把参数调到危险范围外。
constexpr int GROUP_MIN = 0;
constexpr int GROUP_MAX = 1;
constexpr int SPEED_MIN = 0;
constexpr int SPEED_MAX = 600;
constexpr int VALUE_MIN = 0;
constexpr int VALUE_MAX = 255;
constexpr int SHORT_BEEP_MS = 50;
constexpr int LONG_BEEP_MS = 1000;
constexpr int START_HOLD_MS = 2000;

enum MenuItem {
    MENU_ITEM_GROUP = 0,
    MENU_ITEM_SPEED,
    MENU_ITEM_VALUE,
};

// 当前光标所在菜单项，屏幕右侧的箭头也由它决定。
int selected_item = MENU_ITEM_GROUP;
// 记录上一帧按键状态，用来做“新按下”检测，避免长按一次连续加减。
bool last_key_pressed[5] = {false};
bool start_key_timing = false;
bool start_key_fired = false;
bool speed_touched = false;
float menu_speed = 0.0f;
auto start_key_time = std::chrono::steady_clock::now();

int clamp_int(int value, int min_value, int max_value)
{
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

float clamp_float(float value, float min_value, float max_value)
{
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

bool key_pressed(int key)
{
    // scan_key() 返回原始 GPIO 电平，这里统一转换成“是否按下”。
    return scan_key(key) == KEY_PRESSED_LEVEL;
}

bool key_new_press(int key)
{
    // 只在松开后的第一次按下返回 true，相当于软件消抖/边沿触发。
    bool pressed = key_pressed(key);
    bool is_new = pressed && !last_key_pressed[key];
    last_key_pressed[key] = pressed;
    return is_new;
}

void beep_ms(int ms)
{
    beep_on();
    system_delay_ms(ms);
    beep_off();
}

void reset_key_state()
{
    for(int i = 0; i < 5; ++i) {
        last_key_pressed[i] = false;
    }
    start_key_timing = false;
    start_key_fired = false;
}

void adjust_current_item(int delta)
{
    // 只关心“当前选中项”，真正改哪个变量由 selected_item 决定。
    switch(selected_item) {
        case MENU_ITEM_GROUP:
            Group = static_cast<uint8_t>(clamp_int(static_cast<int>(Group) + delta, GROUP_MIN, GROUP_MAX));
            break;
        case MENU_ITEM_SPEED:
            // Speed 会被 control_init()/Choice_PID() 重设，所以手动调过后用 menu_speed 锁住菜单值。
            speed_touched = true;
            menu_speed = clamp_float(menu_speed + static_cast<float>(delta), SPEED_MIN, SPEED_MAX);
            Speed = menu_speed;
            break;
        case MENU_ITEM_VALUE:
            Max_01_value = clamp_int(Max_01_value + delta, VALUE_MIN, VALUE_MAX);
            break;
        default:
            break;
    }
}

void handle_start_key()
{
    // K1 长按 2 秒进入预备发车
    bool pressed = key_pressed(1);

    if(!pressed) {
        start_key_timing = false;
        start_key_fired = false;
        last_key_pressed[1] = false;
        return;
    }

    if(start_key_fired) {
        return;
    }

    if(!start_key_timing) {
        start_key_time = std::chrono::steady_clock::now();
        start_key_timing = true;
    }

    auto now = std::chrono::steady_clock::now();
    int hold_ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now - start_key_time).count());
    if(!start_key_fired && hold_ms >= START_HOLD_MS) {
        start_key_fired = true;
        start_key_timing = false;
        beep_ms(LONG_BEEP_MS);
        track_state.stop_flag = 1;
    }
}

const char* state_text()
{
    if(track_state.stop_flag == 0) return "STOP ";
    if(track_state.stop_flag == 1) return "READY";
    return "RUN  ";
}

} // namespace

void menu_task()
{
    // 菜单只在停车态工作，发车后锁定按键，防止跑车过程中误改参数。
    if(track_state.stop_flag != 0) {
        reset_key_state();
        return;
    }

    if(!speed_touched) {
        // 第一次手动调 Speed 之前，菜单显示跟随当前 PID 组给出的默认 Speed。
        menu_speed = Speed;
    } else {
        Speed = menu_speed;
    }

    // 当前键位：K4 加 1，K3 减 1，K2 切换菜单项。
    if(key_new_press(4)) {
        adjust_current_item(1);
        beep_ms(SHORT_BEEP_MS);
    }

    if(key_new_press(3)) {
        adjust_current_item(-1);
        beep_ms(SHORT_BEEP_MS);
    }

    if(key_new_press(2)) {
        selected_item = (selected_item + 1) % MENU_ITEM_COUNT;
        beep_ms(SHORT_BEEP_MS);
    }

    handle_start_key();
}

void ips_draw(){
    // 左上角保留巡线二值图，下方显示三个可调项和当前车辆状态。
    screen_show_gray_image(0, 0, img_bin[0], TRACK_W, TRACK_H, TRACK_W, TRACK_H, 0);

    screen_show_string(0,100,"Group:");
    screen_show_int(80,100,Group,1);
    screen_show_string(140,100,selected_item == MENU_ITEM_GROUP ? "<--" : "   ");

    screen_show_string(0,120,"Speed:");
    screen_show_float(80,120,Speed,3,1);
    screen_show_string(140,120,selected_item == MENU_ITEM_SPEED ? "<--" : "   ");

    screen_show_string(0,140,"Value:");
    screen_show_int(80,140,Max_01_value,3);
    screen_show_string(140,140,selected_item == MENU_ITEM_VALUE ? "<--" : "   ");


    screen_show_string(0,160,"State:");
    screen_show_string(80,160,state_text());

    screen_show_string(0,180,"DIR:");
    screen_show_int(80,180,track_state.special_barrier_dir,1);

    screen_show_string(0,200,"insp:");
    screen_show_int(80,200,track_state.is_special_barrier,1);

    screen_show_string(0,220,"inZebra:");
    screen_show_int(80,220,track_state.is_zebra,1);

    screen_show_string(0,240,"inBarrier:");
    screen_show_int(80,240,track_state.is_barrier,1);


    //ips200_show_gray_image(0, 180, img_bin[0], TRACK_W, TRACK_H);//显示图像
} 