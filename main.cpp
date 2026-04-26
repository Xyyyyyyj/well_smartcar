
/*********************************************************************************************************************
* LS2K0300 用户主程序：视觉线程独立更新 err_x，控制线程固定 10ms 做差速与速度 PI。
* 转角 = err_x*KP + err_x*|err_x|*KP2 + d_err_x*KD + gyro_z*GKD → steer_rpm_imu（限幅）。
********************************************************************************************************************/

#include "zf_common_headfile.hpp"
#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <time.h>
#include "zf_device_uvc.hpp"
#include "motor_control.hpp"
#include "line_tracking.hpp"
#include "remote_video_tx.hpp"

// 上位机 IP 与端口（与 ls2k300_view 配置一致）
static constexpr const char *REMOTE_VIDEO_IP   = "192.168.1.100";
static constexpr uint32      REMOTE_VIDEO_PORT  = 19001;

// 将 LaneResult 转换为图传协议所需的 line_extract_result
static void lane_result_to_extract(const LaneResult &src, line_extract_result &dst)
{
    const int lc = (src.left_points_count  < LINE_RESULT_MAX_POINTS)
                       ? (int)src.left_points_count  : LINE_RESULT_MAX_POINTS;
    const int rc = (src.right_points_count < LINE_RESULT_MAX_POINTS)
                       ? (int)src.right_points_count : LINE_RESULT_MAX_POINTS;
    dst.left_count  = lc;
    dst.right_count = rc;
    dst.left_ipm_count  = 0;
    dst.right_ipm_count = 0;
    dst.center_ipm_count = 0;
    for(int i = 0; i < lc; i++)
    {
        dst.left_x[i] = (uint16)src.left_points[i].x;
        dst.left_y[i] = (uint16)src.left_points[i].y;
    }
    for(int i = 0; i < rc; i++)
    {
        dst.right_x[i] = (uint16)src.right_points[i].x;
        dst.right_y[i] = (uint16)src.right_points[i].y;
    }
}

static bool save_pgm(const char *path, const uint8_t *gray, int w, int h)
{
    if(nullptr == path || nullptr == gray || w <= 0 || h <= 0)
    {
        return false;
    }

    FILE *fp = std::fopen(path, "wb");
    if(nullptr == fp)
    {
        return false;
    }

    // PGM binary (P5): header + raw 8-bit gray data, row-major
    std::fprintf(fp, "P5\n%d %d\n255\n", w, h);
    const size_t bytes = (size_t)w * (size_t)h;
    const size_t wrote = std::fwrite(gray, 1, bytes, fp);
    std::fclose(fp);
    return wrote == bytes;
}

// =============================== 速度闭环（rpm）参数 ===============================
static constexpr int32 ENCODER_COUNTS_PER_REV = 1024;
// 轮速外环基准目标转速（rpm）
static constexpr int32 TARGET_RPM_BASE = 1000;

static constexpr float PWM_DUTY_LIMIT_PERCENT = 100.0f;

static constexpr bool  USE_MANUAL_PWM = false;
static constexpr float MANUAL_DUTY_PERCENT_L = 10.0f;
static constexpr float MANUAL_DUTY_PERCENT_R = 10.0f;

static constexpr float SPEED_KP = 5.0f;
static constexpr float SPEED_KI = 5.0f;
static constexpr float SPEED_LOOP_DIV = 100000.0f;
static constexpr float SPEED_INTEGRAL_LIMIT = 500000.0f;
// 反向制动由 PI 直接输出负占空比（smartcar 同风格），仅限制反向占空比下限比例
static constexpr float REVERSE_BRAKE_DUTY_LIMIT_SCALE = 0.20f;

// =============================== 巡线转角合成 → 轮速差速 ===============================
// 转角值（映射为 steer_rpm_imu，单位与左右目标 rpm 同）：需整定
// 合成差速单轮最多加减的转速上限
static constexpr int32 STEER_RPM_MAX = 5000;

// =============================== 弯道/直道状态判断 + 分模式转角参数 ===============================
// |err_x| 小于阈值认为该帧“更像直道”，否则更像弯道；使用多帧去抖做状态切换。
static constexpr float STRAIGHT_ERR_X_THRESHOLD = 10.0f;
static constexpr int32 STRAIGHT_ENTER_FRAMES = 5; // 连续满足直道帧数，切入直道
static constexpr int32 CURVE_ENTER_FRAMES = 2;    // 连续满足弯道帧数，切入弯道

// 直道转角参数（更保守）
static constexpr float STEER_STRAIGHT_KP  = 20.0f;
static constexpr float STEER_STRAIGHT_KP2 = 1.0f;
static constexpr float STEER_STRAIGHT_KD  = 0.0f;
static constexpr float STEER_STRAIGHT_GKD = 0.8f;

// 弯道转角参数（沿用当前参数）
static constexpr float STEER_CURVE_KP  = 30.0f;
static constexpr float STEER_CURVE_KP2 = 2.0f;
static constexpr float STEER_CURVE_KD  = 0.0f;
static constexpr float STEER_CURVE_GKD = 1.0f;

// 控制环固定周期（用于速度推算）
static constexpr int32 CONTROL_PERIOD_MS = 10;
static constexpr int32 LOG_PERIOD_MS = 500;

static inline timespec timespec_add_ms(timespec t, int32 ms)
{
    t.tv_sec += ms / 1000;
    t.tv_nsec += (long)(ms % 1000) * 1000000L;
    if(t.tv_nsec >= 1000000000L)
    {
        t.tv_sec += 1;
        t.tv_nsec -= 1000000000L;
    }
    return t;
}

static inline bool timespec_before(const timespec &a, const timespec &b)
{
    if(a.tv_sec != b.tv_sec)
    {
        return a.tv_sec < b.tv_sec;
    }
    return a.tv_nsec < b.tv_nsec;
}

static inline uint32 timespec_diff_us_signed(const timespec &a, const timespec &b)
{
    // return (a - b) in microseconds, saturate to uint32
    int64 sec = (int64)a.tv_sec - (int64)b.tv_sec;
    int64 nsec = (int64)a.tv_nsec - (int64)b.tv_nsec;
    int64 us = sec * 1000000LL + nsec / 1000LL;
    if(us < 0) us = 0;
    if(us > 0xFFFFFFFFLL) us = 0xFFFFFFFFLL;
    return (uint32)us;
}

/** 睡到下一节拍：CLOCK_MONOTONIC + TIMER_ABSTIME，累积不漂移；超时则快进 next，避免连跑不占 CPU。 */
static void wait_control_period(timespec &next_deadline)
{
    next_deadline = timespec_add_ms(next_deadline, CONTROL_PERIOD_MS);
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    while(!timespec_before(now, next_deadline))
    {
        next_deadline = timespec_add_ms(next_deadline, CONTROL_PERIOD_MS);
    }
    while(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_deadline, nullptr) == EINTR)
    {
    }
}

struct vision_worker
{
    std::atomic<bool> exit_flag{false};
    std::atomic<int32> latest_err_x{0};
    std::atomic<uint32> frame_seq{0};
    pthread_t tid{};

    zf_device_uvc uvc_cam;
    remote_video_tx video_tx;

    static void *thread_entry(void *arg)
    {
        auto *self = static_cast<vision_worker *>(arg);

        while(!self->exit_flag.load(std::memory_order_relaxed))
        {
            if(0 != self->uvc_cam.wait_image_refresh())
            {
                continue;
            }

            uint8_t *gray_ptr = self->uvc_cam.get_gray_image_ptr();
            if(nullptr == gray_ptr)
            {
                continue;
            }

            int32 err_x = 0;
            line_tracking_process_frame(gray_ptr, err_x);
            self->latest_err_x.store(err_x);
            self->frame_seq.fetch_add(1);

            // 图传：发送处理后的灰度图及巡线点
            if(self->video_tx.is_inited())
            {
                line_extract_result extract{};
                lane_result_to_extract(line_tracking_get_last_result(), extract);
                remote_video_ctrl_meta meta{};
                self->video_tx.send_gray_frame(gray_ptr, UVC_WIDTH, UVC_HEIGHT, extract, meta);
            }
        }

        return nullptr;
    }

    bool start()
    {
        if(0 != uvc_cam.init("/dev/video0"))
        {
            return false;
        }

        // 初始化完成后抓拍一帧，保存为 PGM 文件（便于快速验证摄像头取图正常）
        // 文件生成在程序当前工作目录下：uvc_snapshot.pgm
        if(0 == uvc_cam.wait_image_refresh())
        {
            uint8_t *gray_ptr = uvc_cam.get_gray_image_ptr();
            if(nullptr != gray_ptr)
            {
                if(save_pgm("uvc_snapshot.pgm", gray_ptr, (int)UVC_WIDTH, (int)UVC_HEIGHT))
                {
                    std::printf("UVC snapshot saved: uvc_snapshot.pgm (%dx%d)\r\n", (int)UVC_WIDTH, (int)UVC_HEIGHT);
                }
                else
                {
                    std::printf("UVC snapshot save failed.\r\n");
                }
            }
        }

        if(0 != video_tx.init(REMOTE_VIDEO_IP, REMOTE_VIDEO_PORT))
        {
            std::printf("Remote video TX init failed (IP=%s port=%lu), image streaming disabled.\r\n",
                        REMOTE_VIDEO_IP, (unsigned long)REMOTE_VIDEO_PORT);
        }
        else
        {
            std::printf("Remote video TX init OK, target=%s:%lu\r\n",
                        REMOTE_VIDEO_IP, (unsigned long)REMOTE_VIDEO_PORT);
        }

        return 0 == pthread_create(&tid, nullptr, thread_entry, this);
    }

    void stop()
    {
        exit_flag.store(true, std::memory_order_relaxed);
        if(tid)
        {
            pthread_join(tid, nullptr);
            tid = {};
        }
    }
};

static int32 encoder_delta_to_rpm(int32 delta_count, uint32 dt_us)
{
    if(ENCODER_COUNTS_PER_REV <= 0 || dt_us == 0)
    {
        return 0;
    }

    const int64 denom = (int64)ENCODER_COUNTS_PER_REV * (int64)dt_us;
    return (int32)(((int64)delta_count * 60000000LL) / denom);
}

struct inc_pi_state
{
    float u{0.0f};      // 上一次输出（占空比命令）
    float e1{0.0f};     // 上一次误差
    float i_acc{0.0f};  // 误差积分累加（限幅）
};

enum class road_mode : uint8_t
{
    STRAIGHT = 0,
    CURVE = 1
};

// 弯/直道独立状态机：只基于 err_x 判定，不依赖任何环岛状态或标志位。
struct road_mode_sm
{
    road_mode mode{road_mode::STRAIGHT};
    int32 straight_like_count{0};
    int32 curve_like_count{0};

    road_mode update(int32 err_x)
    {
        const int64 abs_err = (int64)std::llabs((long long)err_x);
        const bool straight_like = ((float)abs_err <= STRAIGHT_ERR_X_THRESHOLD);
        if(straight_like)
        {
            if(straight_like_count < INT32_MAX) straight_like_count++;
            curve_like_count = 0;
        }
        else
        {
            if(curve_like_count < INT32_MAX) curve_like_count++;
            straight_like_count = 0;
        }

        if(mode == road_mode::STRAIGHT)
        {
            if(curve_like_count >= CURVE_ENTER_FRAMES) mode = road_mode::CURVE;
        }
        else
        {
            if(straight_like_count >= STRAIGHT_ENTER_FRAMES) mode = road_mode::STRAIGHT;
        }
        return mode;
    }
};

// 增量式 PI：
// u(k) = u(k-1) + Kp*(e(k)-e(k-1)) + Ki*e(k)
// 这里 Kp/Ki 与 SPEED_LOOP_DIV 配合做定点缩放；采样周期固定 10ms，因此 Ki 需按该周期整定。
static int32 speed_inc_pi_output(int32 target_rpm, int32 actual_rpm, inc_pi_state &st, int32 max_duty)
{
    const float e = (float)(target_rpm - actual_rpm);
    const float de = e - st.e1;

    float i_new = st.i_acc + e;
    if(i_new > SPEED_INTEGRAL_LIMIT) i_new = SPEED_INTEGRAL_LIMIT;
    if(i_new < -SPEED_INTEGRAL_LIMIT) i_new = -SPEED_INTEGRAL_LIMIT;
    const float di = i_new - st.i_acc;

    const float du = (SPEED_KP * de + SPEED_KI * di) * ((float)max_duty / SPEED_LOOP_DIV);
    float u_new = st.u + du;

    // 与 smartcar 一致：PI 允许有限反向输出，实现“PI 控制的反向制动”
    const float u_min = -(float)max_duty * REVERSE_BRAKE_DUTY_LIMIT_SCALE;
    if(u_new < u_min) u_new = u_min;
    if(u_new > (float)max_duty) u_new = (float)max_duty;

    st.i_acc = i_new;
    st.u = u_new;
    st.e1 = e;
    return (u_new >= 0.0f) ? (int32)(u_new + 0.5f) : (int32)(u_new - 0.5f);
}

int main(int, char**)
{
    vision_worker vision;
    if(!vision.start())
    {
        printf("UVC camera init failed, please check /dev/video0.\r\n");
        while(1)
        {
            system_delay_ms(1000);
        }
    }

    two_motor_driver motors;
    motors.init();
    const int32 max_duty_left = (int32)((float)motors.duty_max_left() * (PWM_DUTY_LIMIT_PERCENT / 100.0f));
    const int32 max_duty_right = (int32)((float)motors.duty_max_right() * (PWM_DUTY_LIMIT_PERCENT / 100.0f));

    inc_pi_state pi_left{};
    inc_pi_state pi_right{};
    int32 duty_left = 0;
    int32 duty_right = 0;
    int32 duty_cmd_left = 0;
    int32 duty_cmd_right = 0;
    road_mode_sm road_sm{};

    motors.set_speed_duty(0, 0);

    zf_device_imu imu;
    const imu_device_type_enum imu_type = imu.init();
    const bool imu_ok = (imu_type == DEV_IMU660RA);
    if(!imu_ok)
    {
        printf("IMU660RA: init fail or not DEV_IMU660RA (type=%d), yaw PID disabled.\r\n", (int)imu_type);
    }
    else
    {
        printf("IMU660RA ok, line steer uses gyro_z * GKD term.\r\n");
    }

    printf("Vision thread + 10ms control thread + wheel speed PI start.\r\n");

    timespec control_next{};
    clock_gettime(CLOCK_MONOTONIC, &control_next);

    timespec log_next = control_next;
    uint32 last_frame_seq = 0;
    int32 last_frame_err_x = 0;

    zf_driver_encoder encoder_left(ZF_ENCODER_QUAD_1);
    zf_driver_encoder encoder_right(ZF_ENCODER_QUAD_2);
    encoder_left.clear_count();
    encoder_right.clear_count();
    timespec enc_last_ts{};
    clock_gettime(CLOCK_MONOTONIC, &enc_last_ts);

    while(1)
    {
        const uint32 frame_seq = vision.frame_seq.load();
        const int32 err_x = vision.latest_err_x.load();
        const int32 base_rpm = TARGET_RPM_BASE;

        int32 d_err_x = 0;
        if(frame_seq != last_frame_seq)
        {
            d_err_x = err_x - last_frame_err_x;
            last_frame_err_x = err_x;
            last_frame_seq = frame_seq;
        }

        int16 gyro_z = 0;
        if(imu_ok)
        {
            gyro_z = imu.get_gyro_z();
        }

        const int64 err_x64 = (int64)err_x;
        const int64 abs_err = (int64)std::llabs((long long)err_x);
        const road_mode mode = road_sm.update(err_x);

        float kp = STEER_STRAIGHT_KP;
        float kp2 = STEER_STRAIGHT_KP2;
        float kd = STEER_STRAIGHT_KD;
        float gkd = STEER_STRAIGHT_GKD;
        if(mode == road_mode::CURVE)
        {
            kp = STEER_CURVE_KP;
            kp2 = STEER_CURVE_KP2;
            kd = STEER_CURVE_KD;
            gkd = STEER_CURVE_GKD;
        }

        float turn = (float)err_x64 * kp +
                     (float)(err_x64 * abs_err) * kp2 +
                     (float)d_err_x * kd;
        if(imu_ok)
        {
            turn += (float)gyro_z * gkd;
        }

        int32 steer_rpm_imu = clamp_int32((int32)std::lround(turn), -STEER_RPM_MAX, STEER_RPM_MAX);

        int32 target_rpm_left = base_rpm + steer_rpm_imu;
        int32 target_rpm_right = base_rpm - steer_rpm_imu;
        target_rpm_left = clamp_int32(target_rpm_left, 0, TARGET_RPM_BASE + STEER_RPM_MAX);
        target_rpm_right = clamp_int32(target_rpm_right, 0, TARGET_RPM_BASE + STEER_RPM_MAX);

        const int16 delta_left_i16 = encoder_left.get_count();
        const int16 delta_right_i16 = encoder_right.get_count();
        encoder_left.clear_count();
        encoder_right.clear_count();

        timespec enc_now{};
        clock_gettime(CLOCK_MONOTONIC, &enc_now);
        const uint32 dt_us = timespec_diff_us_signed(enc_now, enc_last_ts);
        enc_last_ts = enc_now;

        const int32 delta_left = (int32)delta_left_i16;
        const int32 delta_right = (int32)delta_right_i16;

        int32 rpm_left = 0;
        int32 rpm_right = 0;
        rpm_left = encoder_delta_to_rpm(delta_left, dt_us);
        // 右轮安装方向与左轮相反时在这里反相一次。
        rpm_right = encoder_delta_to_rpm(-delta_right, dt_us);

        if(USE_MANUAL_PWM)
        {
            duty_left = (int32)((float)motors.duty_max_left() * (MANUAL_DUTY_PERCENT_L / 100.0f));
            duty_right = (int32)((float)motors.duty_max_right() * (MANUAL_DUTY_PERCENT_R / 100.0f));
            duty_left = clamp_int32(duty_left, -(int32)((float)max_duty_left * REVERSE_BRAKE_DUTY_LIMIT_SCALE), max_duty_left);
            duty_right = clamp_int32(duty_right, -(int32)((float)max_duty_right * REVERSE_BRAKE_DUTY_LIMIT_SCALE), max_duty_right);
        }
        else
        {
            duty_left = speed_inc_pi_output(target_rpm_left, rpm_left, pi_left, max_duty_left);
            duty_right = speed_inc_pi_output(target_rpm_right, rpm_right, pi_right, max_duty_right);
        }

        duty_cmd_left = duty_left;
        duty_cmd_right = duty_right;
        motors.set_speed_duty(duty_cmd_left, duty_cmd_right);

        timespec now_ts{};
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        if(!timespec_before(now_ts, log_next))
        {
            log_next = timespec_add_ms(log_next, LOG_PERIOD_MS);
            // 暂时注释掉其他周期日志，避免淹没环岛调试输出
        //     printf("frm:%lu base:%ld mode:%s imu:%d lc_x:%ld dlc:%ld gz:%d steer:%ld | tgtL:%ld tgtR:%ld | dt:%luus | RPM L:%ld R:%ld | duty L:%d R:%d\r\n",
        //            (unsigned long)frame_seq,
        //            (long)base_rpm,
        //            (mode == road_mode::STRAIGHT) ? "S" : "C",
        //            (int)imu_ok,
        //            (long)err_x,
        //            (long)d_err_x,
        //            (int)gyro_z,
        //            (long)steer_rpm_imu,
        //            (long)target_rpm_left,
        //            (long)target_rpm_right,
        //            (unsigned long)dt_us,
        //            (long)rpm_left,
        //            (long)rpm_right,
        //            (int)duty_cmd_left,
        //            (int)duty_cmd_right);
        }

        wait_control_period(control_next);
    }

    vision.stop();
}
