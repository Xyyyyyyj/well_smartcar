#ifndef _REMOTE_VIDEO_TX_HPP_
#define _REMOTE_VIDEO_TX_HPP_

#include "zf_driver_udp.hpp"

static constexpr int32 LINE_RESULT_MAX_POINTS = 256;

// 精简版线提取结果，仅保留图传所需字段
struct line_extract_result
{
    int32 left_count{0};
    int32 right_count{0};
    int32 left_ipm_count{0};
    int32 right_ipm_count{0};
    int32 center_ipm_count{0};
    uint16 left_x[LINE_RESULT_MAX_POINTS]{};
    uint16 left_y[LINE_RESULT_MAX_POINTS]{};
    uint16 right_x[LINE_RESULT_MAX_POINTS]{};
    uint16 right_y[LINE_RESULT_MAX_POINTS]{};
    uint16 left_ipm_x[LINE_RESULT_MAX_POINTS]{};
    uint16 left_ipm_y[LINE_RESULT_MAX_POINTS]{};
    uint16 right_ipm_x[LINE_RESULT_MAX_POINTS]{};
    uint16 right_ipm_y[LINE_RESULT_MAX_POINTS]{};
    uint16 center_ipm_x[LINE_RESULT_MAX_POINTS]{};
    uint16 center_ipm_y[LINE_RESULT_MAX_POINTS]{};
};

struct remote_video_ctrl_meta
{
    static constexpr int32 FAR_MAX_POINTS = 128;
    static constexpr int32 MERGE_MAX_POINTS = 128;

    uint16 track_side{0};
    uint16 element_type{0};
    uint16 near_aim_valid{0};
    int16 near_aim_err_x10{0};

    uint16 cross_type{0};
    uint16 cross_not_have_line{0};
    uint16 lpt0_found{0};
    uint16 lpt1_found{0};
    int16 lpt0_id{-1};
    int16 lpt1_id{-1};
    uint16 far_lpt0_found{0};
    uint16 far_lpt1_found{0};

    uint16 far_x1{0};
    uint16 far_x2{0};
    uint16 circle_type{0};

    uint16 far_left_count{0};
    uint16 far_right_count{0};
    uint16 far_left_x[FAR_MAX_POINTS]{};
    uint16 far_left_y[FAR_MAX_POINTS]{};
    uint16 far_right_x[FAR_MAX_POINTS]{};
    uint16 far_right_y[FAR_MAX_POINTS]{};
    uint16 far_raw_left_count{0};
    uint16 far_raw_right_count{0};
    uint16 far_raw_left_x[FAR_MAX_POINTS]{};
    uint16 far_raw_left_y[FAR_MAX_POINTS]{};
    uint16 far_raw_right_x[FAR_MAX_POINTS]{};
    uint16 far_raw_right_y[FAR_MAX_POINTS]{};

    uint16 merge_left_count{0};
    uint16 merge_right_count{0};
    uint16 merge_left_x[MERGE_MAX_POINTS]{};
    uint16 merge_left_y[MERGE_MAX_POINTS]{};
    uint16 merge_right_x[MERGE_MAX_POINTS]{};
    uint16 merge_right_y[MERGE_MAX_POINTS]{};
};

class remote_video_tx
{
private:
    zf_driver_udp m_udp;
    uint32 m_next_frame_id;
    bool m_inited;
    bool m_handshake_ready;
    uint32 m_last_wait_log_ms;
    uint32 m_last_stream_log_ms;
    uint32 m_stream_frames_since_log;

    void try_update_handshake();

public:
    remote_video_tx();

    int8 init(const char *ip_addr, uint32 port);
    uint32 send_gray_frame(const uint8 *frame_gray,
                           uint16 width,
                           uint16 height,
                           const line_extract_result &line_info,
                           const remote_video_ctrl_meta &ctrl_meta);
    bool is_inited() const;
};

#endif
