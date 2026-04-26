#include "remote_video_tx.hpp"
#include <cstdint>
#include <cstring>
#include <ctime>

namespace
{
static constexpr uint16 REMOTE_VIDEO_MAGIC = 0x5A5A;
static constexpr uint32 REMOTE_VIDEO_MAX_UDP_PAYLOAD = 1400;
static constexpr uint32 REMOTE_META_SIZE = 56;
static constexpr uint32 REMOTE_POINT_SIZE = 4;
static constexpr char REMOTE_HANDSHAKE_CMD[] = "LS2K_VIDEO_HANDSHAKE";
static constexpr uint32 REMOTE_HANDSHAKE_MAX_SIZE = 64;

#pragma pack(push, 1)
struct remote_video_packet_header
{
    uint16 magic;
    uint16 header_size;
    uint32 frame_id;
    uint16 packet_index;
    uint16 packet_total;
    uint16 payload_size;
    uint16 reserved;
};
#pragma pack(pop)
} // namespace

static uint32 monotonic_ms_now()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    const uint64_t ms = (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
    return (uint32)(ms & 0xFFFFFFFFULL);
}

static bool parse_handshake_ip(const uint8 *buff, uint32 recv_len, char *out_ip, uint32 out_ip_size)
{
    if(nullptr == buff || nullptr == out_ip || out_ip_size == 0)
        return false;
    const uint32 cmd_len = (uint32)(sizeof(REMOTE_HANDSHAKE_CMD) - 1);
    if(recv_len <= cmd_len)
        return false;
    if(0 != memcmp(buff, REMOTE_HANDSHAKE_CMD, cmd_len))
        return false;
    if(buff[cmd_len] != '|')
        return false;
    const uint32 ip_len = recv_len - cmd_len - 1;
    if(ip_len == 0 || ip_len >= out_ip_size)
        return false;
    for(uint32 i = 0; i < ip_len; i++)
    {
        const char c = (char)buff[cmd_len + 1 + i];
        if(!((c >= '0' && c <= '9') || c == '.'))
            return false;
        out_ip[i] = c;
    }
    out_ip[ip_len] = '\0';
    return true;
}

remote_video_tx::remote_video_tx()
    : m_next_frame_id(0), m_inited(false), m_handshake_ready(false),
      m_last_wait_log_ms(0), m_last_stream_log_ms(0), m_stream_frames_since_log(0)
{
}

int8 remote_video_tx::init(const char *ip_addr, uint32 port)
{
    if(0 != m_udp.init(ip_addr, port))
    {
        m_inited = false;
        return -1;
    }
    m_handshake_ready = false;
    m_last_wait_log_ms = 0;
    m_last_stream_log_ms = 0;
    m_stream_frames_since_log = 0;
    m_inited = true;
    return 0;
}

void remote_video_tx::try_update_handshake()
{
    uint8 recv_buff[REMOTE_HANDSHAKE_MAX_SIZE];
    char target_ip[32];
    while(true)
    {
        const uint32 recv_len = m_udp.read_data(recv_buff, sizeof(recv_buff));
        if(0 == recv_len)
            break;
        if(recv_len == (uint32)(sizeof(REMOTE_HANDSHAKE_CMD) - 1)
           && 0 == memcmp(recv_buff, REMOTE_HANDSHAKE_CMD, sizeof(REMOTE_HANDSHAKE_CMD) - 1))
        {
            if(!m_handshake_ready)
                printf("Remote video handshake OK, start streaming.\r\n");
            m_handshake_ready = true;
            continue;
        }
        if(parse_handshake_ip(recv_buff, recv_len, target_ip, sizeof(target_ip)))
        {
            if(0 == m_udp.set_target_ip(target_ip))
            {
                if(!m_handshake_ready)
                    printf("Remote video handshake OK, start streaming. target=%s\r\n", target_ip);
                m_handshake_ready = true;
            }
        }
    }
}

uint32 remote_video_tx::send_gray_frame(const uint8 *frame_gray,
                                        uint16 width,
                                        uint16 height,
                                        const line_extract_result &line_info,
                                        const remote_video_ctrl_meta &ctrl_meta)
{
    const uint32 frame_size = (uint32)width * (uint32)height;
    if(!m_inited || nullptr == frame_gray || 0 == frame_size)
        return 0;

    try_update_handshake();
    if(!m_handshake_ready)
    {
        const uint32 now_ms = monotonic_ms_now();
        if((uint32)(now_ms - m_last_wait_log_ms) >= 1000U)
        {
            printf("Remote video waiting handshake...\r\n");
            m_last_wait_log_ms = now_ms;
        }
        return 0;
    }

    const uint16 left_count = (uint16)((line_info.left_count < 0) ? 0 :
        ((line_info.left_count > LINE_RESULT_MAX_POINTS) ? LINE_RESULT_MAX_POINTS : line_info.left_count));
    const uint16 right_count = (uint16)((line_info.right_count < 0) ? 0 :
        ((line_info.right_count > LINE_RESULT_MAX_POINTS) ? LINE_RESULT_MAX_POINTS : line_info.right_count));
    const uint16 left_ipm_count = (uint16)((line_info.left_ipm_count < 0) ? 0 :
        ((line_info.left_ipm_count > LINE_RESULT_MAX_POINTS) ? LINE_RESULT_MAX_POINTS : line_info.left_ipm_count));
    const uint16 right_ipm_count = (uint16)((line_info.right_ipm_count < 0) ? 0 :
        ((line_info.right_ipm_count > LINE_RESULT_MAX_POINTS) ? LINE_RESULT_MAX_POINTS : line_info.right_ipm_count));
    const uint16 center_ipm_count = (uint16)((line_info.center_ipm_count < 0) ? 0 :
        ((line_info.center_ipm_count > LINE_RESULT_MAX_POINTS) ? LINE_RESULT_MAX_POINTS : line_info.center_ipm_count));
    const uint16 far_left_count = (uint16)((ctrl_meta.far_left_count > remote_video_ctrl_meta::FAR_MAX_POINTS)
                                              ? remote_video_ctrl_meta::FAR_MAX_POINTS : ctrl_meta.far_left_count);
    const uint16 far_right_count = (uint16)((ctrl_meta.far_right_count > remote_video_ctrl_meta::FAR_MAX_POINTS)
                                               ? remote_video_ctrl_meta::FAR_MAX_POINTS : ctrl_meta.far_right_count);
    const uint16 far_raw_left_count = (uint16)((ctrl_meta.far_raw_left_count > remote_video_ctrl_meta::FAR_MAX_POINTS)
                                                  ? remote_video_ctrl_meta::FAR_MAX_POINTS : ctrl_meta.far_raw_left_count);
    const uint16 far_raw_right_count = (uint16)((ctrl_meta.far_raw_right_count > remote_video_ctrl_meta::FAR_MAX_POINTS)
                                                   ? remote_video_ctrl_meta::FAR_MAX_POINTS : ctrl_meta.far_raw_right_count);
    const uint16 merge_left_count = (uint16)((ctrl_meta.merge_left_count > remote_video_ctrl_meta::MERGE_MAX_POINTS)
                                                ? remote_video_ctrl_meta::MERGE_MAX_POINTS : ctrl_meta.merge_left_count);
    const uint16 merge_right_count = (uint16)((ctrl_meta.merge_right_count > remote_video_ctrl_meta::MERGE_MAX_POINTS)
                                                 ? remote_video_ctrl_meta::MERGE_MAX_POINTS : ctrl_meta.merge_right_count);

    const uint32 points_size = ((uint32)left_count + (uint32)right_count + (uint32)left_ipm_count
                                + (uint32)right_ipm_count + (uint32)center_ipm_count) * REMOTE_POINT_SIZE;
    const uint32 far_points_size = ((uint32)far_left_count + (uint32)far_right_count) * REMOTE_POINT_SIZE;
    const uint32 merge_points_size = ((uint32)merge_left_count + (uint32)merge_right_count) * REMOTE_POINT_SIZE;
    const uint32 far_raw_points_size = ((uint32)far_raw_left_count + (uint32)far_raw_right_count) * REMOTE_POINT_SIZE;
    const uint32 payload_size_total = REMOTE_META_SIZE + frame_size + points_size
                                      + far_points_size + far_raw_points_size + merge_points_size;

    static constexpr uint32 header_size = (uint32)sizeof(remote_video_packet_header);
    static constexpr uint32 max_chunk_data = REMOTE_VIDEO_MAX_UDP_PAYLOAD - header_size;
    if(0 == max_chunk_data)
        return 0;

    const uint16 packet_total = (uint16)((payload_size_total + max_chunk_data - 1) / max_chunk_data);
    uint32 sent_bytes = 0;

    uint8 packet_buffer[REMOTE_VIDEO_MAX_UDP_PAYLOAD];
    remote_video_packet_header header{};
    header.magic = REMOTE_VIDEO_MAGIC;
    header.header_size = (uint16)header_size;
    header.frame_id = m_next_frame_id++;
    header.packet_total = packet_total;

    for(uint16 idx = 0; idx < packet_total; idx++)
    {
        const uint32 offset = (uint32)idx * max_chunk_data;
        const uint32 remain = payload_size_total - offset;
        const uint16 chunk_size = (uint16)((remain > max_chunk_data) ? max_chunk_data : remain);

        header.packet_index = idx;
        header.payload_size = chunk_size;
        header.reserved = 0;

        memcpy(packet_buffer, &header, header_size);
        uint8 *dst = packet_buffer + header_size;
        const uint32 chunk_begin = offset;
        const uint32 chunk_end = offset + chunk_size;

        uint32 write_off = chunk_begin;
        while(write_off < chunk_end)
        {
            if(write_off < REMOTE_META_SIZE)
            {
                const uint16 meta_vals[28] =
                {
                    width, height,
                    left_count, right_count, left_ipm_count, right_ipm_count, center_ipm_count,
                    ctrl_meta.track_side, ctrl_meta.element_type, ctrl_meta.near_aim_valid,
                    (uint16)ctrl_meta.near_aim_err_x10,
                    ctrl_meta.cross_type, ctrl_meta.cross_not_have_line,
                    ctrl_meta.lpt0_found, ctrl_meta.lpt1_found,
                    (uint16)ctrl_meta.lpt0_id, (uint16)ctrl_meta.lpt1_id,
                    ctrl_meta.far_lpt0_found, ctrl_meta.far_lpt1_found,
                    far_left_count, far_right_count,
                    ctrl_meta.far_x1, ctrl_meta.far_x2,
                    merge_left_count, merge_right_count,
                    far_raw_left_count, far_raw_right_count,
                    ctrl_meta.circle_type,
                };
                const uint32 meta_pos = write_off;
                const uint32 copy_n = ((chunk_end - write_off) < (REMOTE_META_SIZE - meta_pos))
                                          ? (chunk_end - write_off) : (REMOTE_META_SIZE - meta_pos);
                memcpy(dst, ((const uint8 *)meta_vals) + meta_pos, copy_n);
                dst += copy_n;
                write_off += copy_n;
                continue;
            }

            const uint32 gray_begin = REMOTE_META_SIZE;
            const uint32 gray_end = gray_begin + frame_size;
            if(write_off < gray_end)
            {
                const uint32 gray_pos = write_off - gray_begin;
                const uint32 copy_n = ((chunk_end - write_off) < (gray_end - write_off))
                                          ? (chunk_end - write_off) : (gray_end - write_off);
                memcpy(dst, frame_gray + gray_pos, copy_n);
                dst += copy_n;
                write_off += copy_n;
                continue;
            }

            const uint32 points_begin = gray_end;
            const uint32 point_idx = (write_off - points_begin) / REMOTE_POINT_SIZE;
            const uint32 in_point_off = (write_off - points_begin) % REMOTE_POINT_SIZE;
            uint16 x = 0;
            uint16 y = 0;
            const uint32 left_begin = 0;
            const uint32 right_begin = left_begin + (uint32)left_count;
            const uint32 left_ipm_begin = right_begin + (uint32)right_count;
            const uint32 right_ipm_begin = left_ipm_begin + (uint32)left_ipm_count;
            const uint32 center_ipm_begin = right_ipm_begin + (uint32)right_ipm_count;
            const uint32 far_left_begin = center_ipm_begin + (uint32)center_ipm_count;
            const uint32 far_right_begin = far_left_begin + (uint32)far_left_count;
            const uint32 far_raw_left_begin = far_right_begin + (uint32)far_right_count;
            const uint32 far_raw_right_begin = far_raw_left_begin + (uint32)far_raw_left_count;
            const uint32 merge_left_begin = far_raw_right_begin + (uint32)far_raw_right_count;
            const uint32 merge_right_begin = merge_left_begin + (uint32)merge_left_count;

            if(point_idx < (uint32)left_count)
            {
                x = line_info.left_x[point_idx];
                y = line_info.left_y[point_idx];
            }
            else if(point_idx < left_ipm_begin)
            {
                const uint32 ridx = point_idx - (uint32)left_count;
                x = line_info.right_x[ridx];
                y = line_info.right_y[ridx];
            }
            else if(point_idx < right_ipm_begin)
            {
                const uint32 lidx = point_idx - left_ipm_begin;
                x = line_info.left_ipm_x[lidx];
                y = line_info.left_ipm_y[lidx];
            }
            else if(point_idx < center_ipm_begin)
            {
                const uint32 ridx = point_idx - right_ipm_begin;
                x = line_info.right_ipm_x[ridx];
                y = line_info.right_ipm_y[ridx];
            }
            else if(point_idx < far_left_begin)
            {
                const uint32 cidx = point_idx - center_ipm_begin;
                x = line_info.center_ipm_x[cidx];
                y = line_info.center_ipm_y[cidx];
            }
            else if(point_idx < far_right_begin)
            {
                const uint32 fidx = point_idx - far_left_begin;
                x = ctrl_meta.far_left_x[fidx];
                y = ctrl_meta.far_left_y[fidx];
            }
            else if(point_idx < merge_left_begin)
            {
                if(point_idx < far_raw_left_begin)
                {
                    const uint32 fidx = point_idx - far_right_begin;
                    x = ctrl_meta.far_right_x[fidx];
                    y = ctrl_meta.far_right_y[fidx];
                }
                else if(point_idx < far_raw_right_begin)
                {
                    const uint32 fidx = point_idx - far_raw_left_begin;
                    x = ctrl_meta.far_raw_left_x[fidx];
                    y = ctrl_meta.far_raw_left_y[fidx];
                }
                else
                {
                    const uint32 fidx = point_idx - far_raw_right_begin;
                    x = ctrl_meta.far_raw_right_x[fidx];
                    y = ctrl_meta.far_raw_right_y[fidx];
                }
            }
            else if(point_idx < merge_right_begin)
            {
                const uint32 midx = point_idx - merge_left_begin;
                x = ctrl_meta.merge_left_x[midx];
                y = ctrl_meta.merge_left_y[midx];
            }
            else
            {
                const uint32 midx = point_idx - merge_right_begin;
                x = ctrl_meta.merge_right_x[midx];
                y = ctrl_meta.merge_right_y[midx];
            }

            const uint16 point_vals[2] = {x, y};
            const uint32 copy_n = ((chunk_end - write_off) < (REMOTE_POINT_SIZE - in_point_off))
                                      ? (chunk_end - write_off) : (REMOTE_POINT_SIZE - in_point_off);
            memcpy(dst, ((const uint8 *)point_vals) + in_point_off, copy_n);
            dst += copy_n;
            write_off += copy_n;
        }

        const uint32 packet_size = header_size + chunk_size;
        const uint32 real_sent = m_udp.send_data(packet_buffer, packet_size);
        if(real_sent != packet_size)
            break;
        sent_bytes += chunk_size;
    }

    if(sent_bytes > 0)
    {
        m_stream_frames_since_log++;
        const uint32 now_ms = monotonic_ms_now();
        if((uint32)(now_ms - m_last_stream_log_ms) >= 1000U)
        {
            printf("Remote video sending data... frames=%lu/s\r\n", (unsigned long)m_stream_frames_since_log);
            m_stream_frames_since_log = 0;
            m_last_stream_log_ms = now_ms;
        }
    }

    return sent_bytes;
}

bool remote_video_tx::is_inited() const
{
    return m_inited;
}
