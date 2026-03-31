#ifndef __PYRO_PYRO_UART_MESSAGE_H__
#define __PYRO_PYRO_UART_MESSAGE_H__

#include <cstdint>

struct frame_header
{
    uint8_t sof;
} __attribute__((packed));

struct frame_tailer
{
    uint16_t crc16;
} __attribute__((packed));

struct frame_enter
{
    uint8_t enter;
} __attribute__((packed));

#if ROBOT_ID == SENTRY_ID
typedef struct
{
    float vx;
    float vy;
    float vz;
    float wz;
    float imu;
    uint8_t stuck;
    uint8_t mode; // 1为进攻，2为防御，3为移动
} __attribute__((packed)) nav2mcu_data_t;

typedef struct
{
    float shoot_yaw;
    float shoot_yaw_speed;
    float shoot_yaw_acceleration;
    float shoot_pitch;
    float shoot_pitch_speed;
    float shoot_pitch_acceleration;
    uint8_t fire           : 1;
    uint8_t is_single_shot : 1;
    uint8_t target_id      : 6;
    uint8_t aim_state;
} __attribute__((packed)) aim2mcu_data_t;

typedef struct
{
    float curr_yaw;
    float curr_pitch;
    float self_v_magnitude;
    float self_v_angle;
    float curr_speed;
    uint8_t shoot_delay;
    uint8_t state       : 5;
    uint8_t stop_record : 1;
    uint8_t autoaim     : 1;
    uint8_t enemy_color : 1;
} __attribute__((packed)) mcu2aim_data_t;

typedef struct
{
    uint16_t self_hp;
    uint16_t self_ammo;
    uint8_t game_state;
    uint16_t self_base_hp;
    uint16_t self_outpost_hp;
    uint16_t game_time;
} __attribute__((packed)) mcu2nav_data_t;

typedef struct
{
    frame_header header;
    nav2mcu_data_t data;
    frame_tailer tailer;
} __attribute__((packed)) nav2mcu_msg_t;

typedef struct
{
    frame_header header;
    aim2mcu_data_t data;
    frame_tailer tailer;
} __attribute__((packed)) aim2mcu_msg_t;

typedef struct
{
    frame_header header{};
    mcu2aim_data_t data{};
    frame_tailer tailer{};
    frame_enter enter;
} __attribute__((packed)) mcu2aim_msg_t;

typedef struct
{
    frame_header header;
    mcu2nav_data_t data;
    frame_tailer tailer;
} __attribute__((packed)) mcu2nav_msg_t;

typedef struct
{

} __attribute__((packed)) handshake_protocol_t;

#endif


#endif // PYRO_PYRO_UART_MESSAGE_H
