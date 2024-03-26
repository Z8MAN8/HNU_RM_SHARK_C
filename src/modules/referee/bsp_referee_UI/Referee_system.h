/*
* Change Logs:
* Date            Author          Notes
* 2024-01-9      ChenSihan     first version
*/
#ifndef REFEREE_SYSTEM_H
#define REFEREE_SYSTEM_H

#pragma once
#define HEADER_SOF 0xA5
#define Agreement_RX_BUF_NUM 512        //DMA要传输的数据项数目NDTR寄存器填充值 200，即收200字节后自动填充并转换缓冲数组

#define FIFO_BUF_LENGTH     1024
#ifndef SETINGS_REFEREE_SYSTEM_H
#define SETINGS_REFEREE_SYSTEM_H
#define REF_PROTOCOL_FRAME_MAX_SIZE         128
#define REF_PROTOCOL_HEADER_SIZE            sizeof(Frame_header_Typedef)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))
#include "drv_common.h"


typedef enum
{
    GAME_STATUS_CMD_ID                 = 0x0001,
    GAME_RESULT_CMD_ID                = 0x0002,
    GAME_ROBOT_HP_CMD_ID              = 0x0003,
    FIELD_EVENTS_CMD_ID               = 0x0101,
    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,
    SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,
    REFEREE_WARNING_CMD_ID            = 0x0104,
    ROBOT_STATUS_CMD_ID                = 0x0201,
    POWER_HEAT_DATA_CMD_ID            = 0x0202,
    ROBOT_POS_CMD_ID                  = 0x0203,
    BUFF_MUSK_CMD_ID                  = 0x0204,
    AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,
    ROBOT_HURT_CMD_ID                 = 0x0206,
    SHOOT_DATA_CMD_ID                 = 0x0207,
    BULLET_REMAINING_CMD_ID           = 0x0208,
    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,
    IDCustomData,
}referee_cmd_id_t;


/**
*   接收协议数据的帧头数据结构体
*/
typedef  struct
{
    uint8_t SOF;                           /*! 数据帧起始字节，固定值为 0xA5 */
    uint16_t data_length;                  /*! 数据帧中 data 的长度 */
    uint8_t seq;                           /*! 包序号 */
    uint8_t CRC8;                          /*! 帧头 CRC8校验 */
} __attribute__((__packed__)) Frame_header_Typedef;

/**
*   接收的协议数据
*/
typedef struct
{
    Frame_header_Typedef* frame_header;        /*! 帧头数据结构体 */
    uint16_t cmd_id;                          /*! 命令码 ID */
    uint8_t* data;                            /*! 接收的数据指针 */
    uint16_t frame_tail;                      /*! frame_tail */
} __attribute__((__packed__)) RX_AgreementData;


typedef enum
{
    STEP_HEADER_SOF  = 0,
    STEP_LENGTH_LOW  = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ   = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16  = 5,
} unpack_step_e;

/**
*   在单字节解包时用来码放从fifo中取出来的字节的容器，将fifo中取出来的字节拾掇成数据帧的结构
*/
typedef struct
{
    Frame_header_Typedef *p_header;
    uint16_t       data_len;
    uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
    unpack_step_e  unpack_step;
    uint16_t       index;
} unpack_data_t;
/**
*   比赛状态数据         对应的命令码ID为：0x0001
*/
typedef struct
{
    uint8_t game_type : 4;                      /*! 比赛的类型 */
    uint8_t game_progress : 4;                  /*! 当前比赛阶段 */
    uint16_t stage_remain_time;                 /*! 当前阶段剩余时间 */
    uint64_t SyncTimeStamp;                     /*! 机器人接收到该指令的精确 Unix 时间 */
} __attribute__((__packed__)) game_status_t;

/**
*   比赛结果数据         对应的命令码ID为：0x0002
*/
typedef struct
{
    uint8_t winner;                             /*! 0 平局 1 红方胜利 2 蓝方胜利 */
} __attribute__((__packed__))  ext_game_result_t;




/**
*   机器人血量数据         对应的命令码ID为：0x0003
*/
typedef struct
{
    uint16_t red_1_robot_HP;                    /*! 红 1 英雄机器人血量，未上场以及罚下血量为 0 */
    uint16_t red_2_robot_HP;                    /*! 红 2 工程机器人血量 */
    uint16_t red_3_robot_HP;                    /*! 红 3 步兵机器人血量 */
    uint16_t red_4_robot_HP;                    /*! 红 4 步兵机器人血量 */
    uint16_t red_5_robot_HP;                    /*! 红 5 步兵机器人血量 */
    uint16_t red_7_robot_HP;                    /*! 红 7 哨兵机器人血量 */
    uint16_t red_outpost_HP;                    /*! 红方前哨战血量 */
    uint16_t red_base_HP;                       /*! 红方基地血量 */
    uint16_t blue_1_robot_HP;                   /*! 蓝 1 英雄机器人血量 */
    uint16_t blue_2_robot_HP;                   /*! 蓝 2 工程机器人血量 */
    uint16_t blue_3_robot_HP;                   /*! 蓝 3 步兵机器人血量 */
    uint16_t blue_4_robot_HP;                   /*! 蓝 4 步兵机器人血量 */
    uint16_t blue_5_robot_HP;                   /*! 蓝 5 步兵机器人血量 */
    uint16_t blue_7_robot_HP;                   /*! 蓝 7 哨兵机器人血量 */
    uint16_t blue_outpost_HP;                   /*! 蓝方前哨站血量 */
    uint16_t blue_base_HP;                      /*! 蓝方基地血量 */
} __attribute__((__packed__)) ext_game_robot_HP_t;


/**
 *   人工智能挑战赛加成/惩罚区分布与潜伏模式状态        对应的命令码ID为：0x0005
 *   发送频率：1Hz 周期发送，发送范围：所有机器人
 *   激活状态： 0:未激活  1:可激活
 *   状态信息：1 为红方回血区；
 *           2 为红方弹药补给区；
 *           3 为蓝方回血区；
 *           4 为蓝方弹药补给区；
 *           5 为禁止射击区；
 *           6 为禁止移动区
 *   潜伏模式阶段：0：正常阶段；
 *           1：准备进入潜伏阶段；
 *           2：潜伏阶段
*/
typedef struct
{
    uint8_t F1_zone_status:1;                               /*! F1 激活状态 */
    uint8_t F1_zone_buff_debuff_status:3;                   /*! F1 状态信息 */
    uint8_t F2_zone_status:1;                               /*! F2 激活状态 */
    uint8_t F2_zone_buff_debuff_status:3;                   /*! F1 状态信息 */
    uint8_t F3_zone_status:1;                               /*! F3 激活状态 */
    uint8_t F3_zone_buff_debuff_status:3;                   /*! F1 状态信息 */
    uint8_t F4_zone_status:1;                               /*! F4 激活状态 */
    uint8_t F4_zone_buff_debuff_status:3;                   /*! F1 状态信息 */
    uint8_t F5_zone_status:1;                               /*! F5 激活状态 */
    uint8_t F5_zone_buff_debuff_status:3;                   /*! F1 状态信息 */
    uint8_t F6_zone_status:1;                               /*! F6 激活状态 */
    uint8_t F6_zone_buff_debuff_status:3;                   /*! F1 状态信息*/
    uint16_t red1_bullet_left;                              /*! 红方 1 号剩余弹量 */
    uint16_t red2_bullet_left;                              /*! 红方 2 号剩余弹量 */
    uint16_t blue1_bullet_left;                             /*! 蓝方 1 号剩余弹量 */
    uint16_t blue2_bullet_left;                             /*! 蓝方 2 号剩余弹量 */
    uint8_t lurk_mode;                                      /*! 潜伏模式阶段 */
    uint8_t res;                                            /*! 保留字节 */
} __attribute__((__packed__)) ext_ICRA_buff_debuff_zone_and_lurk_status_t;


/**
 *   场地事件数据。        对应的命令码ID为：：0x0101
 *   发送频率：1Hz
 *   bit 0-2：
 *   bit 0：己方补给站 1 号补血点占领状态 1 为已占领；
 *   bit 1：己方补给站 2 号补血点占领状态 1 为已占领；
 *   bit 2：己方补给站 3 号补血点占领状态 1 为已占领；
     bit 3-5：己方能量机关状态：
           • bit 3 为打击点占领状态，1 为占领；
           • bit 4 为小能量机关激活状态，1 为已激活；
           • bit 5 为大能量机关激活状态，1 为已激活；
     bit 6：己方侧 R2/B2 环形高地占领状态 1 为已占领；
     bit 7：己方侧 R3/B3 梯形高地占领状态 1 为已占领；
     bit 8：己方侧 R4/B4 梯形高地占领状态 1 为已占领；
     bit 9：己方基地护盾状态：
            • 1 为基地有虚拟护盾血量；
            • 0 为基地无虚拟护盾血量；
     bit 10：己方前哨战状态：
            • 1 为前哨战存活；
            • 0 为前哨战被击毁；
     bit 10 -31: 保留
*/
typedef struct
{
    uint32_t event_type;
} __attribute__((__packed__)) ext_event_data_t;



/**
 *   补给站动作标识         对应的命令码ID为：0x0102
 *   发送频率：动作改变后发送, 发送范围：己方机器人
 *   补给站口ID:
 *          1：1 号补给口
 *          2：2 号补给口
 *   补弹机器人 ID：
 *          0 为当前无机器人补弹，
 *          1 为红方英雄机器人补弹
 *          2 为红方工程机器人补弹
 *          3/4/5 为红方步兵机器人补弹
 *          101 为蓝方英雄机器人补弹
 *          102 为蓝方工程机器人补弹
 *          103/104/105 为蓝方步兵机器人补弹
 *    出弹口开闭状态：
 *          0 为关闭
 *          1 为子弹准备中
 *          2 为子弹下落
 *
 *    补弹数量：
 *          50：50 颗子弹；
 *          100：100 颗子弹；
 *          150：150 颗子弹；
 *          200：200 颗子弹
*/
typedef struct
{
    uint8_t supply_projectile_id;               /*! 补给站口 ID */
    uint8_t supply_robot_id;                    /*! 补弹机器人 ID */
    uint8_t supply_projectile_step;             /*! 出弹口开闭状态 */
    uint8_t supply_projectile_num;              /*! 补弹数量 */
} __attribute__((__packed__))  ext_supply_projectile_action_t;



/**
 *   裁判警告信息         对应的命令码ID为：0x0104
 *   发送频率：己方警告发生后发送
 *   警告等级：
 *          1：黄牌
 *          2：红牌
 *          3：判负
 *    犯规机器人 ID：
 *           判负时，机器人 ID 为 0
 *           黄牌、红牌时，机器人 ID 为犯规机器人 ID
*/
typedef struct
{
    uint8_t level;                              /*! 警告等级 */
    uint8_t foul_robot_id;                      /*! 犯规机器人 ID */
} __attribute__((__packed__)) ext_referee_warning_t;




/**
 *   飞镖发射口倒计时         对应的命令码ID为：0x0105
 *   发送频率：1Hz 周期发送，发送范围：己方机器人
 *
*/
typedef struct
{
    uint8_t dart_remaining_time;            /*! 15s 倒计时 */
} __attribute__((__packed__)) ext_dart_remaining_time_t;


/**
 *   比赛机器人状态         对应的命令码ID为：0x0201
 *   发送频率：10Hz
 *    本机器人 ID：
 *            1：红方英雄机器人；
 *            2：红方工程机器人；
 *            3/4/5：红方步兵机器人；
 *            6：红方空中机器人；
 *            7：红方哨兵机器人；
 *            8：红方飞镖机器人；
 *            9：红方雷达站；
 *            101：蓝方英雄机器人；
 *            102：蓝方工程机器人；
 *            103/104/105：蓝方步兵机器人；
 *            106：蓝方空中机器人；
 *            107：蓝方哨兵机器人；
 *            108：蓝方飞镖机器人；
 *            109：蓝方雷达站。
 *   主控电源输出情况：
 *            gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出；
 *            chassis 口输出：1 为有 24V 输出，0 为无 24v 输出；
 *            shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；
*/
typedef struct
{
    uint8_t robot_id;                               /*! 本机器人 ID */
    uint8_t robot_level;                            /*! 机器人等级 */
    uint16_t current_HP;                             /*! 机器人剩余血量 */
    uint16_t maximum_HP;                                /*! 机器人上限血量 */
    uint16_t shooter_barrel_cooling_value;             /*!机器人枪口热量每秒冷却值*/
    uint16_t shooter_barrel_heat_limit;                /*!机器人枪口热量上限*/
    uint16_t chassis_power_limit;                   /*! 机器人底盘功率限制上限 */
    uint8_t mains_power_gimbal_output : 1;          /*! 主控电源输出情况：gimbal 口输出 */
    uint8_t mains_power_chassis_output : 1;         /*! 主控电源输出情况：chassis 口输出 */
    uint8_t mains_power_shooter_output : 1;         /*! 主控电源输出情况：shooter 口输出 */
} __attribute__((__packed__)) robot_status_t;
#endif //SETINGS_REFEREE_SYSTEM_H


/**
 *   实时功率热量数据   对应命令码ID为：0x0202
 *   发送频率：50Hz
 *   底盘输出电压 单位 毫伏
 *   底盘输出电流 单位 毫安
 *   底盘输出功率 单位 W 瓦
 *   底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
 *   1 号 17mm 枪口热量
 *   2 号 17mm 枪口热量
 *   42mm 枪口热量
 */
typedef  struct
{
    uint16_t chassis_volt;                             /*!底盘输出电压*/
    uint16_t chassis_current;                          /*!底盘输出电流*/
    float chassis_power;                               /*!底盘输出功率*/
    uint16_t chassis_power_buffer;                     /*!底盘功率缓存*/
    uint16_t shooter_id1_17mm_cooling_heat;            /*!机器人1 号 17mm 枪口热量*/
    uint16_t shooter_id2_17mm_cooling_heat;            /*!机器人2 号 17mm 枪口热量*/
    uint16_t shooter_id1_42mm_cooling_heat;            /*!机器人42mm 枪口热量*/
} __attribute__((__packed__))ext_power_heat_data_t;


/**
 * 机器人位置    对应命令码ID为：0x0203
 * 发送频率：10Hz
 * 位置 x 坐标，单位 m
 * 位置 y 坐标，单位 m
 * 位置 z 坐标，单位 m
 * 位置枪口，单位度
 */
typedef  struct
{
    float x;                               /*!位置 x 坐标*/
    float y;                               /*!位置 y 坐标*/
    float z;                               /*!位置 z 坐标*/
    float yaw;                             /*!枪口位置*/
} __attribute__((__packed__))ext_game_robot_pos_t;


/**
* 机器人增益  对应命令码ID为：0x0204
* 发送频率：1Hz
* 机器人血量补血状态
* 枪口热量冷却加速
* 机器人防御加成
* 机器人攻击加成
* 其他 bit 保留
*/
typedef  struct
{
    uint8_t power_rune_buff;               /*!机器人增益*/
}__attribute__((__packed__))ext_buff_t;


/**
* 空中机器人能量状态 对应命令码ID为：0x0205
* 发送频率：10Hz
* 可攻击时间 单位 s 30s 递减至 0
*/
typedef  struct
{
    uint8_t attack_time;                /*!可攻击时间*/
} __attribute__((__packed__))aerial_robot_energy_t;

/**
*  伤害状态  对应命令码ID为：0x0206
*  发送频率：伤害发生后发送
*  bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的
*  五个装甲片，其他血量变化类型，该变量数值为 0。
*  bit 4-7：血量变化类型
*  0x0 装甲伤害扣血；
*  0x1 模块掉线扣血；
*  0x2 超射速扣血；
*  0x3 超枪口热量扣血；
*  0x4 超底盘功率扣血；
*  0x5 装甲撞击扣血
*/
typedef  struct
{
    uint8_t armor_id : 4;                    /*!装甲ID */
    uint8_t hurt_type : 4;                   /*!血量变化类型*/
}  __attribute__((__packed__))ext_robot_hurt_t;


/**
*  实时射击信息   对应命令码ID为：0x0207
*  发送频率：射击后发送
*  子弹类型: 1：17mm 弹丸 2：42mm 弹丸
*  发射机构 ID：
*  1：1 号 17mm 发射机构
*  2：2 号 17mm 发射机构
*  3：42mm 发射机构
* 子弹射频 单位 Hz
* 子弹射速 单位 m/s
*/
typedef struct
{
    uint8_t bullet_type;                 /*!子弹类型*/
    uint8_t shooter_id;                  /*!发射枪口ID*/
    uint8_t bullet_freq;                 /*!子弹射频*/
    float bullet_speed;                  /*!子弹射速*/
} __attribute__((__packed__))ext_shoot_data_t;


/**
*  子弹剩余发射数    对应命令码ID为：0x0208
*  发送频率：10Hz 周期发送，所有机器人发送
*   17mm 子弹剩余发射数量含义说明             联盟赛                              对抗赛
*    步兵机器人                   全队步兵与英雄剩余可发射 17mm 弹丸总量          全队 17mm 弹丸剩余可兑换数量
*    英雄机器人                   全队步兵与英雄剩余可发射 17mm 弹丸总量          全队 17mm 弹丸剩余可兑换数量
*    空中机器人、哨兵机器人         该机器人剩余可发射 17mm 弹丸总量               该机器人剩余可发射 17mm 弹丸总量
*
*   17mm 子弹剩余发射数目
*   42mm 子弹剩余发射数目
*   剩余金币数量
*/
typedef  struct
{
    uint16_t bullet_remaining_num_17mm;                 /*!17mm弹头剩余数量*/
    uint16_t bullet_remaining_num_42mm;                 /*!17mm弹头剩余数量*/
    uint16_t coin_remaining_num;                        /*!金币剩余数量*/
}  __attribute__((__packed__))ext_bullet_remaining_t;

/**
*  机器人 RFID 状态  对应命令码ID为：0x0209
*  发送频率：1Hz    发送范围：单一机器人
*  bit 0：基地增益点 RFID 状态；
*  bit 1：高地增益点 RFID 状态；
*  bit 2：能量机关激活点 RFID 状态；
*  bit 3：飞坡增益点 RFID 状态；
*  bit 4：前哨岗增益点 RFID 状态；
*  bit 6：补血点增益点 RFID 状态；
*  bit 7：工程机器人复活卡 RFID 状态；
*  bit 8-31：保留
*/
typedef  struct
{
    uint32_t rfid_status;                        /*!机器人RFID状态 */
}  __attribute__((__packed__))ext_rfid_status_t;

/**
* 飞镖机器人客户端指令数据   对应命令码ID为：0x020A
* 发送频率：10Hz 发送范围：单一机器人
*  当前飞镖发射口的状态
*  1：关闭；
*  2：正在开启或者关闭中
*  0：已经开启
* 飞镖的打击目标，默认为前哨站；
*  0：前哨站；
*  1：基地。
*  切换打击目标时的比赛剩余时间，单位秒，从未切换默认为 0。
*  最近一次操作手确定发射指令时的比赛剩余时间，单位秒, 初始值为 0。
*/
typedef  struct
{
    uint8_t dart_launch_opening_status;             /*!飞镖发射口状态*/
    uint8_t dart_attack_target;                     /*!飞镖打击目标*/
    uint16_t target_change_time;                    /*!切换打击目标时的比赛剩余时间*/
    uint16_t operate_launch_cmd_time;               /*!最近一次操作手确定发射指令时的比赛剩余时间*/
}  __attribute__((__packed__))ext_dart_client_cmd_t;

/**
 *  交互数据接收信息   对应命令码ID为：0x0301
 * 发送者的 ID:
 *      需要校验发送者的 ID 正确性，例如红 1 发送给红 5，此项需要校验红 1
 * 接收者的 ID:
 *      需要校验接收者的 ID 正确性，例如不能发送到敌对机器人的 ID
 * 内容数据段:
 *  最大为 113
*/
typedef  struct
{
    uint16_t send_ID;                               /*! 数据段的内容 ID */
    uint16_t receiver_ID;                           /*! 发送者的 ID */
    uint16_t data_cmd_id;                           /*! 接收者的 ID */
    uint8_t *data;                                  /*! 内容数据段 */
} __attribute__((__packed__)) ext_student_interactive_header_data_t;

/**
 * @brief 裁判系统接收初始化
 */
void Referee_system_Init(uint8_t *  rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
/**
 * @brief 裁判系统接收数据帧解包
 */
void Referee_Data_Unpack();
/**
 * @brief 裁判系统数据更新并保存
 */
void Referee_Data_Solve(uint8_t* referee_data_frame);

#endif //REFEREE_SYSTEM_H