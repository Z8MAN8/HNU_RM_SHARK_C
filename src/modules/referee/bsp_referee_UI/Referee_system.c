/*
* Change Logs:
* Date            Author          Notes
* 2024-01-9      ChenSihan     first version
*/

#include "Referee_system.h"
#include "string.h"
#include "fifo.h"
#include "BSP_CRC.h"

fifo_s_t RX_AgreementData_FIFO;           //实例化的裁判系统接收数据FIFO容器
uint8_t RX_FIFO_Space[FIFO_BUF_LENGTH];              //FIFO的实际存储区�?
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
Frame_header_Typedef Referee_Data_Header;   //接收数据帧头结构体
uint8_t RX_AgreementData_Buffer0[Agreement_RX_BUF_NUM];   //接收裁判系统返回数据的接收缓冲区0,该缓冲区设置的相当富裕
uint8_t RX_AgreementData_Buffer1[Agreement_RX_BUF_NUM];   //接收裁判系统返回数据的接收缓冲区1，该缓冲区设置的相当富裕
uint8_t RX_Agreement_Data[Agreement_RX_BUF_NUM];          //用来单独存放接收数据的Data段
unpack_data_t referee_unpack_obj;

/*!结构体实例化*/
ext_game_status_t                       game_status;
ext_game_result_t                       game_result;
ext_game_robot_HP_t                     game_robot_HP_t;
ext_event_data_t                        field_event;
ext_supply_projectile_action_t          supply_projectile_action_t;
ext_referee_warning_t                   referee_warning_t;
ext_game_robot_status_t                 robot_status;
ext_power_heat_data_t                   power_heat_data_t;
ext_game_robot_pos_t                    game_robot_pos_t;
ext_buff_t                              buff_musk_t;
aerial_robot_energy_t                   robot_energy_t;
ext_robot_hurt_t                        robot_hurt_t;
ext_shoot_data_t                        shoot_data_t;
ext_bullet_remaining_t                  bullet_remaining_t;
ext_student_interactive_header_data_t   student_interactive_data_t;
Frame_header_Typedef Referee_Data_header;         //实例化一个帧头结构体
RX_AgreementData     Referee_Data;                //实例化一个数据帧结构体

/**
 *@brief 裁判系统初始化串口通信以及实例化结构体
 * @param rx1_buf       设定的接收裁判系统返回数据的接收缓冲区1
 * @param rx2_buf       设定的接收裁判系统返回数据的接收缓冲区2
 * @param dma_buf_num   DMA转送数据的空间大小
 *
 */
void Referee_system_Init(uint8_t *  rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    memset(&Referee_Data_header, 0, sizeof(Frame_header_Typedef));
    memset(&Referee_Data, 0, sizeof(RX_AgreementData));

    memset(&game_status, 0, sizeof(ext_game_status_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));


    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));


    memset(&robot_status, 0, sizeof(ext_game_robot_status_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(ext_buff_t));
    memset(&robot_energy_t, 0, sizeof(aerial_robot_energy_t));
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));
    memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_header_data_t));
    //使能DMA串口接收
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);

    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    //失效DMA，并等待直至SxCR_EN寄存器置0，以保证后续的配置数据可以写入
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart6_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
    //fifo初始化
    fifo_s_init(&RX_AgreementData_FIFO,RX_FIFO_Space,FIFO_BUF_LENGTH);    //创建FIFO存储区域

}

/**
 * @brief 裁判系统数据解包函数
 */

void Referee_Data_Unpack()
{
    unpack_data_t *p_obj = &referee_unpack_obj;
    uint8_t byte = 0;
    uint8_t sof = HEADER_SOF;
    while(fifo_s_used(&RX_AgreementData_FIFO))
    {
        byte = fifo_s_get(&RX_AgreementData_FIFO);
        switch (p_obj->unpack_step)  //状态转换机
        {
            case STEP_HEADER_SOF:      //如果是读取帧头SOF的状态
            {
                if(byte == sof)       //判断是否为SOF
                {
                    p_obj->unpack_step = STEP_LENGTH_LOW;       //改变状态，下次拿出来的byte，去试图照应数据长度的低八位
                    p_obj->protocol_packet[p_obj->index++] = byte;  //将数据码好，并将索引长度加1
                }
                else
                {
                    p_obj->index = 0;   //如果不是，就再从fifo中拿出来一个byte，继续读，直到读出来一个sof
                }
            }break;
            case STEP_LENGTH_LOW:       //如果目前的状态是读的数据长度的低八位
            {
                p_obj->data_len = byte;           //低八位直接放入
                p_obj->protocol_packet[p_obj->index++] = byte;   //码好数据
                p_obj->unpack_step = STEP_LENGTH_HIGH;          //转变状态
            }break;

            case STEP_LENGTH_HIGH:  //如果目前的状态时读数据长度的高八位
            {
                p_obj->data_len |= (byte << 8);     //放入data_len的高八位
                p_obj->protocol_packet[p_obj->index++] = byte;  //码好数据
                //整个交互数据的包总共长最大为 128 个字节，减去 frame_header,cmd_id 和 frame_tail 共 9 个字节以及数据段头结构的 6 个字节
                if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
                {
                    p_obj->unpack_step = STEP_FRAME_SEQ;        //转变状态，下一个该读包序号
                }
                else
                {
                    //如果数据长度不合法，就重头开始读取，并且之前码好的数据作废
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            }break;
            case STEP_FRAME_SEQ:
            {
                p_obj->protocol_packet[p_obj->index++] = byte;  //码好数据
                p_obj->unpack_step = STEP_HEADER_CRC8;          //转换状态，下一个byte读的是CRC8
            }break;

            case STEP_HEADER_CRC8:
            {
                //先将这一byte数据放入，使帧头结构完整，以便后面可以进行CRC校验
                p_obj->protocol_packet[p_obj->index++] = byte;
                //如果这一byte放入之后，数据长度是一个帧头的长度，那么就进行CRC校验
                if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
                {
                    if ( Verify_CRC8_Check_Sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
                    {
                        p_obj->unpack_step = STEP_DATA_CRC16;   //如果校验通过，则状态转换成去读取帧尾
                    }
                    else
                    {
                        //如果校验不通过，则从头开始，之前码好的数据作废
                        p_obj->unpack_step = STEP_HEADER_SOF;
                        p_obj->index = 0;
                    }
                }
            }break;

            case STEP_DATA_CRC16:
            {
                //从帧头到帧尾的过程中的数据一律码好
                if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
                {
                    p_obj->protocol_packet[p_obj->index++] = byte;
                }
                //如果数据读取到data末尾，则转换状态，准备开始新一帧的读取
                if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
                {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                    //整包数据校验
                    if ( Verify_CRC16_Check_Sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
                    {
                        //校验通过，则将码好的数据memcp到指定结构体中
                        Referee_Data_Solve(p_obj->protocol_packet);
                    }
                }
            }break;

        }
    }
}


/**
 * @brief 裁判系统命令数据解包函数
 * @param referee_data_frame: 接收到的整帧数据
 */
void Referee_Data_Solve(uint8_t* frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;
    memcpy(&Referee_Data_header, frame, sizeof(Frame_header_Typedef));
    index += sizeof(Frame_header_Typedef);
    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id) {
        case GAME_STATUS_CMD_ID:
            memcpy(&game_status, frame + index, sizeof(ext_game_status_t));
            break;
        case GAME_RESULT_CMD_ID:
            memcpy(&game_result, frame + index, sizeof(ext_game_result_t));
            break;
        case GAME_ROBOT_HP_CMD_ID:
            memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
            break;
        case FIELD_EVENTS_CMD_ID:
            memcpy(&field_event, frame + index, sizeof(ext_event_data_t));
            break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
            memcpy(&supply_projectile_action_t, frame + index, sizeof(ext_supply_projectile_action_t));
            break;
        case REFEREE_WARNING_CMD_ID:
            memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
            break;
        case ROBOT_STATUS_CMD_ID:
            memcpy(&robot_status, frame + index, sizeof(ext_game_robot_status_t));
            break;
        case POWER_HEAT_DATA_CMD_ID:
            memcpy(&power_heat_data_t, frame + index, sizeof(ext_power_heat_data_t));
            break;
        case ROBOT_POS_CMD_ID:
            memcpy(&game_robot_pos_t, frame + index, sizeof(ext_game_robot_pos_t));
            break;
        case BUFF_MUSK_CMD_ID:
            memcpy(&buff_musk_t, frame + index, sizeof(ext_buff_t));
            break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
            memcpy(&robot_energy_t, frame + index, sizeof(aerial_robot_energy_t));
            break;
        case ROBOT_HURT_CMD_ID:
            memcpy(&robot_hurt_t, frame + index, sizeof(ext_robot_hurt_t));
            break;
        case SHOOT_DATA_CMD_ID:
            memcpy(&shoot_data_t, frame + index, sizeof(ext_shoot_data_t));
            break;
        case BULLET_REMAINING_CMD_ID:
            memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
            break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
            memcpy(&student_interactive_data_t, frame + index, sizeof(ext_student_interactive_header_data_t));
            break;
    }
}







