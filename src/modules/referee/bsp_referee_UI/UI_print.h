//
// Created by 16933 on 2022/3/15.
//
#include "stdio.h"
#define ARobot_ID UI_Data_RobotID_RStandard3
#define ACilent_ID UI_Data_CilentID_RStandard3
/****************************数据帧起始字节，为固定值********************/
#define UI_SOF 0xA5
/****************************cmd_id********************/
#define UI_CMD_Robo_Exchange 0x0301
/****************************数据帧帧头数据段内容ID*********************/
#define Client_delete_graphics_ID 0x0100
#define Client_draws_one_graph 0x0101
#define Client_draws_two_graph 0x0102
#define Client_draws_five_graph 0x0103
#define Client_draws_seven_graph 0x0104
#define Client_draws_character 0x0110
/****************************不同数据的datalength*********************/
#define DeleteGraph 8
#define DrawOneGraph 21
#define DrawTwoGraph 36
#define DrawFiveGraph 81
#define DrawSevenGraph 111
#define DrawCharacter 51
/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero 1
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A



/***************************绘图类型**************************/
#define LINE             0
#define RECTANGLE        1
#define CIRCLE           2
#define ELLIPSE           3
#define ARC              4
#define NONE             0

typedef   struct
{
    uint8_t SOF;                    //起始字节,固定0xA5
    uint16_t Data_Length;           //帧数据长度
    uint8_t Seq;                    //包序号
    uint8_t CRC8;                   //CRC8校验值
    uint16_t CMD_ID;                //命令ID
}__attribute__((__packed__))  Head; //数据帧头


typedef  struct
{
    uint16_t data_cmd_id;             //数据段内容ID
    uint16_t sender_ID;
    uint16_t receiver_ID;
}__attribute__((__packed__)) ext_student_interactive_header_data_t_tx;  //数据帧帧头



#ifndef SETINGS_UI_PRINT_H
#define SETINGS_UI_PRINT_H
#include "stdio.h"

//图形数据
typedef  struct
{
    uint8_t graphic_name[3];    /* 图形名：在删除，修改等操作中，作为客户端的索引。  */
    uint32_t operate_tpye:3;      //bit 0-2：图形操作：0：空操作；1：增加；2：修改；3：删除；
    uint32_t graphic_tpye:3;      //Bit 3-5：图形类型：0：直线；1：矩形；2：整圆；3：椭圆；4：圆弧；5：浮点数；6：整型数；7：字符；
    uint32_t layer:4;            //Bit 6-9：图层数，0~9
    uint32_t color:4;            //Bit 10-13：颜色：0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
    uint32_t start_angle:9;     //Bit 14-22：起始角度，单位：°，范围[0,360]；
    uint32_t end_angle:9;       //Bit 23-31：终止角度，单位：°，范围[0,360]。
    uint32_t width:10;          //Bit 0-9：线宽；
    uint32_t start_x:11;        //Bit 10-20：起点 x 坐标；
    uint32_t start_y:11;       //Bit 21-31：起点 y 坐标。
    uint32_t radius:10;        //Bit 0-9：字体大小或者半径；
    uint32_t end_x:11;         //Bit 10-20：终点 x 坐标；
    uint32_t end_y:11;        // Bit 21-31：终点 y 坐标。
} __attribute__((__packed__)) graphic_data_struct_t;



//客户端删除图形 机器人间通信：0x0301
typedef struct
{
    uint8_t operate_tpye;      //图形操作 0: 空操作；1: 删除图层；2: 删除所有；
    uint8_t layer;             //图层数  图层数：0~9
}  __attribute__((__packed__)) ext_client_custom_graphic_delete_t;




//表 5-4 客户端绘制一个图形 机器人间通信：0x0301
typedef  struct
{
    graphic_data_struct_t grapic_data_struct;         //图形 1
} __attribute__((__packed__)) ext_client_custom_graphic_single_t;





//表 5-5 客户端绘制二个图形 机器人间通信：0x0301
typedef  struct
{
    graphic_data_struct_t grapic_data_struct[2];//图形1，图形2
} __attribute__((__packed__)) ext_client_custom_graphic_double_t;




//表 5-6 客户端绘制五个图形 机器人间通信：0x0301
typedef  struct
{
    graphic_data_struct_t grapic_data_struct[5];//图形1-5
} __attribute__((__packed__)) ext_client_custom_graphic_five_t;





//表 5-7 客户端绘制七个图形 机器人间通信：0x0301
typedef  struct
{
    graphic_data_struct_t grapic_data_struct[7];  //图形1-7
} __attribute__((__packed__)) ext_client_custom_graphic_seven_t;





//表 5-8 客户端绘制字符 机器人间通信：0x0301
typedef  struct
{
    graphic_data_struct_t grapic_data_struct;   //字符配置
    uint8_t data[30];                          //字符
} __attribute__((__packed__)) ext_client_custom_character_t;

typedef  struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;
    uint32_t graphic_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t data;
} __attribute__((__packed__)) int_data_struct_t;  //数学数据数据帧（整形、浮点型）



typedef struct
{
    Head * head;
    ext_student_interactive_header_data_t_tx * datahead ;
    graphic_data_struct_t * UI_Image_Data;      //图像数据存放地址
    int_data_struct_t     * UI_Int_Float_Data;  //整数、浮点数数据存放地址
    ext_client_custom_character_t * UI_String_Data; //字符串数据存放地址
    uint8_t * senddata;         //要发送的数据的地址
    uint32_t size;              //要发送的数据的总大小
    uint16_t frequency_cnt;          //要发送的频率计数器（频率/10）
    uint16_t frequency;
    int seqnum;                 //排列序号
    int fresh_count;            //用来计数、控制刷新和添加操作
    uint8_t FOA;                //刷新或者添加控制位  1：添加  2：刷新
    int datelength;           //数据长度
    uint8_t type;             //用来辨别 0 ：是图像  1 ：整数浮点数  2 ：字符串
}UI_TCB;
void DeleteUI(ext_client_custom_graphic_delete_t * data,uint8_t Delete_Operate,uint8_t Layer);
void DrawLine(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y);
void DrawRectangle(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y);
void DrawFullCircle(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t radius);
void DrawString(ext_client_custom_character_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t width,uint32_t start_x,uint32_t start_y,uint8_t chardata[30]);
void DrawInteger(int_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t intdata);
//void DrawFloat(int_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t digits,uint32_t width,uint32_t start_x,uint32_t start_y,uint16_t float_high,uint16_t float_middle,uint16_t float_low);
void DrawFloat(int_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t digits,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t floatdata);
void DrawARC(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_angle,uint32_t end_angle,uint32_t start_x,uint32_t start_y,uint32_t X_axle_length,uint32_t Y_axle_length);
void DrawEllipse(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_angle,uint32_t end_angle,uint32_t start_x,uint32_t start_y,uint32_t X_axle_length,uint32_t Y_axle_length);



void SendPredictData(graphic_data_struct_t data,uint32_t End_x,uint32_t End_y);
void SendStringData(ext_client_custom_character_t *imageData);
void SendData(graphic_data_struct_t *imageData);
void SendIntData(int_data_struct_t *imageData);
void SendDeleteData(ext_client_custom_graphic_delete_t data);
void SendFloatData(int_data_struct_t *imageData);

void uiDrawGraphics(uint8_t graphics_type,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_angle,uint32_t end_angle,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y,uint32_t radius);
void uiDrawIntData(char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t intdata);
void uiDrawStringData(char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t width,uint32_t start_x,uint32_t start_y,uint8_t chardata[30]);
void uiDeleteGraphic(uint8_t Delete_Operate,uint8_t Layer);
void uiDrawFloatData(char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t digits,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t floatdata);

UI_TCB* uiDrawIntDataSeq(char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t intdata,int frequency);
UI_TCB* uiDrawStringDataSeq(char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t width,uint32_t start_x,uint32_t start_y,uint8_t *chardata,int frequency);
UI_TCB* uiDrawGraphicsSeq(uint8_t graphics_type,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_angle,uint32_t end_angle,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y,uint32_t radius,int frequency);
UI_TCB* uiDrawFloatDataSeq(char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t digits,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t floatdata,int frequency);


void uiRefreshIntDataSeq(UI_TCB* TargetTCB , UI_TCB* NowTCB,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t intdata,int frequency);
void uiRefreshFloatDataSeq(UI_TCB* TargetTCB , UI_TCB* NowTCB,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t digits,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t floatdata,int frequency);
void uiRefreshGraphicsSeq(UI_TCB* TargetTCB,UI_TCB* NowTCB,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_angle,uint32_t end_angle,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y,uint32_t radius,int frequency);
void uiRefreshStringDataSeq(UI_TCB* TargetTCB,UI_TCB* NowTCB,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t width,uint32_t start_x,uint32_t start_y,uint8_t *chardata,int frequency);


void UI_Draw(uint8_t* image,uint16_t size);
void UI_Drawnew(UI_TCB * imageTCB);
#endif //SETINGS_UI_PRINT_H
