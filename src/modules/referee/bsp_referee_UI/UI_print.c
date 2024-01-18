#include "UI_print.h"
#include "math.h"
//#include "String.h"
#include "stm32f4xx.h"
#include "BSP_CRC.h"
#include "string.h"
#include "stdlib.h"
#include <rtthread.h>
uint8_t SendDataBuffer[256];

extern UART_HandleTypeDef huart6;

uint8_t Seq=1;
UI_TCB* UI_SendTCBSequence[30];
int UI_TCBNum = 0;

/*!-----------------------------基础调用函数---------------------------*/
/**
* @brief          删除某一图层的UI
* @param[in]      Delete_Operate: 图形操作 其中0：空操作 1：删除图层 2：删除所有
* @param[in]      Layer: 图层数范围为0-9
* @retval         none
*/
void DeleteUI(ext_client_custom_graphic_delete_t * data,uint8_t Delete_Operate,uint8_t Layer){
//    ext_client_custom_graphic_delete_t data;
    data->layer=Layer;
    data->operate_tpye=Delete_Operate;
}

/**
  * @brief          填充指定结构体变量的内容为画一条直线
  * @param[in]      data: 要被操作的结构体的地址
  * @param[in]      graphname: 图像名称，作为删除、修改等操作客户端的索引
  * @param[in]      operate_tpye：图形操作 其中0:空操作 1:增加 2:修改 3:删除
	* @param[in]      layer：图层数范围为0-9
	* @param[in]      color：颜色0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
	* @param[in]      width：线条宽度
	* @param[in]      start_x：起点 x 坐标
	* @param[in]      start_y：起点 y 坐标
	* @param[in]      end_x：终点 x坐标
	* @param[in]      end_y：终点 y坐标
  * @retval         none
  */
void DrawLine(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y){
    //memset(&data,0,sizeof(data));
    data->graphic_name[0]=graphname[0];
    data->graphic_name[1]=graphname[1];
    data->graphic_name[2]=graphname[2];
    data->operate_tpye=operate_tpye;
    data->graphic_tpye=0;
    data->layer=layer;
    data->color=color;
    data->width=width;
    data->start_x=start_x;
    data->start_y=start_y;
    data->end_x=end_x;
    data->end_y=end_y;
}



/**
  * @brief          填充指定结构体变量的内容为画一个矩形
  * @param[in]      data: 要被操作的结构体的地址
  * @param[in]      graphname: 图像名称，作为删除、修改等操作客户端的索引
  * @param[in]      operate_tpye：图形操作 其中0:空操作 1:增加 2:修改 3:删除
	* @param[in]      layer：图层数范围为0-9
	* @param[in]      color：颜色0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
	* @param[in]      width：线条宽度
	* @param[in]      start_x：起点 x 坐标
	* @param[in]      start_y：起点 y 坐标
	* @param[in]      end_x：对角顶点 x坐标
	* @param[in]      end_y：对角顶点 y坐标
  * @retval         none
  */
void DrawRectangle(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y){
    //memset(&data,0,sizeof(data));
    data->graphic_name[0]=graphname[0];
    data->graphic_name[1]=graphname[1];
    data->graphic_name[2]=graphname[2];
    data->operate_tpye=operate_tpye;
    data->graphic_tpye=1;
    data->layer=layer;
    data->color=color;
    data->width=width;
    data->start_x=start_x;
    data->start_y=start_y;
    data->end_x=end_x;
    data->end_y=end_y;
}


/**
  * @brief          填充指定结构体变量的内容为画一个整圆
  * @param[in]      data: 要被操作的结构体的地址
  * @param[in]      graphname: 图像名称，作为删除、修改等操作客户端的索引
  * @param[in]      operate_tpye：图形操作 其中0:空操作 1:增加 2:修改 3:删除
	* @param[in]      layer：图层数范围为0-9
	* @param[in]      color：颜色0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
	* @param[in]      width：线条宽度
	* @param[in]      start_x：圆心 x 坐标
	* @param[in]      start_y：圆心 y 坐标
	* @param[in]      radius：半径
  * @retval         none
  */
void DrawFullCircle(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,
                    uint32_t color,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t radius){
    //memset(&data,0,sizeof(data));
    data->graphic_name[0]=graphname[0];
    data->graphic_name[1]=graphname[1];
    data->graphic_name[2]=graphname[2];
    data->operate_tpye=operate_tpye;
    data->graphic_tpye=2;
    data->layer=layer;
    data->color=color;
    data->width=width;
    data->start_x=start_x;
    data->start_y=start_y;
    data->radius=radius;
}



/**
  * @brief          填充指定结构体变量的内容为画字符串
  * @param[in]      data: 要被操作的结构体的地址
  * @param[in]      graphname: 图像名称，作为删除、修改等操作客户端的索引
  * @param[in]      operate_tpye：图形操作 其中0:空操作 1:增加 2:修改 3:删除
	* @param[in]      layer：图层数范围为0-9
	* @param[in]      color：颜色0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
	* @param[in]      font_size：字体大小
	* @param[in]      end_angle：字符长度
	* @param[in]      width：线条宽度
	* @param[in]      start_x：起点 x 坐标
	* @param[in]      start_y：起点 y 坐标
	* @param[in]      chardata：字符数据
  * @retval         none
  */
void DrawString(ext_client_custom_character_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t width,uint32_t start_x,uint32_t start_y,uint8_t chardata[30]){
    int i=0;
    while(chardata[i]!='\0'){
        data->data[i]=chardata[i];
        i++;
    }
//    memcpy(&(data->data),chardata,sizeof(*chardata));
//    chardata[i+1]='\0';
    data->grapic_data_struct.graphic_name[0]=graphname[0];
    data->grapic_data_struct.graphic_name[1]=graphname[1];
    data->grapic_data_struct.graphic_name[2]=graphname[2];
    data->grapic_data_struct.operate_tpye=operate_tpye;
    data->grapic_data_struct.graphic_tpye=7;
    data->grapic_data_struct.layer=layer;
    data->grapic_data_struct.color=color;
    data->grapic_data_struct.start_angle=font_size;
    data->grapic_data_struct.end_angle=i;
    data->grapic_data_struct.width=width;
    data->grapic_data_struct.start_x=start_x;
    data->grapic_data_struct.start_y=start_y;
}




/**
  * @brief          填充指定结构体变量的内容为画一个整数
  * @param[in]      data: 要被操作的结构体的地址
  * @param[in]      graphname: 图像名称，作为删除、修改等操作客户端的索引
  * @param[in]      operate_tpye：图形操作 其中0:空操作 1:增加 2:修改 3:删除
	* @param[in]      layer：图层数范围为0-9
	* @param[in]      color：颜色0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
 * @param           font_size：字体大小
	* @param[in]      width：线条宽度
	* @param[in]      start_x：起点 x 坐标
	* @param[in]      start_y：起点 y 坐标
	* @param[in]      end_x：终点 x坐标
	* @param[in]      end_y：终点 y坐标
  * @retval         none
  */
void DrawInteger(int_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t intdata){
    //memset(&data,0,sizeof(data));
    data->graphic_name[0]=graphname[0];
    data->graphic_name[1]=graphname[1];
    data->graphic_name[2]=graphname[2];
    data->operate_tpye=operate_tpye;
    data->graphic_tpye=6;
    data->layer=layer;
    data->color=color;
    data->start_angle=font_size;
    data->end_angle=0;
    data->width=width;
    data->start_x=start_x;
    data->start_y=start_y;
    data->data=intdata;
}



/**
 * @brief 填充指定结构体变量的内容为画一个浮点数
 * @param data: 要被操作的结构体的地址
 * @param operate_tpye
 * @param layer
 * @param color
 * @param font_size
 * @param digits 小数点后有效位数
 * @param width
 * @param start_x
 * @param start_y
 * @param float_high 32位浮点数
 * @param float_middle 32位浮点数
 * @param float_low 32位浮点数
 */
//void DrawFloat(int_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t digits,uint32_t width,uint32_t start_x,uint32_t start_y,uint16_t float_high,uint16_t float_middle,uint16_t float_low)
void DrawFloat(int_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t digits,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t floatdata)
{
    //memset(&data,0,sizeof(data));
    data->graphic_name[0]=graphname[0];
    data->graphic_name[1]=graphname[1];
    data->graphic_name[2]=graphname[2];
    data->operate_tpye=operate_tpye;
    data->graphic_tpye=5;
    data->layer=layer;
    data->color=color;
    data->start_angle=font_size;
    data->end_angle=digits;
    data->width=width;
    data->start_x=start_x;
    data->start_y=start_y;
//    memcpy(&(data->data),&float_low, sizeof(float_low));
//    memcpy(&(data->data)+sizeof(float_low),&float_middle, sizeof(float_middle));
//    memcpy(&(data->data)+sizeof(float_low)+sizeof(float_middle),&float_high, sizeof(float_high));
    data->data = floatdata;
}

/**
 * @brief 填充指定结构体变量的内容为画一个圆弧
 * @param data: 要被操作的结构体的地址
 * @param operate_tpye
 * @param layer
 * @param color
 * @param font_size
 * @param width
 * @param start_angle:起始角度
 * @param end_angle：终止角度
 * @param X_axle_length：x半轴长度
 * @param Y_axle_length：y半轴长度
 */
void DrawARC(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,
             uint32_t color,uint32_t width,uint32_t start_angle,uint32_t end_angle,uint32_t start_x,uint32_t start_y,uint32_t X_axle_length,uint32_t Y_axle_length)
{
    data->graphic_name[0]=graphname[0];
    data->graphic_name[1]=graphname[1];
    data->graphic_name[2]=graphname[2];
    data->operate_tpye=operate_tpye;
    data->graphic_tpye=4;
    data->layer=layer;
    data->color=color;
    data->width=width;
    data->start_angle=start_angle;
    data->end_angle=end_angle;
    data->start_x=start_x;
    data->start_y=start_y;
    data->end_x=X_axle_length;
    data->end_y=Y_axle_length;
}

/**
 * @brief 填充指定结构体变量的内容为画一个椭圆
 * @param data: 要被操作的结构体的地址
 * @param operate_tpye
 * @param layer
 * @param color
 * @param font_size
 * @param width
 * @param start_angle:起始角度
 * @param end_angle：终止角度
 * @param X_axle_length：x半轴长度
 * @param Y_axle_length：y半轴长度
 */
void DrawEllipse(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,
                 uint32_t color,uint32_t width,uint32_t start_angle,uint32_t end_angle,uint32_t start_x,uint32_t start_y,uint32_t X_axle_length,uint32_t Y_axle_length)
{
    data->graphic_name[0]=graphname[0];
    data->graphic_name[1]=graphname[1];
    data->graphic_name[2]=graphname[2];
    data->operate_tpye=operate_tpye;
    data->graphic_tpye=3;
    data->layer=layer;
    data->color=color;
    data->width=width;
    data->start_angle=start_angle;
    data->end_angle=end_angle;
    data->start_x=start_x;
    data->start_y=start_y;
    data->end_x=X_axle_length;
    data->end_y=Y_axle_length;
}



/*!---------------         基本不用      -----------------*/


void SendPredictData(graphic_data_struct_t data,uint32_t End_x,uint32_t End_y)
{
    graphic_data_struct_t imageData;

    imageData=data;
    imageData.end_x=End_x;
    imageData.end_y=End_y;
    uint16_t frametail=0xFFFF;
    //帧头处理
    Head head;
    head.SOF=UI_SOF;
    head.Data_Length=DrawOneGraph;
    head.Seq=Seq;
    head.CRC8=Get_CRC8_Check_Sum((uint8_t*)&head,4,0xFF);
    head.CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
    ext_student_interactive_header_data_t_tx datahead;
    datahead.data_cmd_id=Client_draws_one_graph;
    datahead.sender_ID=ARobot_ID;
    datahead.receiver_ID=ACilent_ID;
//帧尾处理
    frametail=Get_CRC16_Check_Sum((unsigned char *)&head,sizeof(head),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(datahead),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&data,sizeof(data),frametail);             //CRC16校验   //CRC16校验值计算（部分）

//    UI_SendByteS((unsigned char *)&head,(unsigned char *)&datahead,(unsigned char *)&imageData,(unsigned char *)&frametail,sizeof(imageData));
    Seq++;
}

/**@brief
 * @param
 */
void SendStringData(ext_client_custom_character_t *imageData)
{
    uint16_t frametail=0xFFFF;
    //帧头处理
    Head head;
    head.SOF=UI_SOF;
    head.Data_Length=DrawCharacter;
    head.Seq=Seq;
    head.CRC8=Get_CRC8_Check_Sum((uint8_t*)&head,4,0xFF);
    head.CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
    ext_student_interactive_header_data_t_tx datahead;
    datahead.data_cmd_id=Client_draws_character;
    datahead.sender_ID=ARobot_ID;
    datahead.receiver_ID=ACilent_ID;
//帧尾处理
    frametail=Get_CRC16_Check_Sum((unsigned char *)&head,sizeof(head),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(datahead),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)imageData,sizeof(ext_client_custom_character_t),frametail);             //CRC16校验   //CRC16校验值计算（部分）

//    UI_SendByteS((unsigned char *)&head,(unsigned char *)&datahead,(unsigned char *)&imageData,(unsigned char *)&frametail,sizeof(ext_client_custom_character_t));
    Seq++;
}
/**@brief
 * @param
 */
void SendData(graphic_data_struct_t *imageData)
{
    uint16_t frametail=0xFFFF;
    //帧头处理
    Head head;
    head.SOF=UI_SOF;
    head.Data_Length=DrawOneGraph;
    head.Seq=Seq;
    head.CRC8=Get_CRC8_Check_Sum((uint8_t*)&head,4,0xFF);
    head.CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
    ext_student_interactive_header_data_t_tx datahead;
    datahead.data_cmd_id=Client_draws_one_graph;
    datahead.sender_ID=ARobot_ID;
    datahead.receiver_ID=ACilent_ID;
//帧尾处理
    frametail=Get_CRC16_Check_Sum((unsigned char *)&head,sizeof(head),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(datahead),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&imageData,sizeof(graphic_data_struct_t),frametail);             //CRC16校验   //CRC16校验值计算（部分）

//    UI_SendByteS((unsigned char *)&head,(unsigned char *)&datahead,(unsigned char *)&imageData,(unsigned char *)&frametail,sizeof(graphic_data_struct_t));
    Seq++;
}



/**@brief
 * @param
 */

void SendIntData(int_data_struct_t *imageData)
{
    uint16_t frametail=0xFFFF;
    //帧头处理
    Head head;
    head.SOF=UI_SOF;
    head.Data_Length=DrawOneGraph;
    head.Seq=Seq;
    head.CRC8=Get_CRC8_Check_Sum((uint8_t*)&head,4,0xFF);
    head.CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
    ext_student_interactive_header_data_t_tx datahead;
    datahead.data_cmd_id=Client_draws_one_graph;
    datahead.sender_ID=ARobot_ID;
    datahead.receiver_ID=ACilent_ID;
//帧尾处理
    frametail=Get_CRC16_Check_Sum((unsigned char *)&head,sizeof(head),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(datahead),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)imageData,sizeof(int_data_struct_t),frametail);             //CRC16校验   //CRC16校验值计算（部分）

//    UI_SendByteS((unsigned char *)&head,(unsigned char *)&datahead,(unsigned char *)&imageData,(unsigned char *)&frametail,sizeof(int_data_struct_t));
    Seq++;
}


/**@brief
 * @param
 */


void SendDeleteData(ext_client_custom_graphic_delete_t data)
{
    ext_client_custom_graphic_delete_t imageData;
    uint16_t frametail=0xFFFF;
    //帧头处理
    Head head;
    head.SOF=UI_SOF;
    head.Data_Length=DeleteGraph;
    head.Seq=Seq;
    head.CRC8=Get_CRC8_Check_Sum((uint8_t*)&head,4,0xFF);
    head.CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
    ext_student_interactive_header_data_t_tx datahead;
    datahead.data_cmd_id=Client_delete_graphics_ID;
    datahead.sender_ID=ARobot_ID;
    datahead.receiver_ID=ACilent_ID;
//帧尾处理
    frametail=Get_CRC16_Check_Sum((unsigned char *)&head,sizeof(head),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(datahead),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&data,sizeof(data),frametail);             //CRC16校验   //CRC16校验值计算（部分）

//    UI_SendByteS((unsigned char *)&head,(unsigned char *)&datahead,(unsigned char *)&imageData,(unsigned char *)&frametail,sizeof(imageData));
    Seq++;
}


void SendFloatData(int_data_struct_t *imageData)
{
    uint16_t frametail=0xFFFF;
    //帧头处理
    Head head;
    head.SOF=UI_SOF;
    head.Data_Length=DrawOneGraph;
    head.Seq=Seq;
    head.CRC8=Get_CRC8_Check_Sum((uint8_t*)&head,4,0xFF);
    head.CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
    ext_student_interactive_header_data_t_tx datahead;
    datahead.data_cmd_id=Client_draws_one_graph;
    datahead.sender_ID=ARobot_ID;
    datahead.receiver_ID=ACilent_ID;
//帧尾处理
    frametail=Get_CRC16_Check_Sum((unsigned char *)&head,sizeof(head),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(datahead),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)imageData,sizeof(int_data_struct_t),frametail);             //CRC16校验   //CRC16校验值计算（部分）

//    UI_SendByteS((unsigned char *)&head,(unsigned char *)&datahead,(unsigned char *)&imageData,(unsigned char *)&frametail,sizeof(int_data_struct_t));
    Seq++;
}




/*!-------------普通版本：不可指定绘图频率----- 注意：要搭配下面注释的发送函数使用  -----------*/
//void UI_SendByteS(unsigned char *framehead,unsigned char *datahead,unsigned char *data,unsigned char *datatail,int data_length)
//{
//    int TxDataLength =  sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx) + data_length + sizeof(uint16_t);
//    memcpy(SendDataBuffer,framehead,sizeof(Head));
//    memcpy(SendDataBuffer + sizeof(Head),datahead, sizeof(ext_student_interactive_header_data_t_tx));
//    memcpy(SendDataBuffer+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx),data,data_length);
//    memcpy(SendDataBuffer+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx) + data_length,datatail, sizeof(uint16_t));
//    HAL_UART_Transmit_IT(&huart6,SendDataBuffer,TxDataLength);
//
//}
/**@brief  图形绘画函数
 * @param   graphics_type: 图形类型 可填：
 *                              LINE:直线
 *                              RECTANGLE:矩形
 *                              CIRCLE:圆
 * @param graphname: 图像名称，作为删除、修改等操作客户端的索引
 * @param operate_tpye：图形操作 其中0:空操作 1:增加 2:修改 3:删除
 * @param layer：图层数范围为0-9
 * @param color：颜色0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
 * @param width：线条宽度
 * @param start_angle: 在画圆弧时为起始角度   画其他图形时没有意义
 * @param end_angle: 在画圆弧时为终止角度 画其他图形时没有意义
 * @param start_x：当画直线、矩形时为 起点 x 坐标  画圆时为圆心x 坐标
 * @param start_y：当画直线、矩形时为 起点 y 坐标  画圆时为圆心y 坐标
 * @param end_x:  当画直线时为终点x坐标     画矩形时为对角顶点 x坐标     画椭圆和圆弧时为x半轴长度 当画圆时该参数无用，可填 NONE
 * @param end_y:  当画直线时为终点y坐标     画矩形时为对角顶点 y坐标     画椭圆和圆弧时为y半轴长度 当画圆时该参数无用，可填 NONE
 * @param radius: 当画圆时表示圆的半径   画其他图形时该参数无用，可填 NONE
 */

void uiDrawGraphics(uint8_t graphics_type,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_angle,uint32_t end_angle,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y,uint32_t radius)
{
    graphic_data_struct_t imageData;
    switch (graphics_type) {
        case LINE:
            DrawLine(&imageData,(char*)graphname, operate_tpye ,layer,color,width,start_x,start_y,end_x,end_y);
            break;
        case RECTANGLE:
            DrawRectangle(&imageData,graphname,operate_tpye,layer,color,width,start_x,start_y,end_x,end_y);
            break;
        case CIRCLE:
            DrawFullCircle(&imageData,graphname,operate_tpye,layer,color,width,start_x,start_y,radius);
            break;
        case ELLIPSE:
            DrawEllipse(&imageData,graphname,operate_tpye,layer,color,width,start_angle,end_angle,start_x,start_y,end_x,end_y);
            break;
        case ARC:
            DrawARC(&imageData,graphname,operate_tpye,layer,color,width,start_angle,end_angle,start_x,start_y,end_x,end_y);
    }

    uint16_t frametail=0xFFFF;
    //帧头处理
    Head head;
    head.SOF=UI_SOF;
    head.Data_Length=DrawOneGraph;
    head.Seq=Seq;
    head.CRC8=Get_CRC8_Check_Sum((uint8_t*)&head,4,0xFF);
    head.CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
    ext_student_interactive_header_data_t_tx datahead;
    datahead.data_cmd_id=Client_draws_one_graph;
    datahead.sender_ID=ARobot_ID;
    datahead.receiver_ID=ACilent_ID;
//帧尾处理
    frametail=Get_CRC16_Check_Sum((unsigned char *)&head,sizeof(head),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(datahead),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&imageData,sizeof(imageData),frametail);             //CRC16校验   //CRC16校验值计算（部分）

    //UI_SendByteS((unsigned char *)&head,(unsigned char *)&datahead,(unsigned char *)&imageData,(unsigned char *)&frametail,sizeof(imageData));
    Seq++;
}


/**
  * @brief          画一个整数
  * @param[in]      data: 要被操作的结构体的地址
  * @param[in]      graphname: 图像名称，作为删除、修改等操作客户端的索引
  * @param[in]      operate_tpye：图形操作 其中0:空操作 1:增加 2:修改 3:删除
	* @param[in]      layer：图层数范围为0-9
	* @param[in]      color：颜色0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
	* @param[in]      width：线条宽度
 * @param           font_size:字体大小
	* @param[in]      start_x：起点 x 坐标
	* @param[in]      start_y：起点 y 坐标
 * @param               intdata:要画的整型数据
  * @retval         none
  */
void uiDrawIntData(char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t intdata)
{
    int_data_struct_t imageData;
    DrawInteger(&imageData,(char*)graphname,operate_tpye,layer,color,font_size,width,start_x,start_y,intdata);
    uint16_t frametail=0xFFFF;
    //帧头处理
    Head head;
    head.SOF=UI_SOF;
    head.Data_Length=DrawOneGraph;
    head.Seq=Seq;
    head.CRC8=Get_CRC8_Check_Sum((uint8_t*)&head,4,0xFF);
    head.CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
    ext_student_interactive_header_data_t_tx datahead;
    datahead.data_cmd_id=Client_draws_one_graph;
    datahead.sender_ID=ARobot_ID;
    datahead.receiver_ID=ACilent_ID;
//帧尾处理
    frametail=Get_CRC16_Check_Sum((unsigned char *)&head,sizeof(head),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(datahead),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&imageData,sizeof(int_data_struct_t),frametail);             //CRC16校验   //CRC16校验值计算（部分）

//    UI_SendByteS((unsigned char *)&head,(unsigned char *)&datahead,(unsigned char *)&imageData,(unsigned char *)&frametail,sizeof(int_data_struct_t));
    Seq++;
}
/**
  * @brief          画字符串
  * @param[in]      graphname: 图像名称，作为删除、修改等操作客户端的索引
  * @param[in]      operate_tpye：图形操作 其中0:空操作 1:增加 2:修改 3:删除
	* @param[in]      layer：图层数范围为0-9
 * @param            font_size: 字体大小
	* @param[in]      color：颜色0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
	* @param[in]      width：线条宽度
	* @param[in]      start_x：起点 x 坐标
	* @param[in]      start_y：起点 y 坐标
 * @param             chardata:要画的字符串数据
  * @retval         none
  */
void uiDrawStringData(char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t width,uint32_t start_x,uint32_t start_y,uint8_t *chardata)
{
    static ext_client_custom_character_t imageData={0};
    memset(&(imageData.data),0, sizeof(imageData.data));
    DrawString(&imageData,(char*)graphname,operate_tpye,layer,color,font_size,width,start_x,start_y,chardata);
    uint16_t frametail=0xFFFF;
    //帧头处理
    Head head;
    head.SOF=UI_SOF;
    head.Data_Length=DrawCharacter;
    head.Seq=Seq;
    head.CRC8=Get_CRC8_Check_Sum((uint8_t*)&head,4,0xFF);
    head.CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
    ext_student_interactive_header_data_t_tx datahead;
    datahead.data_cmd_id=Client_draws_character;
    datahead.sender_ID=ARobot_ID;
    datahead.receiver_ID=ACilent_ID;
//帧尾处理
    frametail=Get_CRC16_Check_Sum((unsigned char *)&head,sizeof(head),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(datahead),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&imageData,sizeof(ext_client_custom_character_t),frametail);             //CRC16校验   //CRC16校验值计算（部分）

//    UI_SendByteS((unsigned char *)&head,(unsigned char *)&datahead,(unsigned char *)&imageData,(unsigned char *)&frametail,sizeof(ext_client_custom_character_t));
    Seq++;
}

/**
* @brief          删除某一图层的UI
* @param[in]      Delete_Operate: 图形操作 其中0：空操作 1：删除图层 2：删除所有
* @param[in]      Layer: 图层数范围为0-9
*
*/
void uiDeleteGraphic(uint8_t Delete_Operate,uint8_t Layer)
{
    ext_client_custom_graphic_delete_t imageData;
    DeleteUI(&imageData,Delete_Operate,Layer);
    uint16_t frametail=0xFFFF;
    //帧头处理
    Head head;
    head.SOF=UI_SOF;
    head.Data_Length=DeleteGraph;
    head.Seq=Seq;
    head.CRC8=Get_CRC8_Check_Sum((uint8_t*)&head,4,0xFF);
    head.CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
    ext_student_interactive_header_data_t_tx datahead;
    datahead.data_cmd_id=Client_delete_graphics_ID;
    datahead.sender_ID=ARobot_ID;
    datahead.receiver_ID=ACilent_ID;
//帧尾处理
    frametail=Get_CRC16_Check_Sum((unsigned char *)&head,sizeof(head),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(datahead),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&imageData,sizeof(imageData),frametail);             //CRC16校验   //CRC16校验值计算（部分）

//    UI_SendByteS((unsigned char *)&head,(unsigned char *)&datahead,(unsigned char *)&imageData,(unsigned char *)&frametail,sizeof(imageData));
    Seq++;
}

/**
 * @brief   画一个浮点数
 * @param data: 要被操作的结构体的地址
 * @param operate_tpye
 * @param layer
 * @param color
 * @param font_size
 * @param digits 小数点后有效位数
 * @param width
 * @param start_x
 * @param start_y
 * @param float_high 32位浮点数
 * @param float_middle 32位浮点数
 * @param float_low 32位浮点数
 */
void uiDrawFloatData(char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t digits,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t floatdata)
{
    int_data_struct_t imageData;
    DrawFloat(&imageData,(char*)graphname,operate_tpye,layer,color,font_size,digits,width,start_x,start_y,floatdata);
    uint16_t frametail=0xFFFF;
    //帧头处理
    Head head;
    head.SOF=UI_SOF;
    head.Data_Length=DrawOneGraph;
    head.Seq=Seq;
    head.CRC8=Get_CRC8_Check_Sum((uint8_t*)&head,4,0xFF);
    head.CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
    ext_student_interactive_header_data_t_tx datahead;
    datahead.data_cmd_id=Client_draws_one_graph;
    datahead.sender_ID=ARobot_ID;
    datahead.receiver_ID=ACilent_ID;
//帧尾处理
    frametail=Get_CRC16_Check_Sum((unsigned char *)&head,sizeof(head),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(datahead),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&imageData,sizeof(int_data_struct_t),frametail);             //CRC16校验   //CRC16校验值计算（部分）

//    UI_SendByteS((unsigned char *)&head,(unsigned char *)&datahead,(unsigned char *)&imageData,(unsigned char *)&frametail,sizeof(int_data_struct_t));
    Seq++;
}



/*!-------------------   升级版本：     可指定频率发送画图数据  ------------*/


/**
  * @brief      按指定频率画一个整数
  * @param      data: 要被操作的结构体的地址
  * @param      graphname: 图像名称，作为删除、修改等操作客户端的索引
  * @param      operate_tpye：图形操作 其中0:空操作 1:增加 2:修改 3:删除
  * @param      layer：图层数范围为0-9
  * @param      color：颜色0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
  * @param      width：线条宽度
  * @param      font_size:字体大小
  * @param      start_x：起点 x 坐标
  * @param      start_y：起点 y 坐标
  * @param      intdata:要画的整型数据
  * @param      frequency:指定的发送频率
  * @retval     UI_TCB* :返回的是该图像的控制块指针
  */
UI_TCB* uiDrawIntDataSeq(char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t intdata,int frequency)
{
    //新建imageData空间
    int_data_struct_t* imageData;
    imageData = (int_data_struct_t*)rt_malloc(sizeof(int_data_struct_t));
    //新建TCB空间
    UI_TCB* TCB;
    TCB = (UI_TCB*)rt_malloc(sizeof(UI_TCB));


    DrawInteger(imageData,(char*)graphname,operate_tpye,layer,color,font_size,width,start_x,start_y,intdata);
    uint16_t frametail=0xFFFF;
    //帧头处理
    //新建帧头
    //static Head head;
    Head* head;
    head = (Head*) rt_malloc(sizeof(Head));
    head->SOF=UI_SOF;
    head->Data_Length=DrawOneGraph;
    head->Seq=Seq;
    head->CRC8=Get_CRC8_Check_Sum((uint8_t*)head,4,0xFF);
    head->CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
    static ext_student_interactive_header_data_t_tx datahead;
    datahead.data_cmd_id=Client_draws_one_graph;
    datahead.sender_ID=ARobot_ID;
    datahead.receiver_ID=ACilent_ID;
//帧尾处理
    frametail=Get_CRC16_Check_Sum((unsigned char *)head,sizeof(Head),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(ext_student_interactive_header_data_t_tx),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)imageData,sizeof(int_data_struct_t),frametail);             //CRC16校验   //CRC16校验值计算（部分）
//数据拼装

    int data_length = sizeof(int_data_struct_t);
    int TxDataLength =  sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx) + data_length + sizeof(uint16_t);
    //新建数据空间
    //static uint8_t ImageSendDataBuffer[150];
    uint8_t* ImageSendDataBuffer = (uint8_t *) rt_malloc(TxDataLength);
    memcpy(ImageSendDataBuffer,(unsigned char *)head,sizeof(Head));
    memcpy(ImageSendDataBuffer + sizeof(Head),(unsigned char *)&datahead, sizeof(ext_student_interactive_header_data_t_tx));
    memcpy(ImageSendDataBuffer+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx),(unsigned char *)imageData,data_length);
    memcpy(ImageSendDataBuffer+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx) + data_length,(unsigned char *)&frametail, sizeof(uint16_t));
//UI控制块组装
    TCB->frequency = frequency/10;
    TCB->frequency_cnt =  TCB->frequency;
    TCB->senddata = ImageSendDataBuffer;
    TCB->size  = TxDataLength;
    TCB->seqnum = UI_TCBNum;
    UI_SendTCBSequence[UI_TCBNum++] = TCB;
    TCB->head = head;
    TCB->datahead = &datahead;
    TCB->UI_Int_Float_Data = imageData;
    TCB->fresh_count = 5;
    TCB->FOA = 1;
    TCB->datelength = data_length;
//    UI_SendByteS((unsigned char *)&head,(unsigned char *)&datahead,(unsigned char *)&imageData,(unsigned char *)&frametail,sizeof(int_data_struct_t));
    Seq++;
    return TCB;
}




/**
  * @brief      按指定频率画字符串
  * @param      graphname: 图像名称，作为删除、修改等操作客户端的索引
  * @param      operate_tpye：图形操作 其中0:空操作 1:增加 2:修改 3:删除
  * @param      layer：图层数范围为0-9
  * @param      font_size: 字体大小
  * @param      color：颜色0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
  * @param      width：线条宽度
  * @param      start_x：起点 x 坐标
  * @param      start_y：起点 y 坐标
  * @param      chardata:要画的字符串数据
  * @param      frequency:指定的发送频率
  * @retval     UI_TCB* :返回的是该图像的控制块指针
  */

UI_TCB* uiDrawStringDataSeq(char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t width,uint32_t start_x,uint32_t start_y,uint8_t *chardata,int frequency)
{
    //新建imageData空间
    ext_client_custom_character_t* imageData;
    imageData = (ext_client_custom_character_t*)rt_malloc(sizeof(ext_client_custom_character_t));
    //新建TCB空间
    UI_TCB* TCB;
    TCB = (UI_TCB*)rt_malloc(sizeof(UI_TCB));

    memset(&(imageData->data),0, sizeof(imageData->data));
    DrawString(imageData,(char*)graphname,operate_tpye,layer,color,font_size,width,start_x,start_y,chardata);
    uint16_t frametail=0xFFFF;
    //帧头处理
    //新建帧头
    //static Head head;
    Head* head;
    head = (Head*) rt_malloc(sizeof(Head));
    head->SOF=UI_SOF;
    head->Data_Length=DrawCharacter;
    head->Seq=Seq;
    head->CRC8=Get_CRC8_Check_Sum((uint8_t*)head,4,0xFF);
    head->CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
    static ext_student_interactive_header_data_t_tx datahead;
    datahead.data_cmd_id=Client_draws_character;
    datahead.sender_ID=ARobot_ID;
    datahead.receiver_ID=ACilent_ID;
//帧尾处理
    frametail=Get_CRC16_Check_Sum((unsigned char *)head,sizeof(Head),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(datahead),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)imageData,sizeof(ext_client_custom_character_t),frametail);             //CRC16校验   //CRC16校验值计算（部分）
//数据拼装

    int data_length = sizeof(ext_client_custom_character_t);
    int TxDataLength =  sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx) + data_length + sizeof(uint16_t);
    //新建数据空间
    //static uint8_t ImageSendDataBuffer[150];
    uint8_t* ImageSendDataBuffer = (uint8_t *) rt_malloc(TxDataLength);
    memcpy(ImageSendDataBuffer,(unsigned char *)head,sizeof(Head));
    memcpy(ImageSendDataBuffer + sizeof(Head),(unsigned char *)&datahead, sizeof(ext_student_interactive_header_data_t_tx));
    memcpy(ImageSendDataBuffer+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx),(unsigned char *)imageData,data_length);
    memcpy(ImageSendDataBuffer+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx) + data_length,(unsigned char *)&frametail, sizeof(uint16_t));

//UI控制块组装

    TCB->frequency = frequency/10;
    TCB->frequency_cnt =  TCB->frequency;
    TCB->senddata = ImageSendDataBuffer;
    TCB->size  = TxDataLength;
    TCB->seqnum = UI_TCBNum;
    TCB->head = head;
    TCB->datahead = &datahead;
    TCB->UI_String_Data = imageData;
    TCB->fresh_count = 5;
    TCB->FOA = 1;
    UI_SendTCBSequence[UI_TCBNum++] = TCB;
    TCB->datelength = data_length;
    TCB->type = 2;
    Seq++;
    return TCB;


}


/**@brief  按指定频率画一个图形
 * @param   graphics_type: 图形类型 可填：
 *                              LINE:直线
 *                              RECTANGLE:矩形
 *                              CIRCLE:圆
 * @param graphname: 图像名称，作为删除、修改等操作客户端的索引
 * @param operate_tpye：图形操作 其中0:空操作 1:增加 2:修改 3:删除
 * @param layer：图层数范围为0-9
 * @param color：颜色0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
 * @param width：线条宽度
 * @param start_angle: 在画圆弧时为起始角度   画其他图形时没有意义
 * @param end_angle: 在画圆弧时为终止角度 画其他图形时没有意义
 * @param start_x：当画直线、矩形时为 起点 x 坐标  画圆时为圆心x 坐标
 * @param start_y：当画直线、矩形时为 起点 y 坐标  画圆时为圆心y 坐标
 * @param end_x:  当画直线时为终点x坐标     画矩形时为对角顶点 x坐标     当画圆时该参数无用，可填 NONE
 * @param end_y:  当画直线时为终点y坐标     画矩形时为对角顶点 y坐标     当画圆时该参数无用，可填 NONE
 * @param radius: 当画圆时表示圆的半径   画其他图形时该参数无用，可填 NONE
 * @param frequency:指定的发送频率
 * @return UI_TCB* :返回的是该图像的控制块指针
 */

UI_TCB* uiDrawGraphicsSeq(uint8_t graphics_type,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_angle,uint32_t end_angle,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y,uint32_t radius,int frequency)
{
    //新建imageData空间
    graphic_data_struct_t* imageData;
    imageData = (graphic_data_struct_t*)rt_malloc(sizeof(graphic_data_struct_t));
    //新建TCB空间
    UI_TCB* TCB;
    TCB = (UI_TCB*)rt_malloc(sizeof(UI_TCB));
    switch (graphics_type) {
        case LINE:
            DrawLine(imageData,(char*)graphname, operate_tpye ,layer,color,width,start_x,start_y,end_x,end_y);
            break;
        case RECTANGLE:
            DrawRectangle(imageData,(char*)graphname,operate_tpye,layer,color,width,start_x,start_y,end_x,end_y);
            break;
        case CIRCLE:
            DrawFullCircle(imageData,(char*)graphname,operate_tpye,layer,color,width,start_x,start_y,radius);
            break;
        case ARC:
            DrawARC(imageData,(char*)graphname,operate_tpye,layer,color,width,start_angle,end_angle,start_x,start_y,end_x,end_y);
            break;
        case ELLIPSE:
            DrawEllipse(imageData,(char*)graphname,operate_tpye,layer,color,width,start_angle,end_angle,start_x,start_y,end_x,end_y);
    }

    uint16_t frametail=0xFFFF;
    //帧头处理
    //新建帧头
    //static Head head;
    Head* head;
    head = (Head*) rt_malloc(sizeof(Head));
    head->SOF=UI_SOF;
    head->Data_Length=DrawOneGraph;
    head->Seq=Seq;
    head->CRC8=Get_CRC8_Check_Sum((uint8_t*)head,4,0xFF);
    head->CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
    static ext_student_interactive_header_data_t_tx datahead;
    datahead.data_cmd_id=Client_draws_one_graph;
    datahead.sender_ID=ARobot_ID;
    datahead.receiver_ID=ACilent_ID;
//帧尾处理
    frametail=Get_CRC16_Check_Sum((unsigned char *)head,sizeof(Head),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(datahead),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)imageData,sizeof(graphic_data_struct_t),frametail);             //CRC16校验   //CRC16校验值计算（部分）
//数据拼装

    int data_length = sizeof(graphic_data_struct_t);
    int TxDataLength =  sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx) + data_length + sizeof(uint16_t);
    //新建数据空间
    //static uint8_t ImageSendDataBuffer[150];
    uint8_t* ImageSendDataBuffer = (uint8_t *) rt_malloc(TxDataLength);
    memcpy(ImageSendDataBuffer,(unsigned char *)head,sizeof(Head));
    memcpy(ImageSendDataBuffer + sizeof(Head),(unsigned char *)&datahead, sizeof(ext_student_interactive_header_data_t_tx));
    memcpy(ImageSendDataBuffer+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx),(unsigned char *)imageData,data_length);
    memcpy(ImageSendDataBuffer+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx) + data_length,(unsigned char *)&frametail, sizeof(uint16_t));

//UI控制块组装
    TCB->UI_Image_Data = imageData;
    TCB->frequency = frequency/10;
    TCB->frequency_cnt =  TCB->frequency;
    TCB->senddata = ImageSendDataBuffer;
    TCB->size  = TxDataLength;
    TCB->seqnum = UI_TCBNum;
    TCB->head = head;
    TCB->datahead = &datahead;
    TCB->fresh_count = 5;
    TCB->FOA = 1;
    TCB->datelength = data_length;
    TCB->type = 0;
    UI_SendTCBSequence[UI_TCBNum++] = TCB;


    Seq++;
    return TCB;
}


/**
 * @brief   按指定频率画一个浮点数
 * @param   data: 要被操作的结构体的地址
 * @param   operate_tpye
 * @param   layer
 * @param   color
 * @param   font_size
 * @param   digits 小数点后有效位数
 * @param   width
 * @param   start_x
 * @param   start_y
 * @param   floatdata:浮点数乘1000以后得到的数据
 * @return  UI_TCB* :返回的是该图像的控制块指针
 */
UI_TCB* uiDrawFloatDataSeq(char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t digits,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t floatdata,int frequency)
{
    //新建imageData空间
    int_data_struct_t* imageData;
    imageData = (int_data_struct_t*)rt_malloc(sizeof(int_data_struct_t));
    //新建TCB空间
    UI_TCB* TCB;
    TCB = (UI_TCB*)rt_malloc(sizeof(UI_TCB));

    DrawFloat(imageData,(char*)graphname,operate_tpye,layer,color,font_size,digits,width,start_x,start_y,floatdata);
    uint16_t frametail=0xFFFF;
    //帧头处理
    //新建帧头
    //static Head head;
    Head* head;
    head = (Head*) rt_malloc(sizeof(Head));
    head->SOF=UI_SOF;
    head->Data_Length=DrawOneGraph;
    head->Seq=Seq;
    head->CRC8=Get_CRC8_Check_Sum((uint8_t*)head,4,0xFF);
    head->CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
    static ext_student_interactive_header_data_t_tx datahead;
    datahead.data_cmd_id=Client_draws_one_graph;
    datahead.sender_ID=ARobot_ID;
    datahead.receiver_ID=ACilent_ID;
//帧尾处理
    frametail=Get_CRC16_Check_Sum((unsigned char *)head,sizeof(Head),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(datahead),frametail);
    frametail=Get_CRC16_Check_Sum((unsigned char *)imageData,sizeof(int_data_struct_t),frametail);             //CRC16校验   //CRC16校验值计算（部分）

//数据拼装
    int data_length = sizeof(int_data_struct_t);
    int TxDataLength =  sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx) + data_length + sizeof(uint16_t);
    //新建数据空间
    //static uint8_t ImageSendDataBuffer[150];
    uint8_t* ImageSendDataBuffer = (uint8_t *) rt_malloc(TxDataLength);
    memcpy(ImageSendDataBuffer,(unsigned char *)head,sizeof(Head));
    memcpy(ImageSendDataBuffer + sizeof(Head),(unsigned char *)&datahead, sizeof(ext_student_interactive_header_data_t_tx));
    memcpy(ImageSendDataBuffer+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx),(unsigned char *)imageData,data_length);
    memcpy(ImageSendDataBuffer+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx) + data_length,(unsigned char *)&frametail, sizeof(uint16_t));

//UI控制块组装
    TCB->frequency = frequency/10;
    TCB->frequency_cnt =  TCB->frequency;
    TCB->senddata = ImageSendDataBuffer;
    TCB->size  = TxDataLength;
    TCB->seqnum = UI_TCBNum;
    UI_SendTCBSequence[UI_TCBNum++] = TCB;
    TCB->head = head;
    TCB->datahead = &datahead;
    TCB->UI_Int_Float_Data = imageData;
    TCB->fresh_count = 5;
    TCB->FOA = 1;
    TCB->datelength = data_length;
    TCB->type = 1;
//    UI_SendByteS((unsigned char *)&head,(unsigned char *)&datahead,(unsigned char *)&imageData,(unsigned char *)&frametail,sizeof(int_data_struct_t));
    Seq++;
    return TCB;
}


/*!发送模块，一切UI都经该模块发送给裁判系统*/
void UI_Draw(uint8_t* image,uint16_t size)
{
    HAL_UART_Transmit_DMA(&huart6,image,size);
    while (!__HAL_UART_GET_FLAG(&huart6,UART_FLAG_TC))
    {
        ;
    }
}

//void UI_Drawnew(UI_TCB * imageTCB)
//{
//    uint16_t frametail=0xFFFF;
//    static uint8_t ImageSendDataBuffer[150];
//    imageTCB->head->Seq = Seq++;
//    //CRC校验
//    frametail=Get_CRC16_Check_Sum((unsigned char *)imageTCB->head,sizeof(Head),frametail);
//    frametail=Get_CRC16_Check_Sum((unsigned char *)imageTCB->datahead,sizeof(ext_student_interactive_header_data_t_tx),frametail);
//    switch (imageTCB->type) {
//        case 0:
//            frametail=Get_CRC16_Check_Sum((unsigned char *)imageTCB->UI_Image_Data,sizeof(graphic_data_struct_t),frametail);
//            break;
//
//        case 1:
//            frametail=Get_CRC16_Check_Sum((unsigned char *)imageTCB->UI_Int_Float_Data,sizeof(int_data_struct_t),frametail);
//            break;
//
//        case 2:
//            frametail=Get_CRC16_Check_Sum((unsigned char *)imageTCB->UI_String_Data,sizeof(ext_client_custom_character_t),frametail);
//            break;
//    }
//
//    memcpy(ImageSendDataBuffer,(unsigned char *)imageTCB->head,sizeof(Head));
//    memcpy(ImageSendDataBuffer + sizeof(Head),(unsigned char *)imageTCB->datahead, sizeof(ext_student_interactive_header_data_t_tx));
//    switch (imageTCB->type) {
//        case 0:
//            memcpy(ImageSendDataBuffer+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx),(unsigned char *)imageTCB->UI_Image_Data,imageTCB->datelength);
//            break;
//
//        case 1:
//            memcpy(ImageSendDataBuffer+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx),(unsigned char *)imageTCB->UI_Int_Float_Data,imageTCB->datelength);
//            break;
//
//        case 2:
//            memcpy(ImageSendDataBuffer+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx),(unsigned char *)imageTCB->UI_String_Data,imageTCB->datelength);
//            break;
//    }
//    memcpy(ImageSendDataBuffer+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx) + imageTCB->datelength,(unsigned char *)&frametail, sizeof(uint16_t));
//    HAL_UART_Transmit_DMA(&huart6,ImageSendDataBuffer,imageTCB->size);
//
//}



/*!************************UI更新     搭配指定频率发送画图数据函数 使用******************************************/
/*!
 * @brief 更新整数型数据
 * @param TargetTCB:想要更新的UI绘图任务指针
 * @param NowTCB: 当前绘图任务队列中执行到的绘图任务指针
 * @param else ： 相关绘图的参数，可参照上方的函数进行辨识
 * */
void uiRefreshIntDataSeq(UI_TCB* TargetTCB , UI_TCB* NowTCB,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t intdata,int frequency)
{
    if (NowTCB->seqnum == TargetTCB->seqnum)
    {
        uint16_t frametail=0xFFFF;

        NowTCB->UI_Int_Float_Data->layer = layer;
        NowTCB->UI_Int_Float_Data->color = color;
        NowTCB->UI_Int_Float_Data->start_angle = font_size;
        NowTCB->UI_Int_Float_Data->width = width;
        NowTCB->UI_Int_Float_Data->start_x = start_x;
        NowTCB->UI_Int_Float_Data->start_y = start_y;
        NowTCB->UI_Int_Float_Data->data = intdata;
        NowTCB->frequency = frequency;
        if(NowTCB->FOA==1)
        {
            NowTCB->UI_Int_Float_Data->operate_tpye = 1;
        } else
        {
            NowTCB->UI_Int_Float_Data->operate_tpye = 2;
        }

        //帧头处理
        Head head;
        head.SOF=UI_SOF;
        head.Data_Length=DrawOneGraph;
        head.Seq=Seq;
        head.CRC8=Get_CRC8_Check_Sum((uint8_t*)&head,4,0xFF);
        head.CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
        ext_student_interactive_header_data_t_tx datahead;
        datahead.data_cmd_id=Client_draws_one_graph;
        datahead.sender_ID=ARobot_ID;
        datahead.receiver_ID=ACilent_ID;
//帧尾处理
        frametail=Get_CRC16_Check_Sum((unsigned char *)&head,sizeof(head),frametail);
        frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(datahead),frametail);
        frametail=Get_CRC16_Check_Sum((unsigned char *)NowTCB->UI_Int_Float_Data,sizeof(int_data_struct_t),frametail);
        //数据拼装
        int data_length = sizeof(int_data_struct_t);
        memcpy(NowTCB->senddata,(unsigned char *)&head,sizeof(Head));
        memcpy(NowTCB->senddata + sizeof(Head),(unsigned char *)&datahead, sizeof(ext_student_interactive_header_data_t_tx));
        memcpy(NowTCB->senddata+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx),(unsigned char *)(NowTCB->UI_Int_Float_Data),data_length);
        memcpy(NowTCB->senddata+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx) + data_length,(unsigned char *)&frametail, sizeof(uint16_t));
    }
}



/*!
 * @brief 更新整数型数据
 * @param TargetTCB:想要更新的UI绘图任务指针
 * @param NowTCB: 当前绘图任务队列中执行到的绘图任务指针
 * @param else ： 相关绘图的参数，可参照上方的函数进行辨识
 * */
void uiRefreshFloatDataSeq(UI_TCB* TargetTCB , UI_TCB* NowTCB,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t digits,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t floatdata,int frequency)
{
    if (NowTCB->seqnum == TargetTCB->seqnum)
    {
        uint16_t frametail=0xFFFF;

        NowTCB->UI_Int_Float_Data->layer = layer;
        NowTCB->UI_Int_Float_Data->color = color;
        NowTCB->UI_Int_Float_Data->start_angle = font_size;
        NowTCB->UI_Int_Float_Data->end_angle = digits;
        NowTCB->UI_Int_Float_Data->width = width;
        NowTCB->UI_Int_Float_Data->start_x = start_x;
        NowTCB->UI_Int_Float_Data->start_y = start_y;
        NowTCB->UI_Int_Float_Data->data = floatdata;
        NowTCB->frequency = frequency;
        if(NowTCB->FOA==1)
        {
            NowTCB->UI_Int_Float_Data->operate_tpye = 1;
        } else
        {
            NowTCB->UI_Int_Float_Data->operate_tpye = 2;
        }

        //帧头处理
        Head head;
        head.SOF=UI_SOF;
        head.Data_Length=DrawOneGraph;
        head.Seq=Seq;
        head.CRC8=Get_CRC8_Check_Sum((uint8_t*)&head,4,0xFF);
        head.CMD_ID=UI_CMD_Robo_Exchange;
        //数据处理
        ext_student_interactive_header_data_t_tx datahead;
        datahead.data_cmd_id=Client_draws_one_graph;
        datahead.sender_ID=ARobot_ID;
        datahead.receiver_ID=ACilent_ID;
        //帧尾处理
        frametail=Get_CRC16_Check_Sum((unsigned char *)&head,sizeof(head),frametail);
        frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(datahead),frametail);
        frametail=Get_CRC16_Check_Sum((unsigned char *)NowTCB->UI_Int_Float_Data,sizeof(int_data_struct_t),frametail);
        //数据拼装
        int data_length = sizeof(int_data_struct_t);
        memcpy(NowTCB->senddata,(unsigned char *)&head,sizeof(Head));
        memcpy(NowTCB->senddata + sizeof(Head),(unsigned char *)&datahead, sizeof(ext_student_interactive_header_data_t_tx));
        memcpy(NowTCB->senddata+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx),(unsigned char *)(NowTCB->UI_Int_Float_Data),data_length);
        memcpy(NowTCB->senddata+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx) + data_length,(unsigned char *)&frametail, sizeof(uint16_t));
    }

}



/*!
 * @brief 更新整数型数据
 * @param TargetTCB:想要更新的UI绘图任务指针
 * @param NowTCB: 当前绘图任务队列中执行到的绘图任务指针
 * @param else ： 相关绘图的参数，可参照上方的函数进行辨识
 * */
void uiRefreshGraphicsSeq(UI_TCB* TargetTCB,UI_TCB* NowTCB,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_angle,uint32_t end_angle,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y,uint32_t radius,int frequency)
{
    if (NowTCB->seqnum == TargetTCB->seqnum)
    {
        uint16_t frametail=0xFFFF;
        NowTCB->UI_Image_Data->layer = layer;
        NowTCB->UI_Image_Data->color = color;
        NowTCB->UI_Image_Data->start_angle = start_angle;
        NowTCB->UI_Image_Data->end_angle = end_angle;
        NowTCB->UI_Image_Data->width = width;
        NowTCB->UI_Image_Data->start_x = start_x;
        NowTCB->UI_Image_Data->start_y = start_y;
        NowTCB->UI_Image_Data->end_x = end_x;
        NowTCB->UI_Image_Data->end_y = end_y;
        NowTCB->UI_Image_Data->radius = radius;
        NowTCB->frequency = frequency;
        if(NowTCB->FOA==1)
        {
            NowTCB->UI_Image_Data->operate_tpye = 1;
        } else
        {
            NowTCB->UI_Image_Data->operate_tpye = 2;
        }

        //帧头处理
        Head head;
        head.SOF=UI_SOF;
        head.Data_Length=DrawOneGraph;
        head.Seq=Seq;
        head.CRC8=Get_CRC8_Check_Sum((uint8_t*)&head,4,0xFF);
        head.CMD_ID=UI_CMD_Robo_Exchange;
        //数据处理
        ext_student_interactive_header_data_t_tx datahead;
        datahead.data_cmd_id=Client_draws_one_graph;
        datahead.sender_ID=ARobot_ID;
        datahead.receiver_ID=ACilent_ID;
        //帧尾处理
        frametail=Get_CRC16_Check_Sum((unsigned char *)&head,sizeof(head),frametail);
        frametail=Get_CRC16_Check_Sum((unsigned char *)&datahead,sizeof(datahead),frametail);
        frametail=Get_CRC16_Check_Sum((unsigned char *)NowTCB->UI_Image_Data,sizeof(graphic_data_struct_t),frametail);
        //数据拼装
        int data_length = sizeof(graphic_data_struct_t);
        memcpy(NowTCB->senddata,(unsigned char *)&head,sizeof(Head));
        memcpy(NowTCB->senddata + sizeof(Head),(unsigned char *)&datahead, sizeof(ext_student_interactive_header_data_t_tx));
        memcpy(NowTCB->senddata+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx),(unsigned char *)(NowTCB->UI_Image_Data),data_length);
        memcpy(NowTCB->senddata+ sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx) + data_length,(unsigned char *)&frametail, sizeof(uint16_t));
    }

};




/*!
 * @brief 更新整数型数据
 * @param TargetTCB:想要更新的UI绘图任务指针
 * @param NowTCB: 当前绘图任务队列中执行到的绘图任务指针
 * @param else ： 相关绘图的参数，可参照上方的函数进行辨识
 * */
void uiRefreshStringDataSeq(UI_TCB* TargetTCB,UI_TCB* NowTCB,uint32_t layer,uint32_t color,uint32_t font_size,uint32_t width,uint32_t start_x,uint32_t start_y,uint8_t *chardata,int frequency)
{
//    static UI_TCB* StringTCB = NowTCB;
    uint16_t frametail = 0xFFFF;
    if (NowTCB->seqnum == TargetTCB->seqnum) {
        //uint16_t frametail = 0xFFFF;
        NowTCB->UI_String_Data->grapic_data_struct.layer = layer;
        NowTCB->UI_String_Data->grapic_data_struct.color = color;
        NowTCB->UI_String_Data->grapic_data_struct.start_angle = font_size;
        NowTCB->UI_String_Data->grapic_data_struct.end_angle = sizeof(chardata) - 1;
        NowTCB->UI_String_Data->grapic_data_struct.width = width;
        NowTCB->UI_String_Data->grapic_data_struct.start_x = start_x;
        NowTCB->UI_String_Data->grapic_data_struct.start_y = start_y;
        memset(NowTCB->UI_String_Data->data,0, 30);
        memcpy(NowTCB->UI_String_Data->data, chardata, NowTCB->UI_String_Data->grapic_data_struct.end_angle);
        NowTCB->frequency = frequency;
        if (NowTCB->FOA == 1) {
            NowTCB->UI_String_Data->grapic_data_struct.operate_tpye = 1;
        } else {
            NowTCB->UI_String_Data->grapic_data_struct.operate_tpye = 2;
        }

        //帧头处理
        Head head;
        head.SOF = UI_SOF;
        head.Data_Length = DrawCharacter;
        head.Seq = Seq;
        head.CRC8 = Get_CRC8_Check_Sum((uint8_t *) &head, 4, 0xFF);
        head.CMD_ID = UI_CMD_Robo_Exchange;
//数据处理
        ext_student_interactive_header_data_t_tx datahead;
        datahead.data_cmd_id = Client_draws_character;
        datahead.sender_ID = ARobot_ID;
        datahead.receiver_ID = ACilent_ID;
//帧尾处理
        frametail = Get_CRC16_Check_Sum((unsigned char *) &head, sizeof(head), frametail);
        frametail = Get_CRC16_Check_Sum((unsigned char *) &datahead, sizeof(datahead), frametail);
        frametail = Get_CRC16_Check_Sum((unsigned char *) NowTCB->UI_String_Data,sizeof(ext_client_custom_character_t), frametail);
        //数据拼装
        int data_length = sizeof(ext_client_custom_character_t);
        memcpy(NowTCB->senddata, (unsigned char *) &head, sizeof(Head));
        memcpy(NowTCB->senddata + sizeof(Head), (unsigned char *) &datahead,sizeof(ext_student_interactive_header_data_t_tx));
        memcpy(NowTCB->senddata + sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx),(unsigned char *) (NowTCB->UI_String_Data), data_length);
        memcpy(NowTCB->senddata + sizeof(Head) + sizeof(ext_student_interactive_header_data_t_tx) + data_length,(unsigned char *) &frametail, sizeof(uint16_t));
    }
}


