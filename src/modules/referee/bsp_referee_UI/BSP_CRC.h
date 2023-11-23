//
// Created by 16933 on 2022/3/14.
//

#ifndef SETINGS_BSP_CRC_H
#define SETINGS_BSP_CRC_H
#include "stdint.h"
#include "stdio.h"
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);





#endif //SETINGS_BSP_CRC_H
