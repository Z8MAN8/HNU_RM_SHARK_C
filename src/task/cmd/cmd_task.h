/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
*/
#ifndef _CMD_TASK_H
#define _CMD_TASK_H

#include <rtthread.h>
typedef struct
{
    float num[3];
    float error[2];
}gim_auto_judge;
void cmd_thread_entry(void *argument);

#endif /* _CMD_TASK_H */
