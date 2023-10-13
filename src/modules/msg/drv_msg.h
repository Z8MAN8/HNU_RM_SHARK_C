 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-09-06      ChuShicheng     first version
 */
#ifndef _DRV_MSG_H
#define _DRV_MSG_H

#include <rtthread.h>

#define MAX_TOPIC_COUNT 20    // 最多支持的话题数量
#define MSG_NAME_MAX    12    // 话题名称最大长度

/**
 * @brief 话题类型
 *
 */
typedef struct topic
{
    char name[MSG_NAME_MAX];
    void *msg;              // 指向msg实例的指针
    rt_sem_t sem;           // 保证发布消息的原子性
} topic_t;

/**
 * @brief 订阅者类型.每个发布者拥有发布者实例,并且可以通过链表访问所有订阅了自己发布的话题的订阅者
 *
 */
typedef struct sublisher
{
    const char *topic_name;
    topic_t *tp;               // 话题的指针
    uint8_t len;               // 消息类型长度
} subscriber_t;

/**
 * @brief 发布者类型.每个发布者拥有发布者实例,并且可以通过链表访问所有订阅了自己发布的话题的订阅者
 *
 */
typedef struct publisher
{
    const char *topic_name;
    topic_t *tp;               // 话题的指针
    uint8_t len;               // 消息类型长度
} publisher_t;

/**
 * @brief 订阅name的话题消息
 *
 * @param name 话题名称
 * @param len 消息长度,通过sizeof()获取
 * @return subscriber_t* 返回订阅者实例
 */
subscriber_t *sub_register(char *name, uint8_t len);

/**
 * @brief 注册成为消息发布者
 *
 * @param name 发布者发布的话题名称(话题)
 * @param len  消息类型长度,通过sizeof()获取
 * @return publisher_t* 返回发布者实例
 */
publisher_t *pub_register(char *name, uint8_t len);

/**
 * @brief 发布消息
 *
 * @param pub 发布者实例指针
 * @param data 数据指针,将要发布的消息放到此处
 * @return uint8_t 返回值为0说明发布失败,为1说明发布成功
 */
uint8_t sub_get_msg(subscriber_t *sub, void *data);

/**
 * @brief 获取消息
 *
 * @param sub 订阅者实例指针
 * @param data 数据指针,接收的消息将会放到此处
 * @return uint8_t 返回值为1说明获取到了新的消息
 */
uint8_t pub_push_msg(publisher_t *pub, void *data);

#endif /* _DRV_MSG_H */
