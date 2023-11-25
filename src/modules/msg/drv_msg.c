 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-09-06      ChuShicheng     first version
 */
#include "drv_msg.h"

#define DBG_TAG           "drv.msg"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static uint8_t idx = 0; // 用于记录当前已经注册的话题数量
static topic_t *topic_obj[MAX_TOPIC_COUNT] = {NULL};  // 保存每个注册的话题实例指针，ringbuffer 为话题的载体

/**
 * @brief 检查话题实例是否重名
 *
 * @param name 话题名称
 * @return int 0:不存在重名的话题实例;非0:存在重名的话题实例+1(避免第一个元素重名时返回0)
 */
static uint8_t check_topic_name(char *name){
    for(int i = 0; i < MAX_TOPIC_COUNT; i++){
        if(topic_obj[i] != NULL){
            if(rt_strcmp(topic_obj[i]->name, name) == 0){   // 如果有重名的话题实例
                return i+1;
            }
        }
    }
    return 0;
}

/**
 * @brief 注册成为消息发布者
 *
 * @param name 发布者发布的话题名称(话题)
 * @param len  消息类型长度,通过sizeof()获取
 * @return publisher_t* 返回发布者实例
 */
publisher_t *pub_register(char *name, uint8_t len){
    uint8_t check_num = check_topic_name(name);
    publisher_t *pub = (publisher_t *)rt_malloc(sizeof(publisher_t));

    if(!check_num){  // 如果不存在重名的话题实例
        topic_obj[idx] = (topic_t *)rt_malloc(sizeof(topic_t));
        rt_memset(topic_obj[idx], 0, sizeof(topic_t));
        if(topic_obj[idx] == NULL){
            LOG_E("malloc failed!");
            return NULL;
        }
        topic_obj[idx]->msg = (void *)rt_malloc(len);
        if(topic_obj[idx]->msg == NULL){
            LOG_E("malloc failed!");
            return NULL;
        }
        topic_obj[idx]->sem = rt_sem_create(name, 1, RT_IPC_FLAG_PRIO);
        rt_strcpy(topic_obj[idx]->name, name);
        pub->tp = topic_obj[idx];
        pub->topic_name = topic_obj[idx]->name;
        pub->len = len;
        idx++;
    }
    else{
        pub->tp = topic_obj[check_num - 1];
        pub->topic_name = topic_obj[check_num - 1]->name;
        pub->len = len;
    }

    return pub;
}

/**
 * @brief 订阅name的话题消息
 *
 * @param name 话题名称
 * @param data 消息长度,通过sizeof()获取
 * @return subscriber_t* 返回订阅者实例
 */
subscriber_t *sub_register(char *name, uint8_t len){
    // 和发布者同样的流程，直接调用发布者的注册函数
    subscriber_t *sub = (subscriber_t *)pub_register(name, len);

    return sub;
}

/**
 * @brief 发布消息
 *
 * @param pub 发布者实例指针
 * @param data 数据指针,将要发布的消息放到此处
 * @return uint8_t 返回值为0说明发布失败,为1说明发布成功
 */
uint8_t pub_push_msg(publisher_t *pub, void *data){
    if(rt_sem_take(pub->tp->sem, RT_WAITING_NO)){
        LOG_W("take sem failed!");
        return 0;
    }
    rt_memcpy(pub->tp->msg, data, pub->len);
    rt_sem_release(pub->tp->sem);
    return 1;
}

/**
 * @brief 获取消息
 *
 * @param sub 订阅者实例指针
 * @param data 数据指针,接收的消息将会放到此处
 * @return uint8_t 返回值为1说明获取到了新的消息
 */
uint8_t sub_get_msg(subscriber_t *sub, void *data){
    rt_memcpy(data, sub->tp->msg, sub->len);
    return 1;
}
