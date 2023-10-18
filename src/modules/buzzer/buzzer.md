# BUZZER  说明文档。

## 文件组成

### 定义结构体buzzer_obj，内部添加了句柄、pwm通道、buzzer_on 、buzzer_off。

```c
struct rt_device_pwm* pwm_dev = (struct rt_device_pwm*)rt_device_find(pwm_name);  
if(pwm_dev == RT_NULL) {  
LOG_W("%s not found", pwm_name);  
}  
object->pwm_channel = pwm_channel;  
object->pwm_dev = pwm_dev;  
object->buzzer_on = buzzer_on;  
object->buzzer_off = buzzer_off;  
object->ring_flag = 0;  
buzzer_obj = object;  

return object;  
}
```

#### 加入句柄解决了将蜂鸣器的初始设置单独写成函数而其他函数接收不到句柄的问题。

#### 加入了buzzer_on 、buzzer_off ，可直接通过对象调用。

## 函数解析

### 蜂鸣器注册

#### 想要使用蜂鸣器时调用此函数来获得句柄

##### 定义一个参数并分配内存块（储存结构体内部的具体数值）-> 将这个函数内存块内部数值全部初始化，防止初始数据影响后续数据设置的准确性 -> 查找设备并定义句柄 -> 设置结构体内部具体数值于储存参数中 -> 函数返回此参数

### 蜂鸣器开启

#### 定义句柄等于注册函数中的设置的句柄 -> 使能pwm -> 在循环中设置pwm的频率和脉冲宽度使其鸣叫

### 蜂鸣器关闭

#### 定义句柄等于注册函数中的设置的句柄 -> 失能pwm

## 使用实例

```c
static buzzer_obj_t *obj;  //设置蜂鸣器对象
obj =buzzer_register(PWM_BUZZER); //获得句柄。
obj->buzzer_on(obj, 6, 30, 80, 1000000); //设置蜂鸣器对象、鸣叫次数、每次鸣叫时间、音量、频率。
```
