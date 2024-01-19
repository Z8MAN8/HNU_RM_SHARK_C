////
//// Created by 16933 on 2022/4/5.
////
//
//#include "Referee.h"
//
////#define normal 15;
////#define normal 18;
////#define normal 30;
//ext_power_heat_data_t ext_power_heat_data;
//ext_game_robot_status_t ext_game_robot_status;
//
//__weak void referee_task(void *argument)
//{
//    /*int num=359;
//    int supcolour;
//    int spin_colour;//陀螺
//    int fric_colour;//摩擦轮
//    int cap_open_colour;//弹仓
//    int Refresh_flag15=1;//准心更新标志位
//    int Refresh_flag18=0;//准心更新标志位
//    int Refresh_flag30=0;//准心更新标志位*/
////绘制实例参考
////    UI_TCB * myfloat = uiDrawFloatDataSeq("005",1,2,3,10,3,1,700,750,5124,10);
////    UI_TCB * mygraph = uiDrawGraphicsSeq(ARC,"003",1,2,3,5,10,50,400,800,100,100,NONE,10);
////    UI_TCB * mygraph2 = uiDrawGraphicsSeq(LINE,"004",1,2,3,5,10,50,800,500,100,100,NONE,10);
////    UI_TCB * myint = uiDrawIntDataSeq("123",1,2,5,15,1,400,800,123,10);
////    UI_TCB * myint2 = uiDrawIntDataSeq("256",1,2,5,15,1,400,900,456,10);
////    UI_TCB * mystring2 = uiDrawStringDataSeq("365",1,2,3,15,2,600,600,"hahaha",10);
////    UI_TCB * mystring = uiDrawStringDataSeq("324",1,2,3,15,2,600,700,"dangdangdang",10);
//
//    /* USER CODE BEGIN DataAnalysisTask */
//    Referee_system_Init(RX_AgreementData_Buffer0,RX_AgreementData_Buffer1,Agreement_RX_BUF_NUM);
//    graphic_data_struct_t TEST_data;
//
//
//        //准心15m/s
//
////            UI_TCB * l1 = uiDrawGraphicsSeq(LINE,"3m",1,2,3,2,NONE,NONE,933,470,987,470,NONE,100);
////            UI_TCB * l2 = uiDrawGraphicsSeq(LINE,"4m",1,2,3,2,NONE,NONE,936,440,984,440,NONE,150);
////            UI_TCB * l3 = uiDrawGraphicsSeq(LINE,"2.5m",1,2,3,2,NONE,NONE,925,475,995,475,NONE,160);
////            UI_TCB * l4 = uiDrawGraphicsSeq(LINE,"2m",1,2,3,2,NONE,NONE,920,486,1000,486,NONE,170);
////            UI_TCB * l5 = uiDrawGraphicsSeq(LINE,"vertical",1,2,3,2,NONE,NONE,960,600,960,300,NONE,300);
//
//         /*   UI_TCB * a = uiDrawGraphicsSeq(LINE,"4m",1,2,3,2,NONE,NONE,936,440,984,440,NONE,150);
//            UI_TCB * b= uiDrawGraphicsSeq(LINE,"4mv",1,2,3,2,NONE,NONE,973,436,973,444,NONE,200);
//            UI_TCB * c = uiDrawGraphicsSeq(LINE,"3m",1,2,3,2,NONE,NONE,933,470,987,470,NONE,100);
//            UI_TCB * d = uiDrawGraphicsSeq(LINE,"3mv",1,2,3,2,NONE,NONE,978,466,978,474,NONE,300);
//            UI_TCB * e = uiDrawGraphicsSeq(LINE,"2m",1,2,3,2,NONE,NONE,920,486,1000,486,NONE,170);
//            UI_TCB * f = uiDrawGraphicsSeq(LINE,"2mv",1,2,3,2,NONE,NONE,972,482,972,500,NONE,200);
//            UI_TCB * g = uiDrawGraphicsSeq(LINE,"ver",1,2,3,2,NONE,NONE,972,400,972,550,NONE,200);
//        //准心18m/s*/
//
////            UI_TCB * a = uiDrawGraphicsSeq(LINE,"4m",1,2,3,2,NONE,NONE,951,475,996,475,NONE,100);
////            UI_TCB * b= uiDrawGraphicsSeq(LINE,"4mv",1,2,3,2,NONE,NONE,973,471,973,479,NONE,100);
////            UI_TCB * c = uiDrawGraphicsSeq(LINE,"3m",1,2,3,2,NONE,NONE,951,490,1006,490,NONE,100);
////            UI_TCB * d = uiDrawGraphicsSeq(LINE,"3mv",1,2,3,2,NONE,NONE,978,494,978,486,NONE,100);
////            UI_TCB * e = uiDrawGraphicsSeq(LINE,"2m",1,2,3,2,NONE,NONE,931,500,1013,500,NONE,100);
////            UI_TCB * f = uiDrawGraphicsSeq(LINE,"2mv",1,2,3,2,NONE,NONE,972,498,972,502,NONE,100);
////            UI_TCB * g = uiDrawGraphicsSeq(LINE,"ver",1,2,3,2,NONE,NONE,972,400,972,550,NONE,100);
//
//
//          //准心30m/s
//
//
//
//
//
////字符
//   /* UI_TCB * sspin = uiDrawStringDataSeq("sspin",1,2,3,15,2,155,815,"spin",100);
//    UI_TCB * sfric = uiDrawStringDataSeq("sfric",1,2,3,15,2,155,755,"fric",200);
//    UI_TCB * scapopen = uiDrawStringDataSeq("scapopen",1,2,3,15,2,155,695,"cap_open",200);
////    UI_TCB * s4 = uiDrawStringDataSeq("04",1,2,3,15,2,155,635,"s4",300);
////    UI_TCB * s5 = uiDrawStringDataSeq("05",1,2,3,15,2,155,575,"s5",300);
////    UI_TCB * s6 = uiDrawStringDataSeq("s6",1,2,3,14,2,1658,545,"supcap",300);
////显示圆
//    UI_TCB * cspin = uiDrawGraphicsSeq(CIRCLE,"cspin",1,2,3,5,10,50,100,810,NONE,NONE,16,100);
//    UI_TCB * cfric = uiDrawGraphicsSeq(CIRCLE,"cfric",1,2,3,5,10,50,100,750,NONE,NONE,16,200);
//    UI_TCB * ccapopen = uiDrawGraphicsSeq(CIRCLE,"ccapopen",1,2,3,5,10,50,100,690,NONE,NONE,16,200);
//    UI_TCB * c4 = uiDrawGraphicsSeq(CIRCLE,"004",1,2,3,5,10,50,100,630,NONE,NONE,16,300);
//    UI_TCB * c5 = uiDrawGraphicsSeq(CIRCLE,"005",1,2,3,5,10,50,100,570,NONE,NONE,16,310);
////超级电容
//    UI_TCB * supercap = uiDrawGraphicsSeq(ARC,"supercap",1,2,3,5,0,360,1700,540,65,65,NONE,10);
//    *//* Infinite loop */
//    for(;;) {
//        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) //如果当前缓冲区是0，解包0缓冲区，否则解包1缓冲区
//        {
//            Referee_Data_Unpack(RX_AgreementData_Buffer0, &Referee_Data_header, &Referee_Data);
//        } else {
//            Referee_Data_Unpack(RX_AgreementData_Buffer1, &Referee_Data_header, &Referee_Data);
//        }
//       /* for (int i = 0; i < UI_TCBNum; i++) {
//            UI_SendTCBSequence[i]->frequency_cnt -= 1; // 遍历每一个UI绘制任务，并将每个的频率倒数计数器键1
//            if (UI_SendTCBSequence[i]->frequency_cnt == 0)  //如果某个UI绘制任务的倒数器到0，即该执行(draw)这个UI绘制任务
//            {
//                UI_Draw(UI_SendTCBSequence[i]->senddata, UI_SendTCBSequence[i]->size);   //给裁判系统发送该任务对应的绘图数据包
//
//                //UI_Drawnew(UI_SendTCBSequence[i]);
//                UI_SendTCBSequence[i]->frequency_cnt = UI_SendTCBSequence[i]->frequency;    //发送过后将计数器再置回原来的状态
//
//                //以下是用来判断是该执行新增作画还是修改作画操作的逻辑  目的：发5次中4次为修改一次为新增
//                if ((UI_SendTCBSequence[i]->fresh_count) % 5 ==
//                    0)      //如果余5为0则有两种可能，第一次发或者第五次发 为了保证第一次发时是新增，这样才能使后续的修改操作有效
//                {
//                    if (UI_SendTCBSequence[i]->fresh_count == 0)      //保证最后一次还是发的修改操作
//                    {
//                        UI_SendTCBSequence[i]->FOA = 2;
//                        UI_SendTCBSequence[i]->fresh_count =
//                                5 + 1;     //重新开始数数,因为判断结束后会照常-1，如果是5的话判断一结束就会被减成4，这样永远都不存在5，也就不会再新增
//                    } else {
//                        UI_SendTCBSequence[i]->FOA = 1;             //如果是第一次发 则要保证是新增操作
//                    }
//
//                } else          //其余状态都是发的修改操作
//                {
//                    UI_SendTCBSequence[i]->FOA = 2;
//                }
//
//                UI_SendTCBSequence[i]->fresh_count -= 1;  //更新计数器递减
//            }
//            //需要更新的绘图任务写在此处
//            //uiRefreshStringDataSeq(mystring,UI_SendTCBSequence[i],2,3,25,2,1200,500,"li",200);
////            uiRefreshGraphicsSeq(mygraph, UI_SendTCBSequence[i],2,3,5,10,num,800,600,100,100,NONE,10);
//            uiRefreshGraphicsSeq(supercap, UI_SendTCBSequence[i],2,supcolour,5,0,num,1700,540,60,60,NONE,10);
//            uiRefreshGraphicsSeq(cspin, UI_SendTCBSequence[i],2,spin_colour,5,10,50,100,810,NONE,NONE,16,10);
//            uiRefreshGraphicsSeq(cfric, UI_SendTCBSequence[i],2,fric_colour,5,10,50,100,750,NONE,NONE,16,10);
//            uiRefreshGraphicsSeq(ccapopen, UI_SendTCBSequence[i],2,cap_open_colour,5,10,50,100,690,NONE,NONE,16,10);
//            if(fric_flag==1)//弹速为15
//            {
//             if(Refresh_flag15==0)//如果没有更新过
//             {
//                 uiRefreshGraphicsSeq(a, UI_SendTCBSequence[i],2,3,2,NONE,NONE,936,440,984,440,NONE,150);
//                 uiRefreshGraphicsSeq(b, UI_SendTCBSequence[i],2,3,2,NONE,NONE,973,436,973,444,NONE,100);
//                 uiRefreshGraphicsSeq(c, UI_SendTCBSequence[i],2,3,2,NONE,NONE,933,470,987,470,NONE,100);
//                 uiRefreshGraphicsSeq(d, UI_SendTCBSequence[i],2,3,2,NONE,NONE,978,466,978,474,NONE,100);
//                 uiRefreshGraphicsSeq(e, UI_SendTCBSequence[i],2,3,2,NONE,NONE,920,486,1000,486,NONE,170);
//                 uiRefreshGraphicsSeq(f, UI_SendTCBSequence[i],2,3,2,NONE,NONE,972,482,972,500,NONE,100);
//                 uiRefreshGraphicsSeq(g, UI_SendTCBSequence[i],2,3,2,NONE,NONE,972,400,972,550,NONE,100);
//                 Refresh_flag15=1;
//                 Refresh_flag18=0;
//                 Refresh_flag30=0;
//             }
//            }
//            else if(fric_flag==2)//弹速为18
//            {
//              if(Refresh_flag18==0)
//              {
//                  uiRefreshGraphicsSeq(a, UI_SendTCBSequence[i],2,3,2,NONE,NONE,951,475,996,475,NONE,100);
//                  uiRefreshGraphicsSeq(b, UI_SendTCBSequence[i],2,3,2,NONE,NONE,973,471,973,479,NONE,100);
//                  uiRefreshGraphicsSeq(c, UI_SendTCBSequence[i],2,3,2,NONE,NONE,951,490,1006,490,NONE,100);
//                  uiRefreshGraphicsSeq(d, UI_SendTCBSequence[i],2,3,2,NONE,NONE,978,494,978,486,NONE,100);
//                  uiRefreshGraphicsSeq(e, UI_SendTCBSequence[i],2,3,2,NONE,NONE,931,500,1013,500,NONE,100);
//                  uiRefreshGraphicsSeq(f, UI_SendTCBSequence[i],2,3,2,NONE,NONE,972,498,972,502,NONE,100);
//                  uiRefreshGraphicsSeq(g, UI_SendTCBSequence[i],2,3,2,NONE,NONE,972,400,972,550,NONE,100);
//                  Refresh_flag18=1;
//                  Refresh_flag15=0;
//                  Refresh_flag30=0;
//              }
//            }
//            else
//            {
//                if(Refresh_flag30==0)
//                {
//                    Refresh_flag18=0;
//                    Refresh_flag15=0;
//                    Refresh_flag30=1;
//                }
//            }
//            //uiRefreshFloatDataSeq(myfloat,UI_SendTCBSequence[i],2,3,10,3,1,700,800,num,50);
////            uiRefreshIntDataSeq(myint,UI_SendTCBSequence[i],2,5,25,1,650,700,num,20);
////            uiRefreshIntDataSeq(myint2,UI_SendTCBSequence[i],2,5,25,1,650,800,num,10);
//        }
//        //判断底盘是否为陀螺状态，打开为橙色，没打开为绿色
//        if(spin_flag==0)
//            fric_colour=3;
//        else
//            spin_colour=2;
//        num=(PowerData[1]/PowerData[0]-0.45)*360/0.47;
//        if(num<=90&&num>=0)
//        {
//            supcolour=5;
//        }
//            else
//        {
//                supcolour=3;
//        }
//
//        //判断摩擦轮是否打开，打开为橙色，没打开为绿色
//        if(fric_wheel_run==0)
//            fric_colour=3;
//        else
//            fric_colour=2;
//
//        // 判断弹仓盖是否打开，打开为橙色，没打开为绿色
//        if(cap_open_flag==0)
//            cap_open_colour=3;
//        else if(cap_open_flag==1)
//            cap_open_colour=2;
//*/
//
//    }
//}
//__weak void USART6_IRQHandler(void)
//{
//    if(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!=RESET)//如果串口中断开启
//    {
//        static uint16_t this_time_rx_len = 0;
//        __HAL_UART_CLEAR_IDLEFLAG(&huart6);      //清除空闲中断
//        if((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) //如果当前的缓冲区是缓冲区0
//        {
//            //计算这一帧接收的数据的长度
//            __HAL_DMA_DISABLE(&hdma_usart6_rx);
//            this_time_rx_len = Agreement_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
//            //重新设定数据长度
//            hdma_usart6_rx.Instance->NDTR = Agreement_RX_BUF_NUM;
//            //把缓冲区设置成缓冲区1
//            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
//            __HAL_DMA_ENABLE(&hdma_usart6_rx);
//            //将这1帧数据放入fifo0
//            fifo_s_puts(&RX_AgreementData_FIFO,(char *)RX_AgreementData_Buffer0,this_time_rx_len);
//        }
//        else //如果当前的缓冲区是缓冲区1
//        {
//            //计算这一帧接收的数据的长度
//            __HAL_DMA_DISABLE(&hdma_usart6_rx);
//            this_time_rx_len = Agreement_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
//            //osSemaphoreRelease(RefereeRxOKHandle);  //释放信号量
//            //重新设定数据长度
//            hdma_usart6_rx.Instance->NDTR = Agreement_RX_BUF_NUM;
//            //把缓冲区设置成缓冲区0
//            hdma_usart6_rx.Instance->CR &= ~DMA_SxCR_CT;
//            __HAL_DMA_ENABLE(&hdma_usart6_rx);
//            fifo_s_puts(&RX_AgreementData_FIFO,(char *)RX_AgreementData_Buffer1,this_time_rx_len);
//        }
//    }
//    HAL_UART_IRQHandler(&huart6);
//
//}
