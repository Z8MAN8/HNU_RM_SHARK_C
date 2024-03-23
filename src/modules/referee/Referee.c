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
//robot_status_t ext_game_robot_status;
//
//__weak void referee_task(void *argument)
//{
//    /*int num=359;
//    int supcolour;
//    int spin_colour;//����
//    int fric_colour;//Ħ����
//    int cap_open_colour;//����
//    int Refresh_flag15=1;//׼�ĸ��±�־λ
//    int Refresh_flag18=0;//׼�ĸ��±�־λ
//    int Refresh_flag30=0;//׼�ĸ��±�־λ*/
////����ʵ���ο�
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
//        //׼��15m/s
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
//        //׼��18m/s*/
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
//          //׼��30m/s
//
//
//
//
//
////�ַ�
//   /* UI_TCB * sspin = uiDrawStringDataSeq("sspin",1,2,3,15,2,155,815,"spin",100);
//    UI_TCB * sfric = uiDrawStringDataSeq("sfric",1,2,3,15,2,155,755,"fric",200);
//    UI_TCB * scapopen = uiDrawStringDataSeq("scapopen",1,2,3,15,2,155,695,"cap_open",200);
////    UI_TCB * s4 = uiDrawStringDataSeq("04",1,2,3,15,2,155,635,"s4",300);
////    UI_TCB * s5 = uiDrawStringDataSeq("05",1,2,3,15,2,155,575,"s5",300);
////    UI_TCB * s6 = uiDrawStringDataSeq("s6",1,2,3,14,2,1658,545,"supcap",300);
////��ʾԲ
//    UI_TCB * cspin = uiDrawGraphicsSeq(CIRCLE,"cspin",1,2,3,5,10,50,100,810,NONE,NONE,16,100);
//    UI_TCB * cfric = uiDrawGraphicsSeq(CIRCLE,"cfric",1,2,3,5,10,50,100,750,NONE,NONE,16,200);
//    UI_TCB * ccapopen = uiDrawGraphicsSeq(CIRCLE,"ccapopen",1,2,3,5,10,50,100,690,NONE,NONE,16,200);
//    UI_TCB * c4 = uiDrawGraphicsSeq(CIRCLE,"004",1,2,3,5,10,50,100,630,NONE,NONE,16,300);
//    UI_TCB * c5 = uiDrawGraphicsSeq(CIRCLE,"005",1,2,3,5,10,50,100,570,NONE,NONE,16,310);
////��������
//    UI_TCB * supercap = uiDrawGraphicsSeq(ARC,"supercap",1,2,3,5,0,360,1700,540,65,65,NONE,10);
//    *//* Infinite loop */
//    for(;;) {
//        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) //�����ǰ��������0�����0��������������1������
//        {
//            Referee_Data_Unpack(RX_AgreementData_Buffer0, &Referee_Data_header, &Referee_Data);
//        } else {
//            Referee_Data_Unpack(RX_AgreementData_Buffer1, &Referee_Data_header, &Referee_Data);
//        }
//       /* for (int i = 0; i < UI_TCBNum; i++) {
//            UI_SendTCBSequence[i]->frequency_cnt -= 1; // ����ÿһ��UI�������񣬲���ÿ����Ƶ�ʵ�����������1
//            if (UI_SendTCBSequence[i]->frequency_cnt == 0)  //���ĳ��UI��������ĵ�������0������ִ��(draw)���UI��������
//            {
//                UI_Draw(UI_SendTCBSequence[i]->senddata, UI_SendTCBSequence[i]->size);   //������ϵͳ���͸������Ӧ�Ļ�ͼ���ݰ�
//
//                //UI_Drawnew(UI_SendTCBSequence[i]);
//                UI_SendTCBSequence[i]->frequency_cnt = UI_SendTCBSequence[i]->frequency;    //���͹��󽫼��������û�ԭ����״̬
//
//                //�����������ж��Ǹ�ִ���������������޸������������߼�  Ŀ�ģ���5����4��Ϊ�޸�һ��Ϊ����
//                if ((UI_SendTCBSequence[i]->fresh_count) % 5 ==
//                    0)      //�����5Ϊ0�������ֿ��ܣ���һ�η����ߵ���η� Ϊ�˱�֤��һ�η�ʱ����������������ʹ�������޸Ĳ�����Ч
//                {
//                    if (UI_SendTCBSequence[i]->fresh_count == 0)      //��֤���һ�λ��Ƿ����޸Ĳ���
//                    {
//                        UI_SendTCBSequence[i]->FOA = 2;
//                        UI_SendTCBSequence[i]->fresh_count =
//                                5 + 1;     //���¿�ʼ����,��Ϊ�жϽ�������ճ�-1�������5�Ļ��ж�һ�����ͻᱻ����4��������Զ��������5��Ҳ�Ͳ���������
//                    } else {
//                        UI_SendTCBSequence[i]->FOA = 1;             //����ǵ�һ�η� ��Ҫ��֤����������
//                    }
//
//                } else          //����״̬���Ƿ����޸Ĳ���
//                {
//                    UI_SendTCBSequence[i]->FOA = 2;
//                }
//
//                UI_SendTCBSequence[i]->fresh_count -= 1;  //���¼������ݼ�
//            }
//            //��Ҫ���µĻ�ͼ����д�ڴ˴�
//            //uiRefreshStringDataSeq(mystring,UI_SendTCBSequence[i],2,3,25,2,1200,500,"li",200);
////            uiRefreshGraphicsSeq(mygraph, UI_SendTCBSequence[i],2,3,5,10,num,800,600,100,100,NONE,10);
//            uiRefreshGraphicsSeq(supercap, UI_SendTCBSequence[i],2,supcolour,5,0,num,1700,540,60,60,NONE,10);
//            uiRefreshGraphicsSeq(cspin, UI_SendTCBSequence[i],2,spin_colour,5,10,50,100,810,NONE,NONE,16,10);
//            uiRefreshGraphicsSeq(cfric, UI_SendTCBSequence[i],2,fric_colour,5,10,50,100,750,NONE,NONE,16,10);
//            uiRefreshGraphicsSeq(ccapopen, UI_SendTCBSequence[i],2,cap_open_colour,5,10,50,100,690,NONE,NONE,16,10);
//            if(fric_flag==1)//����Ϊ15
//            {
//             if(Refresh_flag15==0)//���û�и��¹�
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
//            else if(fric_flag==2)//����Ϊ18
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
//        //�жϵ����Ƿ�Ϊ����״̬����Ϊ��ɫ��û��Ϊ��ɫ
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
//        //�ж�Ħ�����Ƿ�򿪣���Ϊ��ɫ��û��Ϊ��ɫ
//        if(fric_wheel_run==0)
//            fric_colour=3;
//        else
//            fric_colour=2;
//
//        // �жϵ��ָ��Ƿ�򿪣���Ϊ��ɫ��û��Ϊ��ɫ
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
//    if(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!=RESET)//��������жϿ���
//    {
//        static uint16_t this_time_rx_len = 0;
//        __HAL_UART_CLEAR_IDLEFLAG(&huart6);      //��������ж�
//        if((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) //�����ǰ�Ļ������ǻ�����0
//        {
//            //������һ֡���յ����ݵĳ���
//            __HAL_DMA_DISABLE(&hdma_usart6_rx);
//            this_time_rx_len = Agreement_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
//            //�����趨���ݳ���
//            hdma_usart6_rx.Instance->NDTR = Agreement_RX_BUF_NUM;
//            //�ѻ��������óɻ�����1
//            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
//            __HAL_DMA_ENABLE(&hdma_usart6_rx);
//            //����1֡���ݷ���fifo0
//            fifo_s_puts(&RX_AgreementData_FIFO,(char *)RX_AgreementData_Buffer0,this_time_rx_len);
//        }
//        else //�����ǰ�Ļ������ǻ�����1
//        {
//            //������һ֡���յ����ݵĳ���
//            __HAL_DMA_DISABLE(&hdma_usart6_rx);
//            this_time_rx_len = Agreement_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
//            //osSemaphoreRelease(RefereeRxOKHandle);  //�ͷ��ź���
//            //�����趨���ݳ���
//            hdma_usart6_rx.Instance->NDTR = Agreement_RX_BUF_NUM;
//            //�ѻ��������óɻ�����0
//            hdma_usart6_rx.Instance->CR &= ~DMA_SxCR_CT;
//            __HAL_DMA_ENABLE(&hdma_usart6_rx);
//            fifo_s_puts(&RX_AgreementData_FIFO,(char *)RX_AgreementData_Buffer1,this_time_rx_len);
//        }
//    }
//    HAL_UART_IRQHandler(&huart6);
//
//}
