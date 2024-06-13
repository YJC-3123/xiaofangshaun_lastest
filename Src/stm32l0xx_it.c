/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
/* USER CODE BEGIN EV */
extern uint8_t REC_FLAG;
extern GNRMC GPS;
extern lis3dh_t g_lis3dh;
extern uint8_t NB_4G_State;
extern uint8_t Water_State;
extern float Voleage;
extern bool isGetWP;
extern uint8_t Water_Pre[10];
extern uint8_t it0_count;
extern int32_t g_init_x;
extern int32_t g_init_y;
extern int32_t g_init_z;
extern uint8_t gIMEI[16];

bool err_rpt_flag = false;		//标记从START开始是否已经上报过一次,防止出现异常后重复上报
bool gUsrStartFlag = false;		//标记用户是否通过蓝牙启动设备
uint16_t gTimCounter;
uint16_t gCycleRptCount = 1440;		//每6h周期上报一次（1440个15s）
uint8_t default_lon[] = "113.20E";
uint8_t default_lat[] = "23.9N";
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and line 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */

  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt and DAC1/DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)		//霍尔开关被触发
	{
		HAL_TIM_Base_Stop_IT(&htim6);
		gUsrStartFlag = false;
		if(it0_count % 2 != 0){		//此处防止磁铁靠近和远离临界点处触发两次
			it0_count++;
			return;
		}
		if(it0_count == 100 || it0_count == 101)
			it0_count = 0;
		++it0_count;
		HAL_Delay(10);	//消抖
		BEEP_On(1000);	//蜂鸣器
		
		NB_4G_State = Get_Mode_State();	//获取模式选择状态
		//GNSS_data_parse(uint8_t * buff_t);	//获取当前位置信息，时间
		lis3dh_get_xyz(&g_lis3dh);	//获取姿态信息
		Voleage = get_voleage();		//获取电池电量信息
		Water_State = Get_Water_State();	//获取水浸状态
		//连接远端蓝牙请求获取水压
		isGetWP = false;
		connet_remote_ble();
		HAL_Delay(1000);		//Todo：此处通过延迟保证连接成功，应优化为检查连接状态
		send_remote_ble("#GET_REQ#");
		//等待收到水压消息，蓝牙数据接收中断优先级需高于本中断优先级
		for(uint8_t i=0;i<30;i++)
		{
			HAL_Delay(100);
			if(isGetWP == true)
				break;
		}
		//开启蓝牙广播等待用户连接进行巡检或启动设备
		send_msg_ble("TTM:ADV-1");
		//定时120s后如果用户未通过蓝牙启动则自行启动
		__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
		HAL_TIM_Base_Start_IT(&htim2);
		
	}
	else if(GPIO_Pin == GPIO_PIN_5)		//蓝牙模块需要传输数据
	{
		uint8_t ble_recv_buffer[20];
		memset(ble_recv_buffer,0,sizeof(ble_recv_buffer));
		HAL_UART_Receive(&huart2, ble_recv_buffer, 20, 1000);
		__HAL_UART_FLUSH_DRREGISTER(&huart2);
		//调试：向调试串口发送接收到的信息
		uint8_t debug_msg[128];
		sprintf((char*)debug_msg,"BLE:%s\n",ble_recv_buffer);
		print_u1(debug_msg);
		//解析数据
		uint8_t msg_status = process_remote_ble_recv(ble_recv_buffer);
		if(msg_status == 0)		//用户连接
		{
			return;
		}
		else if(msg_status == 1)	//用户断开连接
		{
			send_msg_ble("TTM:ADV-0");	//用户断开链接关闭广播
		}
		else if(msg_status == 2)	//收到CHECK命令
		{	
			NB_4G_State = Get_Mode_State();		
			//GNSS_data_parse(uint8_t * buff_t);	
			lis3dh_get_xyz(&g_lis3dh);
			Voleage = get_voleage();
			Water_State = Get_Water_State();
			uint8_t msg[200];
			sprintf((char*)msg, "[INFO]NB_4G:%d,Water:%d,Vol:%f,X:%d,Y:%d,Z:%d,Lon:%s,Lat:%s,WP:%s",
			NB_4G_State,Water_State,Voleage, g_lis3dh.x,g_lis3dh.y,g_lis3dh.z,default_lon, default_lat,Water_Pre);
			send_user_ble(msg);
//			//调试
//			sprintf((char*)msg, "[info]NB_4G:%d,Water:%d,Vol:%f,X:%d,Y:%d,Z:%d,Lon:%s,Lat:%s,WP:0.1234",
//				NB_4G_State,Water_State,Voleage, g_lis3dh.x,g_lis3dh.y,g_lis3dh.z,default_lon, default_lat);
//			send_user_ble(msg);
		}
		else if(msg_status == 3)		//收到START命令
		{
			//START前最后一次巡检的状态为初始状态,若用户未巡检则为霍尔开关触发时采集的信息
			g_init_x = g_lis3dh.x;
			g_init_y = g_lis3dh.y;
			g_init_z = g_lis3dh.z;
			uint8_t msg[256];
			sprintf((char*)msg, "#[normal]NB_4G:%d,Water:%d,Vol:%f,X:%d,Y:%d,Z:%d,Lon:%s,Lat:%s,Water_P:%s,IMEI:%s#",
				NB_4G_State,Water_State,Voleage, g_lis3dh.x,g_lis3dh.y,g_lis3dh.z,default_lon, default_lat,Water_Pre, gIMEI);
			send_msg_nbtcp_server(msg);
			err_rpt_flag = false;
			gTimCounter = 0;
			__HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
			HAL_TIM_Base_Start_IT(&htim6);
			gUsrStartFlag = true;
		}

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == htim6.Instance){		//15s定时中断
			//检查系统信息
			lis3dh_get_xyz(&g_lis3dh);
			Voleage = get_voleage();
			Water_State = Get_Water_State();
			isGetWP = false;
			connet_remote_ble();
			HAL_Delay(1000);
			send_remote_ble("#GET_REQ#");
			for(uint8_t i=0;i<30;i++)
			{
				HAL_Delay(100);
				if(isGetWP == true)
					break;
			}
			if(err_rpt_flag==false && check_error()==true){		//启动后第一次异常上报
				uint8_t msg[256];
				//TODO:返回错误类型
				sprintf((char*)msg, "#[error]NB_4G:%d,Water:%d,Vol:%f,X:%d,Y:%d,Z:%d,Lon:%s,Lat:%s,Water_P:%s,IMEI:%s#",
					NB_4G_State,Water_State,Voleage, g_lis3dh.x,g_lis3dh.y,g_lis3dh.z,default_lon, default_lat,Water_Pre,gIMEI);
				send_msg_nbtcp_server(msg);
				err_rpt_flag = true;
			}
			else{
				if(gTimCounter == gCycleRptCount)		//6h定时上报
				{
					gTimCounter = 0;
					uint8_t msg[256];
					sprintf((char*)msg, "#[normal]NB_4G:%d,Water:%d,Vol:%f,X:%d,Y:%d,Z:%d,Lon:%s,Lat:%s,Water_P:%s,IMEI:%s#",
						NB_4G_State,Water_State,Voleage, g_lis3dh.x,g_lis3dh.y,g_lis3dh.z,default_lon, default_lat,Water_Pre,gIMEI);
					send_msg_nbtcp_server(msg);
				}
			}
			gTimCounter += 1;
	}
	else if(htim->Instance == htim2.Instance)		//用户蓝牙主动激活120s等待中断
	{
			HAL_TIM_Base_Stop(&htim6);
			HAL_TIM_Base_Stop(&htim2);
			if(gUsrStartFlag == false){
				print_u1("enter tim2 it\r\n");
				//主动激活120s内若未通过蓝牙启动则直接启动
				g_init_x = g_lis3dh.x;
				g_init_y = g_lis3dh.y;
				g_init_z = g_lis3dh.z;
				uint8_t msg[256];
				sprintf((char*)msg, "#[normal]NB_4G:%d,Water:%d,Vol:%f,X:%d,Y:%d,Z:%d,Lon:%s,Lat:%s,Water_P:%s,IMEI:%s#",
					NB_4G_State,Water_State,Voleage, g_lis3dh.x,g_lis3dh.y,g_lis3dh.z,default_lon, default_lat,Water_Pre, gIMEI);
				send_msg_nbtcp_server(msg);
				err_rpt_flag = false;
				gTimCounter = 0;
				send_msg_ble("TTM:ADV-0");
				__HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
				HAL_TIM_Base_Start_IT(&htim6);
			}
	}
}

/**
  * @brief 串口中断外部回调，串口信息接收暂时使用串行接收，不使用中断接收
  * @note  中断初始化时设置的接收字节数为1，当收到1字节数据触发中断，并将缓冲区的内容搬移到完整数据缓冲区
  * @param huart UART句柄.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)//如果是串口1(调试串口)
	{
		if((USART1_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART1_RX_STA&0x4000)//接收到了0x0d
			{
				if(u1_aRxBuffer[0]!=0x0a)
					USART1_RX_STA=0;//接收错误,重新开始
				else 
					USART1_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D,则将HAL接收函数的单字节缓冲内容搬到完整数据缓冲
			{	
				if(u1_aRxBuffer[0]==0x0d)
				{
					USART1_RX_STA|=0x4000;
				}
				else
				{
					USART1_RX_BUF[USART1_RX_STA&0X3FFF]=u1_aRxBuffer[0] ;
					USART1_RX_STA++;
					if(USART1_RX_STA>(USART_REC_LEN-1))
							USART1_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}
	}
	else if(huart->Instance == LPUART1) {		//低功耗串口(地理位置串口)
		if((LPUSART1_RX_STA&0x8000)==0)//接收未完成
		{
			if(LPUSART1_RX_STA&0x4000)//接收到了0x0d
			{
				if(lp1_aRxBuffer[0]!=0x0a)
					LPUSART1_RX_STA=0;//接收错误,重新开始
				else 
					LPUSART1_RX_STA|=0x8000;	//接收完成了
		
			}
			else //还没收到0X0D,则将HAL接收函数的单字节缓冲内容搬到完整数据缓冲
			{	
				if(lp1_aRxBuffer[0]==0x0d)
				{
					LPUSART1_RX_STA|=0x4000;
				}
				else
				{
					LPUSART1_RX_BUF[LPUSART1_RX_STA&0X3FFF]=lp1_aRxBuffer[0] ;
					LPUSART1_RX_STA++;
					if(LPUSART1_RX_STA>(USART_REC_LEN-1))
							LPUSART1_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}
	} 
}

/* USER CODE END 1 */
