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

bool err_rpt_flag = false;		//��Ǵ�START��ʼ�Ƿ��Ѿ��ϱ���һ��,��ֹ�����쳣���ظ��ϱ�
bool gUsrStartFlag = false;		//����û��Ƿ�ͨ�����������豸
uint16_t gTimCounter;
uint16_t gCycleRptCount = 1440;		//ÿ6h�����ϱ�һ�Σ�1440��15s��
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
	if(GPIO_Pin == GPIO_PIN_0)		//�������ر�����
	{
		HAL_TIM_Base_Stop_IT(&htim6);
		gUsrStartFlag = false;
		if(it0_count % 2 != 0){		//�˴���ֹ����������Զ���ٽ�㴦��������
			it0_count++;
			return;
		}
		if(it0_count == 100 || it0_count == 101)
			it0_count = 0;
		++it0_count;
		HAL_Delay(10);	//����
		BEEP_On(1000);	//������
		
		NB_4G_State = Get_Mode_State();	//��ȡģʽѡ��״̬
		//GNSS_data_parse(uint8_t * buff_t);	//��ȡ��ǰλ����Ϣ��ʱ��
		lis3dh_get_xyz(&g_lis3dh);	//��ȡ��̬��Ϣ
		Voleage = get_voleage();		//��ȡ��ص�����Ϣ
		Water_State = Get_Water_State();	//��ȡˮ��״̬
		//����Զ�����������ȡˮѹ
		isGetWP = false;
		connet_remote_ble();
		HAL_Delay(1000);		//Todo���˴�ͨ���ӳٱ�֤���ӳɹ���Ӧ�Ż�Ϊ�������״̬
		send_remote_ble("#GET_REQ#");
		//�ȴ��յ�ˮѹ��Ϣ���������ݽ����ж����ȼ�����ڱ��ж����ȼ�
		for(uint8_t i=0;i<30;i++)
		{
			HAL_Delay(100);
			if(isGetWP == true)
				break;
		}
		//���������㲥�ȴ��û����ӽ���Ѳ��������豸
		send_msg_ble("TTM:ADV-1");
		//��ʱ120s������û�δͨ��������������������
		__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
		HAL_TIM_Base_Start_IT(&htim2);
		
	}
	else if(GPIO_Pin == GPIO_PIN_5)		//����ģ����Ҫ��������
	{
		uint8_t ble_recv_buffer[20];
		memset(ble_recv_buffer,0,sizeof(ble_recv_buffer));
		HAL_UART_Receive(&huart2, ble_recv_buffer, 20, 1000);
		__HAL_UART_FLUSH_DRREGISTER(&huart2);
		//���ԣ�����Դ��ڷ��ͽ��յ�����Ϣ
		uint8_t debug_msg[128];
		sprintf((char*)debug_msg,"BLE:%s\n",ble_recv_buffer);
		print_u1(debug_msg);
		//��������
		uint8_t msg_status = process_remote_ble_recv(ble_recv_buffer);
		if(msg_status == 0)		//�û�����
		{
			return;
		}
		else if(msg_status == 1)	//�û��Ͽ�����
		{
			send_msg_ble("TTM:ADV-0");	//�û��Ͽ����ӹرչ㲥
		}
		else if(msg_status == 2)	//�յ�CHECK����
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
//			//����
//			sprintf((char*)msg, "[info]NB_4G:%d,Water:%d,Vol:%f,X:%d,Y:%d,Z:%d,Lon:%s,Lat:%s,WP:0.1234",
//				NB_4G_State,Water_State,Voleage, g_lis3dh.x,g_lis3dh.y,g_lis3dh.z,default_lon, default_lat);
//			send_user_ble(msg);
		}
		else if(msg_status == 3)		//�յ�START����
		{
			//STARTǰ���һ��Ѳ���״̬Ϊ��ʼ״̬,���û�δѲ����Ϊ�������ش���ʱ�ɼ�����Ϣ
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
	if(htim->Instance == htim6.Instance){		//15s��ʱ�ж�
			//���ϵͳ��Ϣ
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
			if(err_rpt_flag==false && check_error()==true){		//�������һ���쳣�ϱ�
				uint8_t msg[256];
				//TODO:���ش�������
				sprintf((char*)msg, "#[error]NB_4G:%d,Water:%d,Vol:%f,X:%d,Y:%d,Z:%d,Lon:%s,Lat:%s,Water_P:%s,IMEI:%s#",
					NB_4G_State,Water_State,Voleage, g_lis3dh.x,g_lis3dh.y,g_lis3dh.z,default_lon, default_lat,Water_Pre,gIMEI);
				send_msg_nbtcp_server(msg);
				err_rpt_flag = true;
			}
			else{
				if(gTimCounter == gCycleRptCount)		//6h��ʱ�ϱ�
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
	else if(htim->Instance == htim2.Instance)		//�û�������������120s�ȴ��ж�
	{
			HAL_TIM_Base_Stop(&htim6);
			HAL_TIM_Base_Stop(&htim2);
			if(gUsrStartFlag == false){
				print_u1("enter tim2 it\r\n");
				//��������120s����δͨ������������ֱ������
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
  * @brief �����ж��ⲿ�ص���������Ϣ������ʱʹ�ô��н��գ���ʹ���жϽ���
  * @note  �жϳ�ʼ��ʱ���õĽ����ֽ���Ϊ1�����յ�1�ֽ����ݴ����жϣ����������������ݰ��Ƶ��������ݻ�����
  * @param huart UART���.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)//����Ǵ���1(���Դ���)
	{
		if((USART1_RX_STA&0x8000)==0)//����δ���
		{
			if(USART1_RX_STA&0x4000)//���յ���0x0d
			{
				if(u1_aRxBuffer[0]!=0x0a)
					USART1_RX_STA=0;//���մ���,���¿�ʼ
				else 
					USART1_RX_STA|=0x8000;	//��������� 
			}
			else //��û�յ�0X0D,��HAL���պ����ĵ��ֽڻ������ݰᵽ�������ݻ���
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
							USART1_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}
	}
	else if(huart->Instance == LPUART1) {		//�͹��Ĵ���(����λ�ô���)
		if((LPUSART1_RX_STA&0x8000)==0)//����δ���
		{
			if(LPUSART1_RX_STA&0x4000)//���յ���0x0d
			{
				if(lp1_aRxBuffer[0]!=0x0a)
					LPUSART1_RX_STA=0;//���մ���,���¿�ʼ
				else 
					LPUSART1_RX_STA|=0x8000;	//���������
		
			}
			else //��û�յ�0X0D,��HAL���պ����ĵ��ֽڻ������ݰᵽ�������ݻ���
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
							LPUSART1_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}
	} 
}

/* USER CODE END 1 */