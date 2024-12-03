/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h" 
#include "stdio.h" 
#include "string.h"
#include <math.h>
#include "common.h"
#include "MPU6050.h"
#include "stdio.h"//
#include "HMC5883L.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADC_MAX_NUM 4*5 //3组ADC,每组最多存储5个值
  uint8_t buff;
uint16_t ADC_Values[ADC_MAX_NUM]={0};
uint8_t RxBuff[1];      //进入中断接收数据的数组
uint8_t DataBuff[5000]; //保存接收到的数据的数组
int RxLine=0;           //接收到的数据长度
uint8_t singalset = 0;
uint8_t C1512set = 0;
uint8_t C118set = 0;
/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void dong_start_adc(){
    
    //启动DMA
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC_Values,ADC_MAX_NUM);
    
}
uint8_t usart1_buf[4]={0};
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//	
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t value = 0;
  uint16_t AdcValue = 0;
	char str[30];
	
	uint16_t ADC_Value1,i;
	uint16_t AD_Buf[10]={0};//两个通道采集数据存在这个数组里面
	float press1;//两个通道采集数据存在这个数组里面
	float press2;
	float press3;
	float press4;
	int  x1 ;
	int  x2 ;
	int  x3 ;
	int  x4 ;
	int  x5 ;
	int  x6 ;
	int  x7 ;
	int  x8 ;
	int  x9 ;
	int  x10 ;
	int  x11 ;
	int  x12 ;
  int singal1,singal2,singal3,singal4,singal5;
	int C15[8],C14[8],C13[8],C12[8],C11[8],C10[8],C9[8],C8[8];
	short x;
  short y;
  short z;
	int16_t ax, ay, az;	
  int16_t gx, gy, gz;
	float ypr[3]; // yaw pitch roll
	int n = 1;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //dong_start_adc();
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&AD_Buf,10);//开启ADC的DMA，采集的数据直接放入AD_Buf这个数组里，操作简单。
  MPU6050_Init(0xD0);  
  HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuff, 1); //打开串口中断接收
//  HMC5883L_SetUp();
//	HAL_Delay(50);
//	HMC5883L_SetUp();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		if(HAL_UART_Receive(&huart1,buff,sizeof(buff),1000) == HAL_OK)
//		{
//		  HAL_UART_Transmit(&huart1,buff,sizeof(buff),100);
//		}
		//printf("x: ");
    //HMC58X3_mgetValues(&ypr[3]);
//		 HMC58X3_mgetValues(ypr);
//		MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		//MPU6050_getlastMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//	  printf("1 \r\n");
		//printf("%d %d %d \r\n",gx,gy,gz);
//	  delay_us(40);
		//printf("ds");
		//AHRS_getYawPitchRoll(ypr); //姿态更新
		//printf("%.0f %.0f %.0f \r\n",ypr[1]* 180/M_PI,ypr[2]* 180/M_PI,ypr[0]* 180/M_PI);
//		HAL_Delay(10);
//      HAL_UART_Transmit(&huart1,(uint8_t *)"abcde",4,0xffff);

   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
   //HAL_Delay(10);
	 x1 = AD_Buf[0];
	 x2 = AD_Buf[1];
	 x3 = AD_Buf[2];
	 x4 = AD_Buf[3];
	 x5 = AD_Buf[4];
	 x6 = AD_Buf[5];
	 x7 = AD_Buf[6];
	 x8 = AD_Buf[7];
	 x9 = AD_Buf[8];
	 x10 = AD_Buf[9];
//     

		if(singalset == 1&&C1512set == 0&&C118set == 0)
		{
	//	获得单点压力值
				if(n%5==1)
				{//c7
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				singal5 = x1;
				}    
				if(n%5==2)
				{//c4
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				singal1 = x1;
				}
				if(n%5==3)
				{//c2
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				singal2 = x1;
				}
				if(n%5==4)
				{//c1
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				singal3 = x1;
				}
				if(n%5==0)
				{//c0
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				singal4 = x1;
				printf("%d %d %d %d %d \r\n",singal1,singal2,singal3,singal4,singal5);
				}
		}
		
		if(singalset == 0&&C1512set == 1&&C118set == 0)
		{
				if(n%4==1)
				{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_Delay(1);
				C12[0]=x3,C12[1]=x4,C12[2]=x5,C12[3]=x6,C12[4]=x7,C12[5]=x8,C12[6]=x9,C12[7]=x10;
				printf("C12 %d %d %d %d %d %d %d %d \r\n",C12[0],C12[1],C12[2],C12[3],C12[4],C12[5],C12[6],C12[7]);

				}    
				if(n%4==2)
				{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				 C15[0]=x3,C15[1]=x4,C15[2]=x5,C15[3]=x6,C15[4]=x7,C15[5]=x8,C15[6]=x9,C15[7]=x10;
				printf("C15 %d %d %d %d %d %d %d %d \r\n",C15[0],C15[1],C15[2],C15[3],C15[4],C15[5],C15[6],C15[7]);

				}
				if(n%4==3)
				{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C14[0]=x3,C14[1]=x4,C14[2]=x5,C14[3]=x6,C14[4]=x7,C14[5]=x8,C14[6]=x9,C14[7]=x10;
				printf("C14 %d %d %d %d %d %d %d %d \r\n",C14[0],C14[1],C14[2],C14[3],C14[4],C14[5],C14[6],C14[7]);

				}
				if(n%4==0)
				{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C13[0]=x3,C13[1]=x4,C13[2]=x5,C13[3]=x6,C13[4]=x7,C13[5]=x8,C13[6]=x9,C13[7]=x10;
				printf("C13 %d %d %d %d %d %d %d %d \r\n",C13[0],C13[1],C13[2],C13[3],C13[4],C13[5],C13[6],C13[7]);
				}
		}
		if(singalset == 0&&C1512set == 0&&C118set == 1)
		{
				if(n%4==1)
				{
				 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C8[0]=x3,C8[1]=x4,C8[2]=x5,C8[3]=x6,C8[4]=x7,C8[5]=x8,C8[6]=x9,C8[7]=x10;
				printf("C8 %d %d %d %d %d %d %d %d \r\n",C8[0],C8[1],C8[2],C8[3],C8[4],C8[5],C8[6],C8[7]);

				}    
				if(n%4==2)
				{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				 C11[0]=x3,C11[1]=x4,C11[2]=x5,C11[3]=x6,C11[4]=x7,C11[5]=x8,C11[6]=x9,C11[7]=x10;
				printf("C11 %d %d %d %d %d %d %d %d \r\n",C11[0],C11[1],C11[2],C11[3],C11[4],C11[5],C11[6],C11[7]);

				}
				if(n%4==3)
				{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C10[0]=x3,C10[1]=x4,C10[2]=x5,C10[3]=x6,C10[4]=x7,C10[5]=x8,C10[6]=x9,C10[7]=x10;
				printf("C10 %d %d %d %d %d %d %d %d \r\n",C10[0],C10[1],C10[2],C10[3],C10[4],C10[5],C10[6],C10[7]);

				}
				if(n%4==0)
				{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C9[0]=x3,C9[1]=x4,C9[2]=x5,C9[3]=x6,C9[4]=x7,C9[5]=x8,C9[6]=x9,C9[7]=x10;
				printf("C9 %d %d %d %d %d %d %d %d \r\n",C9[0],C9[1],C9[2],C9[3],C9[4],C9[5],C9[6],C9[7]);
				}
		}
	
	
		if(singalset == 1&&C1512set == 1&&C118set == 0)
		{
				if(n%9==1)
				{//c7
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				C12[0]=x3,C12[1]=x4,C12[2]=x5,C12[3]=x6,C12[4]=x7,C12[5]=x8,C12[6]=x9,C12[7]=x10;
				printf("C12 %d %d %d %d %d %d %d %d \r\n",C12[0],C12[1],C12[2],C12[3],C12[4],C12[5],C12[6],C12[7]);
				}    
				if(n%9==2)
				{//c4
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				singal1 = x1;
				}
				if(n%9==3)
				{//c2
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				singal2 = x1;
				}
				if(n%9==4)
				{//c1
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				singal3 = x1;
				}
				if(n%9==5)
				{//c0
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				singal4 = x1;
				}
				if(n%9==6)
				{//C15
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_Delay(1);
				singal5 = x1;
				printf("%d %d %d %d %d \r\n",singal1,singal2,singal3,singal4,singal5);
				//C12[0]=x3,C12[1]=x4,C12[2]=x5,C12[3]=x6,C12[4]=x7,C12[5]=x8,C12[6]=x9,C12[7]=x10;
				}    
				if(n%9==7)
				{//c14
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				 C15[0]=x3,C15[1]=x4,C15[2]=x5,C15[3]=x6,C15[4]=x7,C15[5]=x8,C15[6]=x9,C15[7]=x10;
				printf("C15 %d %d %d %d %d %d %d %d \r\n",C15[0],C15[1],C15[2],C15[3],C15[4],C15[5],C15[6],C15[7]);

				}
				if(n%9==8)
				{//c13
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C14[0]=x3,C14[1]=x4,C14[2]=x5,C14[3]=x6,C14[4]=x7,C14[5]=x8,C14[6]=x9,C14[7]=x10;
				printf("C14 %d %d %d %d %d %d %d %d \r\n",C14[0],C14[1],C14[2],C14[3],C14[4],C14[5],C14[6],C14[7]);

				}
				if(n%9==0)
				{//c12
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C13[0]=x3,C13[1]=x4,C13[2]=x5,C13[3]=x6,C13[4]=x7,C13[5]=x8,C13[6]=x9,C13[7]=x10;
				printf("C13 %d %d %d %d %d %d %d %d \r\n",C13[0],C13[1],C13[2],C13[3],C13[4],C13[5],C13[6],C13[7]);

				}
		}
	
		if(singalset == 0&&C1512set == 1&&C118set == 1)
		{
				if(n%8==1)
				{//C15
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_Delay(1);
				C8[0]=x3,C8[1]=x4,C8[2]=x5,C8[3]=x6,C8[4]=x7,C8[5]=x8,C8[6]=x9,C8[7]=x10;
				printf("C8 %d %d %d %d %d %d %d %d \r\n",C8[0],C8[1],C8[2],C8[3],C8[4],C8[5],C8[6],C8[7]);
				}    
				if(n%8==2)
				{//c14
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C15[0]=x3,C15[1]=x4,C15[2]=x5,C15[3]=x6,C15[4]=x7,C15[5]=x8,C15[6]=x9,C15[7]=x10;
				printf("C15 %d %d %d %d %d %d %d %d \r\n",C15[0],C15[1],C15[2],C15[3],C15[4],C15[5],C15[6],C15[7]);

				}
				if(n%8==3)
				{//c13
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C14[0]=x3,C14[1]=x4,C14[2]=x5,C14[3]=x6,C14[4]=x7,C14[5]=x8,C14[6]=x9,C14[7]=x10;
				printf("C14 %d %d %d %d %d %d %d %d \r\n",C14[0],C14[1],C14[2],C14[3],C14[4],C14[5],C14[6],C14[7]);

				}
				if(n%8==4)
				{//c12
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C13[0]=x3,C13[1]=x4,C13[2]=x5,C13[3]=x6,C13[4]=x7,C13[5]=x8,C13[6]=x9,C13[7]=x10;
				printf("C13 %d %d %d %d %d %d %d %d \r\n",C13[0],C13[1],C13[2],C13[3],C13[4],C13[5],C13[6],C13[7]);

				}
				if(n%8==5)
				{//c11
				 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C12[0]=x3,C12[1]=x4,C12[2]=x5,C12[3]=x6,C12[4]=x7,C12[5]=x8,C12[6]=x9,C12[7]=x10;
				printf("C12 %d %d %d %d %d %d %d %d \r\n",C12[0],C12[1],C12[2],C12[3],C12[4],C12[5],C12[6],C12[7]);

				}    
				if(n%8==6)
				{//c10
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				 C11[0]=x3,C11[1]=x4,C11[2]=x5,C11[3]=x6,C11[4]=x7,C11[5]=x8,C11[6]=x9,C11[7]=x10;
				printf("C11 %d %d %d %d %d %d %d %d \r\n",C11[0],C11[1],C11[2],C11[3],C11[4],C11[5],C11[6],C11[7]);

				}
				if(n%8==7)
				{//c9
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C10[0]=x3,C10[1]=x4,C10[2]=x5,C10[3]=x6,C10[4]=x7,C10[5]=x8,C10[6]=x9,C10[7]=x10;
				printf("C10 %d %d %d %d %d %d %d %d \r\n",C10[0],C10[1],C10[2],C10[3],C10[4],C10[5],C10[6],C10[7]);

				}
				if(n%8==0)
				{//c8
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C9[0]=x3,C9[1]=x4,C9[2]=x5,C9[3]=x6,C9[4]=x7,C9[5]=x8,C9[6]=x9,C9[7]=x10;
				printf("C9 %d %d %d %d %d %d %d %d \r\n",C9[0],C9[1],C9[2],C9[3],C9[4],C9[5],C9[6],C9[7]);

				}
		}
	
		if(singalset == 1&&C1512set == 0&&C118set == 1)
		{
			 if(n%9==1)
				{//c7
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				C8[0]=x3,C8[1]=x4,C8[2]=x5,C8[3]=x6,C8[4]=x7,C8[5]=x8,C8[6]=x9,C8[7]=x10;
				printf("C8 %d %d %d %d %d %d %d %d \r\n",C8[0],C8[1],C8[2],C8[3],C8[4],C8[5],C8[6],C8[7]);
				}    
				if(n%9==2)
				{//c4
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				singal1 = x1;
				}
				if(n%9==3)
				{//c2
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				singal2 = x1;
				}
				if(n%9==4)
				{//c1
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				singal3 = x1;
				}
				if(n%9==5)
				{//c0
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				singal4 = x1;
				}
				if(n%9==6)
				{//c11
				 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				singal5 = x1;
				printf("%d %d %d %d %d \r\n",singal1,singal2,singal3,singal4,singal5);
				}    
				if(n%9==7)
				{//c10
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				 C11[0]=x3,C11[1]=x4,C11[2]=x5,C11[3]=x6,C11[4]=x7,C11[5]=x8,C11[6]=x9,C11[7]=x10;
				printf("C11 %d %d %d %d %d %d %d %d \r\n",C11[0],C11[1],C11[2],C11[3],C11[4],C11[5],C11[6],C11[7]);

				}
				if(n%9==8)
				{//c9
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C10[0]=x3,C10[1]=x4,C10[2]=x5,C10[3]=x6,C10[4]=x7,C10[5]=x8,C10[6]=x9,C10[7]=x10;
				printf("C10 %d %d %d %d %d %d %d %d \r\n",C10[0],C10[1],C10[2],C10[3],C10[4],C10[5],C10[6],C10[7]);

				}
				if(n%9==0)
				{//c8
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C9[0]=x3,C9[1]=x4,C9[2]=x5,C9[3]=x6,C9[4]=x7,C9[5]=x8,C9[6]=x9,C9[7]=x10;
				printf("C9 %d %d %d %d %d %d %d %d \r\n",C9[0],C9[1],C9[2],C9[3],C9[4],C9[5],C9[6],C9[7]);
				}
		}
	
		if(singalset == 1&&C1512set == 1&&C118set == 1)
		{
				
				if(n%13==1)
				{//c7
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				C12[0]=x3,C12[1]=x4,C12[2]=x5,C12[3]=x6,C12[4]=x7,C12[5]=x8,C12[6]=x9,C12[7]=x10;
				printf("C12 %d %d %d %d %d %d %d %d \r\n",C12[0],C12[1],C12[2],C12[3],C12[4],C12[5],C12[6],C12[7]);
				}    
				if(n%13==2)
				{//c4
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				singal1 = x1;
				}
				if(n%13==3)
				{//c2
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				singal2 = x1;
				}
				if(n%13==4)
				{//c1
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				singal3 = x1;
				}
				if(n%13==5)
				{//c0
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_Delay(1);
				singal4 = x1;
				}
				if(n%13==6)
				{//c11
				 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				singal5 = x1;
				printf("%d %d %d %d %d %d \r\n",singal1,singal2,singal3,singal4,singal5,n);
				}    
				if(n%13==7)
				{//c10
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				 C11[0]=x3,C11[1]=x4,C11[2]=x5,C11[3]=x6,C11[4]=x7,C11[5]=x8,C11[6]=x9,C11[7]=x10;
				printf("C11 %d %d %d %d %d %d %d %d \r\n",C11[0],C11[1],C11[2],C11[3],C11[4],C11[5],C11[6],C11[7]);

				}
				if(n%13==8)
				{//c9
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C10[0]=x3,C10[1]=x4,C10[2]=x5,C10[3]=x6,C10[4]=x7,C10[5]=x8,C10[6]=x9,C10[7]=x10;
				printf("C10 %d %d %d %d %d %d %d %d \r\n",C10[0],C10[1],C10[2],C10[3],C10[4],C10[5],C10[6],C10[7]);

				}
				if(n%13==9)
				{//c8
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C9[0]=x3,C9[1]=x4,C9[2]=x5,C9[3]=x6,C9[4]=x7,C9[5]=x8,C9[6]=x9,C9[7]=x10;
				printf("C9 %d %d %d %d %d %d %d %d \r\n",C9[0],C9[1],C9[2],C9[3],C9[4],C9[5],C9[6],C9[7]);

				}
				if(n%13==10)
				{//C15
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_Delay(9);
				C8[0]=x3,C8[1]=x4,C8[2]=x5,C8[3]=x6,C8[4]=x7,C8[5]=x8,C8[6]=x9,C8[7]=x10;
				printf("C8 %d %d %d %d %d %d %d %d \r\n",C8[0],C8[1],C8[2],C8[3],C8[4],C8[5],C8[6],C8[7]);
				}  
				if(n%13==11)
				{//c14
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C15[0]=x3,C15[1]=x4,C15[2]=x5,C15[3]=x6,C15[4]=x7,C15[5]=x8,C15[6]=x9,C15[7]=x10;
				printf("C15 %d %d %d %d %d %d %d %d \r\n",C15[0],C15[1],C15[2],C15[3],C15[4],C15[5],C15[6],C15[7]);

				}
				if(n%13==12)
				{//c13
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C14[0]=x3,C14[1]=x4,C14[2]=x5,C14[3]=x6,C14[4]=x7,C14[5]=x8,C14[6]=x9,C14[7]=x10;
				printf("C14 %d %d %d %d %d %d %d %d \r\n",C14[0],C14[1],C14[2],C14[3],C14[4],C14[5],C14[6],C14[7]);

				}
				if(n%13==0)
				{//c12
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_Delay(1);
				C13[0]=x3,C13[1]=x4,C13[2]=x5,C13[3]=x6,C13[4]=x7,C13[5]=x8,C13[6]=x9,C13[7]=x10;
				printf("C13 %d %d %d %d %d %d %d %d \r\n",C13[0],C13[1],C13[2],C13[3],C13[4],C13[5],C13[6],C13[7]);

				}
		}

			n = n + 1;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    RxLine++;                      //每接收到一个数据，进入回调数据长度加1
    DataBuff[RxLine-1]=RxBuff[0];  //把每次接收到的数据保存到缓存数组
    
    if(RxBuff[0]==0xff)            //接收结束标志位，这个数据可以自定义，根据实际需求，这里只做示例使用，不一定是0xff
    {
        if(DataBuff[0]==0x01)     //单点发送
				{
				  singalset = 1;
				}
				if(DataBuff[0]==0x02)
				{
				  singalset = 0;
				}
				if(DataBuff[1]==0x01)     //一个阵列发送
				{
				  C1512set = 1;
				}
				if(DataBuff[1]==0x02)
				{
				  C1512set = 0;
				}
				if(DataBuff[2]==0x01)     //另一个阵列发送
				{
				  C118set = 1;
				}
				if(DataBuff[2]==0x02)
				{
				  C118set = 0;
				}
//			printf("RXLen=%d\r\n",RxLine); 
//        for(int i=0;i<RxLine;i++)
//			{
//			  printf("UART DataBuff[%d] = 0x%x\r\n",i,DataBuff[i]);  
//			}  
                                  
        memset(DataBuff,0,sizeof(DataBuff));  //清空缓存数组
        RxLine=0;  //清空接收长度
    }
    
    RxBuff[0]=0;
    HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuff, 1); //每接收一个数据，就打开一次串口中断接收，否则只会接收一个数据就停止接收
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
