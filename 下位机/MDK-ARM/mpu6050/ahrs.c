#include "ahrs.h"
#include <stdio.h>
volatile float exInt, eyInt, ezInt;  // ������
volatile float q0, q1, q2, q3,w1,w2,w3; // ȫ����Ԫ��
volatile float integralFBhand,handdiff;
volatile uint32_t lastUpdate, now; // �������ڼ��� ��λ us
float f;
volatile float Ya_offset=0,P_offset=0,R_offset=0;
float P[49]={0.0001,0,0,0,0,0,0,
	        0,0.0001,0,0,0,0,0,
			0,0,0.0001,0,0,0,0,
			0,0,0,0.0001,0,0,0,
			0,0,0,0,0.0002,0,0,
			0,0,0,0,0,0.0002,0,
			0,0,0,0,0,0,0.0002};

  float Q[49]={0.0001,0,0,0,0,0,0,
               0,0.0001,0,0,0,0,0,
			   0,0,0.0001,0,0,0,0,
			   0,0,0,0.0001,0,0,0,
			   0,0,0,0,0.0005,0,0,		 
			   0,0,0,0,0,0.0005,0,	 
			   0,0,0,0,0,0,0.0005} ;  
			    
float R[36]={0.0003,0,0,0,0,0,
               0,0.0003,0,0,0,0,
			   0,0,0.0003,0,0,0,
			   0,0,0,0.0002,0,0,
			   0,0,0,0,0.0002,0,
			   0,0,0,0,0,0.0002} ;	
			   		
float A[49],B[49],E[42],F1[36],X[49],Z[49],Ht[42],Ft[49],K[42],O[49],T[6],F[49],Y[7],P1[49],U1[36],U1t[36],D1[36],X1[36],X2[36];
float H[42]={
			   0,0,0,0,0,0,0,
			   0,0,0,0,0,0,0,
			   0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,													   												  
			   0,0,0,0,0,0,0,
							  };
float I[49]={1,0,0,0,0,0,0,
               0,1,0,0,0,0,0,
			   0,0,1,0,0,0,0,
			   0,0,0,1,0,0,0,
			   0,0,0,0,1,0,0,
			   0,0,0,0,0,1,0,
			   0,0,0,0,0,0,1
			   };

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void Initial_Timer3(void)
*��������:	  ��ʼ��Tim2  Tim3 ��������ʱ���������Բ���һ��32λ�Ķ�ʱ�����ṩϵͳus ���ļ�ʱ	
�����������
���������û��	
*******************************************************************************/
//TIM_HandleTypeDef htim2;
//				 
//void Initial_Timer3(void)
//{
//  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE); 
//	/* TIM2 configuration*/ 
//  /* Time Base configuration �������� ���ö�ʱ����ʱ����Ԫ*/
//  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
//  TIM_TimeBaseStructure.TIM_Period = 0xffff; //�Զ���װֵ         
//  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;       
//  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
//  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
//  
//  TIM_PrescalerConfig(TIM2, 0, TIM_PSCReloadMode_Update);
//  /* Disable the TIM2 Update event */
//  TIM_UpdateDisableConfig(TIM2, ENABLE);
//  /* ----------------------TIM2 Configuration as slave for the TIM3 ----------*/
//  /* Select the TIM2 Input Trigger: TIM3 TRGO used as Input Trigger for TIM2*/
//  TIM_SelectInputTrigger(TIM2, TIM_TS_ITR2);
//  /* Use the External Clock as TIM2 Slave Mode */
//  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_External1);
//  /* Enable the TIM2 Master Slave Mode */
//  TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
//  TIM_ARRPreloadConfig(TIM2, ENABLE);	
//	/* ��ʱ������:
//	1.���ö�ʱ��������ֵ 50000
//	2.����ʱ�ӷ�Ƶϵ����TIM_CKD_DIV1
//	3. ����Ԥ��Ƶ��  1Mhz/50000= 1hz 
//	4.��ʱ������ģʽ  ���ϼ���ģʽ
//	*/		 
//  	TIM_TimeBaseStructure.TIM_Period = 0xffff;     
//  	TIM_TimeBaseStructure.TIM_Prescaler = 72;	 //1M ��ʱ��  
//  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
//  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
//	//Ӧ�����õ�TIM3 
//  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
//	// ʹ��TIM3���ؼĴ���ARR
//  	TIM_ARRPreloadConfig(TIM3, ENABLE);	

//	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
//	TIM_UpdateRequestConfig(TIM3, TIM_UpdateSource_Regular);
//	/* ----------------------TIM3 Configuration as Master for the TIM2 -----------*/
//  	/* Use the TIM3 Update event  as TIM3 Trigger Output(TRGO) */
//  	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
//  	/* Enable the TIM3 Master Slave Mode */
//  	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

//  	//������ʱ��
//	  TIM_Cmd(TIM3, ENABLE); 
//  	TIM_Cmd(TIM2, ENABLE);                  
//}

// Fast inverse square-root
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   float invSqrt(float x)
*��������:	   ���ټ��� 1/Sqrt(x) 	
��������� Ҫ�����ֵ
��������� ���
*******************************************************************************/
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint32_t micros(void)
*��������:	  ��ȡϵͳ���е�ʱ�� �����ص�λΪus ��ʱ������	
�����������
�����������������ǰʱ�䣬���ϵ翪ʼ��ʱ  ��λ us
*******************************************************************************/
//uint32_t micros(void)
//{
// 	uint32_t temp=0 ;
// 	temp = TIM2->CNT; //����16λʱ��
// 	temp = temp<<16;
// 	temp += TIM3->CNT; //����16λʱ��
// 	return temp;
//}
__STATIC_INLINE uint32_t GXT_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}
static uint32_t getCurrentMicros(void)
{
  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  GXT_SYSTICK_IsActiveCounterFlag();
  uint32_t m = HAL_GetTick();
  const uint32_t tms = SysTick->LOAD + 1;
  __IO uint32_t u = tms - SysTick->VAL;
  if (GXT_SYSTICK_IsActiveCounterFlag()) {
    m = HAL_GetTick();
    u = tms - SysTick->VAL;
  }
  return (m * 1000 + (u * 1000) / tms);
}

uint32_t micros(void)
{
  return getCurrentMicros();
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void AHRS_init(void)
*��������:	  ��ʼ��IMU���	
			  ��ʼ������������
			  ��ʼ����Ԫ��
			  ����������
			  ����ϵͳʱ��
�����������
���������û��
*******************************************************************************/
void AHRS_init(void)
{
	//Initial_Timer3();
	//MPU6050_initialize();
	HMC5883L_SetUp();
	delay_ms(50);
	//MPU6050_initialize();
	HMC5883L_SetUp();
	//BMP180_init();

  	//������ƫ��
	w1=0;//0.095f;
	w2=0;//0.078f;
	w3=0;//-0.014f;
	
  	lastUpdate = micros();//����ʱ��
  	now = micros();

    q0=1.0;
    q1=0;
    q2=0;
    q3=0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_getValues(float * values)
*��������:	 ��ȡ���ٶ� ������ ������ �ĵ�ǰֵ  
��������� �������ŵ������׵�ַ
���������û��
*******************************************************************************/

void IMU_getValues(float * values) {  
	int16_t accgyroval[6];
	int i;
	char * x;
	//��ȡ���ٶȺ������ǵĵ�ǰADC
    MPU6050_getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);
    for(i = 0; i<6; i++) {
      if(i < 3) {
        values[i] = (float) accgyroval[i];
      }
      else {
        values[i] = ((float) accgyroval[i]) / 32.8f; //ת�ɶ�ÿ��
		//�����Ѿ������̸ĳ��� 1000��ÿ��  32.8 ��Ӧ 1��ÿ��
      }
    }

    HMC58X3_mgetValues(&values[6]);	//��ȡ�����Ƶ�ADCֵ
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_AHRSupdate
*��������:	 ����AHRS ������Ԫ�� 
��������� ��ǰ�Ĳ���ֵ��
���������û��
*******************************************************************************/



void AHRS_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float norm;
  float bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float g=9.79973;
  float Ha1,Ha2,Ha3,Ha4,Hb1,Hb2,Hb3,Hb4;
  float e1,e2,e3,e4,e5,e6;
  float halfT;
/*
	=============================================================================
	=============================================================================
	=============================================================================
	X = [q0 q1 q2 q3 w1 w2 w3]' ��Ԫ�� + ���ٶ�ƫ�� ��Ϊ���β����н��ٶ�ƫ�Ʋ���
	״̬���̣�
	X(k+1) = F*X(k)
	״̬ת�ƾ���
	F = |1_(4*4) + (dq/dt)*dt 		0_(4*3)	|
		  |0_(3*4)				       Eye_(3*3)	|(7*7)
	
	���У�	https://wenku.baidu.com/view/2f3d5bc0d5bbfd0a795673fa.html ����Ԫ��΢�ַ��̵��Ƶ���
			https://wenku.baidu.com/view/b2f5ac27a5e9856a561260ce.html ���߽���Ԫ���󵼡������Ե�����P302
				      |0  -Wx -Wy  -Wz||q0|
	dq/dt = 1/2*|Wx  0   Wz  -Wy||q1|  Wx Wy Wz �ǽ����� ���˴�Ϊ 1 ��΢�֡�
				      |Wy -Wz  0    Wx||q2|
				      |Wz  Wy -Wx   0 ||q3|
	
	X(k+1) = 	F*X(k) = |1_(4*4) + (dq/dt)*dt 		 		0_(4*3)	|		*  	|q0 q1 q2 q3 q4 w1 w2 w3|'(7*1)	
						         |0_(3*4)			        			Eye_(3*3)	|(7*7)
	=============================================================================
	=============================================================================
	=============================================================================
	Y = [ax ay az mx my mz]'    ��������ϵʵ����������ٶ� + ��������ϵʵ��Ĵų�ǿ�� 
	�۲ⷽ�̣�
	Y(k+1) = H*X(k)
	�۲�ת�ƾ���
			    |	-q2*g				q3*g		      -q0*g  			  -q1*g			        0	0  	0	|
			    | q1*g			  q0*g 			    q3*g  			  q2*g			        0	0  	0	|
	H = 2*	|	q0*g 	 		  -q1*g			    -q2*g 			  q3*g			        0	0  	0	|
			    |bx*q0-bz*q2	bx*q1+bz*q3  	-bx*q2-bz*q0  -bx*q3+bz*q1			0	0  	0	|
			    |-bx*q3+bz*q1	bx*q2+bz*q0  	bx*q1+bz*q3 	-bx*q0+bz*q2	  	0	0  	0	|
			    |bx*q2+bz*q0	bx*q3-bz*q1  	bx*q0-bz*q2  	bx*q1+bz*q3			  0	0  	0	|(6*7)
					
				      |-q2*g				q3*g			    -q0*g  			  -q1*g		       0	0  	0	|				|q0|
				      |	q1*g			  q0*g  			  q3*g  			  q2*g		       0	0  	0	|				|q1|
	Y(k+1) =  2*|	q0*g 	 		  -q1*g			    -q2*g 			  q3*g		       0	0  	0	|			* |q2|
				      |bx*q0-bz*q2	bx*q1+bz*q3  	-bx*q2-bz*q0  -bx*q3+bz*q1	 0	0  	0	|				|q3|
				      |-bx*q3+bz*q1	bx*q2+bz*q0  	bx*q1+bz*q3 	-bx*q0+bz*q2	 0	0  	0	|				|w1|
				      |bx*q2+bz*q0	bx*q3-bz*q1  	bx*q0-bz*q2  	bx*q1+bz*q3		 0	0  	0	|(6*7)	|w2|
																																															|w3|(7*1)
	=============================================================================
	=============================================================================
	=============================================================================
	*/


// �Ȱ���Щ�õõ���ֵ���
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;      
  bx = 0.5500;
  bz = 0.8351; 
  now = micros();  //��ȡʱ��
  if(now<lastUpdate){ //��ʱ��������ˡ�
  halfT =  ((float)(now + (0xffff- lastUpdate)) / 2000000.0f);
  }
  else	{
  halfT =  ((float)(now - lastUpdate) / 2000000.0f);
  }
  lastUpdate = now;	//����ʱ��
   norm = invSqrt(ax*ax + ay*ay + az*az);       
  ax = ax * norm*g;
  ay = ay * norm*g;
  az = az * norm*g;

  norm = invSqrt(mx*mx + my*my + mz*mz);          
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;
  
  gx=gx-w1;gy=gy-w2;gz=gz-w3;
/*****************************************************************************************************************************************
	��Сƽ ���������˲�ԭ����Ӧ�á�4.2.1�� P80ҳ��EKF�˲����̡�
	��5�����۲�ת�ƾ���ֵ
	H = dH(X)/dX
	=============================================================================
	=============================�������ٶ�======================================
	=============================================================================
	������Ԫ����ʾ����̬�Ǽ����������ٶ��ڹ�������ϵ�µ�Ԥ��ֵ
	q0q0 + q1q1 + q2q2 + q3q3 = 1
	Reb��ʾRotationEarthFromBody �ӹ�������ϵתΪ��������ϵ(��������ϵ)
	Rbe��ʾRotationBodyFromEarth �ӵ�������ϵתΪ��������ϵ(��������ϵ)
				    |q0q0+q1q1-q2q2-q3q3  	2(q1q2+q0q3)  				2(q1q3-q0q2)| 
	Rbe = 		|2(q1q2-q0q3)   		 q0q0-q1q1+q2q2-q3q3  		2(q2q3+q0q1)| �������ҳ˾��� b' = R(rigth)*a'��
				    |2(q1q3+q0q2)  			2(q2q3-q0q1)  		 q0q0-q1q1-q2q2+q3q3|	���Ե���P295ҳ���е�ΪReb���ҳ˾��󣨹�������ϵ����������ϵ��		
	
	[Gx Gy Gz]'=Rbe*[0 0 g]'
	
	��ˣ�������vx vy vzΪ�������ٶ��ڹ�������ϵ��ͶӰ��
	vx = 2*(q1q3 - q0q2)*g;
	vy = 2*(q0q1 + q2q3)*g;
	vz = (q0q0 - q1q1 - q2q2 + q3q3)*g;
	=============================================================================
	===============================�ų�ǿ��======================================
	=============================================================================
	ͬ��������Ԫ����ʾ����̬�Ǽ���ų�ǿ���ڹ�������ϵ�µ�Ԥ��ֵ
	Reb��ʾRotationEarthFromBody �ӹ�������ϵתΪ��������ϵ(��������ϵ)
	Rbe��ʾRotationBodyFromEarth �ӵ�������ϵתΪ��������ϵ(��������ϵ)
				    |q0q0+q1q1-q2q2-q3q3  	2(q1q2+q0q3)  					2(q1q3-q0q2)|
	Rbe = 		|2(q1q2-q0q3)   		q0q0-q1q1+q2q2-q3q3  			  2(q2q3+q0q1)|�������ҳ˾��� b' = R(rigth)*a'��
				    |2(q1q3+q0q2)  			2(q2q3-q0q1)  			 q0q0-q1q1-q2q2+q3q3| ���Ե���P295ҳ���е�ΪReb���ҳ˾��󣨹�������ϵ����������ϵ��			
	[Mx My Mz]'=Rbe*[bx 0 bz]'
	
	��ˣ�(���� wx wy wz Ϊ�ų�ǿ���ڹ�������ϵ��ͶӰ)
	wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
	wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
	=============================================================================
	=============================================================================
	=============================================================================
	���У�
			|@ax/@q0		@ax/@q1			@ax/@q2  		@ax/@q3			@ax/@w1			@ax/@w2  		@ax/@w3	|
			|@ay/@q0		@ay/@q1  		@ay/@q2  		@ay/@q3			@ay/@w1			@ay/@w2  		@ay/@w3	|
	 H =|@az/@q0 		@az/@q1			@az/@q2 		@az/@q3			@az/@w1			@az/@w2  		@az/@w3	|
			|@mx/@q0		@mx/@q1  		@mx/@q2  		@mx/@q3			@mx/@w1			@mx/@w2  		@mx/@w3	|
			|@my/@q0		@my/@q1  		@my/@q2  		@my/@q3			@my/@w1			@my/@w2  		@my/@w3	|
			|@mz/@q0		@mz/@q1  		@mz/@q2  		@mz/@q3			@mz/@w1			@mz/@w2  		@mz/@w3	|

			|	-q2*g			  q3*g			    -q0*g  			  -q1*g		      0	0  	0	|
			|	q1*g			  q0*g  			  q3*g  			  q2*g		      0	0  	0	|
	= 2*|	q0*g 	 		  -q1*g			    -q2*g 			  q3*g		      0	0  	0	|
			|bx*q0-bz*q2	bx*q1+bz*q3  	-bx*q2-bz*q0  -bx*q3+bz*q1	0	0  	0	|
			|-bx*q3+bz*q1	bx*q2+bz*q0  	bx*q1+bz*q3 	-bx*q0+bz*q2	0	0  	0	|
			|bx*q2+bz*q0	bx*q3-bz*q1  	bx*q0-bz*q2  	bx*q1+bz*q3		0	0  	0	|				
	=============================================================================
	=============================================================================
	=============================================================================
	*/


  Ha1=(-q2)*g; Ha2=q3*g;Ha3=-q0*g;Ha4=q1*g;	 
  Hb1=bx*q0-bz*q2;
  Hb2=bx*q1+bz*q3;//
  Hb3=-bx*q2-bz*q0;
  Hb4=-bx*q3+bz*q1;
  

  H[0]= Ha1;H[1]= Ha2;H[2]= Ha3;H[3]= Ha4;
  H[7]= Ha4;H[8]=-Ha3;H[9]= Ha2;H[10]=-Ha1;
  H[14]=-Ha3;H[15]=-Ha4;H[16]= Ha1;H[17]= Ha2;
  
  H[21]= Hb1;H[22]= Hb2;H[23]= Hb3;H[24]= Hb4;      
  H[28]= Hb4;H[29]=-Hb3;H[30]= Hb2;H[31]=-Hb1;
  H[35]=-Hb3;H[36]=-Hb4;H[37]= Hb1;H[38]= Hb2;
  
  /*****************************************************************************************************************************************
	��Сƽ ���������˲�ԭ����Ӧ�á�4.2.1�� P80ҳ��EKF�˲����̡�
	��2����״̬Ԥ��
	X(k|k-1)=FX(k-1)
	״̬���� һ�����������������Ԫ��  ��Ԫ����ʾ���Ǽ��ٶȵķ�������
					      |0  -Wx -Wy  -Wz|		|q0|
	dq/dt =	1/2	*	|Wx  0   Wz  -Wy|	*	|q1|
					      |Wy -Wz  0    Wx|		|q2|
					      |Wz  Wy -Wx   0 |		|q3|
	*/


  //״̬����
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
    // ��Ԫ����һ
  norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
//F��ֵ
/*****************************************************************************************************************************************
	��Сƽ ���������˲�ԭ����Ӧ�á�4.2.1�� P80ҳ��EKF�˲����̡�
	��4����״̬ת�ƾ���ֵ
	F = dF(X)/dX
	*/
F[0]=1;F[8]=1;F[16]=1;F[24]=1;F[32]=1;F[40]=1;F[48]=1;
F[1]=-gx*halfT;F[2]=-gz*halfT;F[3]=-gz*halfT;	F[4]=0; F[5]=0; F[6]=0;
F[7]=gx*halfT;F[9]=gz*halfT;F[10]=-gy*halfT;F[11]=0; F[12]=0; F[13]=0;
F[14]=gy*halfT;F[15]=-gz*halfT;F[17]=gx*halfT;F[18]=0; F[19]=0;F[20]=0;
F[21]=gz*halfT;F[22]=gy*halfT;F[23]=-gx*halfT;F[25]=0; F[26]=0; F[27]=0;
F[28]=0;F[29]=0;F[30]=0;F[31]=0;F[33]=0;F[34]=0;
F[35]=0;F[36]=0;F[37]=0;F[38]=0;F[39]=0;F[41]=0;
F[42]=0;F[43]=0;F[44]=0;F[45]=0;F[46]=0;F[47]=0;
 //�������˲�
 /****************************************************************************************************************************************
	��Сƽ ���������˲�ԭ����Ӧ�á�4.2.1�� P80ҳ��EKF�˲����̡�
	��6������Э�������
	P(k|k-1) = F(k)P(k-1|k-1)F' + Q
	*/

 MatrixMultiply(F,7,7,P,7,7,A );	//A=F*P
 MatrixTranspose(F,7,7,Ft);	  //Fת��  F'
 MatrixMultiply(A,7,7,Ft,7,7,B); // B=F*P*F'
 MatrixAdd( B,Q,P1,7,7 );
 /****************************************************************************************************************************************
	*��Сƽ ���������˲�ԭ����Ӧ�á�4.2.1�� P80ҳ��EKF�˲����̡�
	*��7�����󿨶����˲�����
	*K(k) = P(k|k-1)H(k)'[H(k)P(k|k-1)H' + R]
	*/
	//[H(k)P(k|k-1)H' + R]

 MatrixTranspose(H,6,7,Ht);	  //Fת��  F'
 MatrixMultiply(P1,7,7,Ht,7,6,E );   //E=P*H'
 MatrixMultiply(H,6,7,E,7,6,F1 ); //	 F1=H*P*H'	6*6
 MatrixAdd(F1,R,X,6,6 );           //X=F1+R	   6*6
 UD(X,6,U1,D1);	   //X��UD�ֽ�
 MatrixTranspose(U1,6,6,U1t);	 //U1��ת��
 MatrixMultiply(U1,6,6,D1,6,6,X1); //X1=U1*D1
 MatrixMultiply(X1,6,6,U1t,6,6,X2); //X2=U1*D1*U1t 
 MatrixInverse(X2,6,0);	 //X2�� 
 MatrixMultiply(E,7,6,X2,6,6,K ); //����K   7*6
/*****************************************************************************************************************************************
	��Сƽ ���������˲�ԭ����Ӧ�á�4.2.1�� P80ҳ��EKF�˲����̡�
	��3�����۲�Ԥ��
	Y(k|k-1)=HX(k-1)
	ԭ��μ���5������ת����
	*/
  vx = 2*(q1q3 - q0q2)*g;
  vy = 2*(q0q1 + q2q3)*g;
  vz = (q0q0 - q1q1 - q2q2 + q3q3)*g;
           
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
  /****************************************************************************************************************************************
	��Сƽ ���������˲�ԭ����Ӧ�á�4.2.1�� P80ҳ��EKF�˲����̡�
	��8��������״̬ X �����������
	X(k) = X(k|k-1)+K[(Y(k)-HX(k-1)]
	*/
	/*����۲�ֵ��Ԥ��ֵ֮���ƫ�� Ϊ�������˲��ṩ����*/

  e1=ax-vx;e2=ay-vy;e3=az-vz;
  e4=mx-wx;e5=my-wy;e6=mz-wz;
	/*�������� T ��ֵ*/
 T[0]=e1;T[1]=e2;T[2]=e3;T[3]=e4;T[4]=e5;T[5]=e6;
 MatrixMultiply(K,7,6,T,6,1,Y );   //Y=K*(Z-Y)	7*1
 //���ݱ仯������״̬����(��Ԫ���͹�������ϵ�µĴų�)
	//X = [q0 q1 q2 q3 w1 w2 w3]'
  q0= q0+Y[0];//������Ԫ��
	q1= q1+Y[1];//������Ԫ��
	q2= q2+Y[2];//������Ԫ��
	q3= q3+Y[3];//������Ԫ��
	w1= w1+Y[4];//���½��ٶ�ƫ��
	w2= w2+Y[5];//���½��ٶ�ƫ��
	w3= w3+Y[6];//���½��ٶ�ƫ��

  /*****************************************************************************************************************************************
	��Сƽ ���������˲�ԭ����Ӧ�á�4.2.1�� P80ҳ��EKF�˲����̡�
	��9��������Э����
	K(k) = [I-K(k)H(k)]P(k|k-1)
	*/
 MatrixMultiply(K,7,6,H,6,7,Z); //Z= K*H		7*7
 MatrixSub(I,Z,O,7,7 );	  //O=I-K*H
 
 MatrixMultiply(O,7,7,P1,7,7,P);
 
  /*��Ԫ����һ�� ��֤��һ��ʹ�õ���Ԫ���ǵ�λ��Ԫ��*/
  norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_getQ(float * q)
*��������:	 ������Ԫ�� ���ص�ǰ����Ԫ����ֵ
��������� ��Ҫ�����Ԫ���������׵�ַ
���������û��
*******************************************************************************/
float mygetqval[9];	//���ڴ�Ŵ�����ת�����������

void AHRS_getQ(float * q) {

	
  IMU_getValues(mygetqval);	 

  //�������ǵĲ���ֵת�ɻ���ÿ��
  //���ٶȺʹ����Ʊ��� ADCֵ������Ҫת��
	//x = mygetqval[3]*2*9/32768;
	//y = mygetqval[4]*2*9/32768;
	//z = mygetqval[5]*2*9/32768;
	//t = 10*sqrt(x*x+y*y+z*z);
	//sprintf(&s,"%0.2f", t);
	//printf("%s",&s);
 AHRS_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
   mygetqval[0], mygetqval[1], mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);
     
  q[0] = q0; //���ص�ǰֵ
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_getYawPitchRoll(float * angles)
*��������:	 ������Ԫ�� ���ص�ǰ��������̬����
��������� ��Ҫ�����̬�ǵ������׵�ַ
���������û��
*******************************************************************************/
void AHRS_getYawPitchRoll(float * angles) {
  float q[4]; //����Ԫ��
  
  AHRS_getQ(q); //����ȫ����Ԫ��
  angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1); // yaw 
  angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]); // pitch
  angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1); // roll
  //if(angles[0]<0)angles[0]+=360.0f;  //�� -+180��  ת��0-360��
	
	
//	AHRS_getQ(q); //����ȫ����Ԫ��
//  angles[0] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], 1-2 * q[2]*q[2] - 2 * q[3] * q[3]); // yaw 
//  angles[1] = asin(2 * q[1] * q[3] - 2 * q[0] * q[2]); // pitch
//  angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1); // roll
//  //if(angles[0]<0)angles[0]+=360.0f;  //�� -+180��  ת��0-360��
}
 

//------------------End of File----------------------------
