#include "HMC5883L.h"
#include "main.h"
#include "i2c.h"
float HMC5883_lastx,HMC5883_lasty,HMC5883_lastz;

int16_t  HMC5883_FIFO[3][11]; //�������˲�
void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z);

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   unsigned char HMC5883_IS_newdata(void)
*��������:	   ��ȡDRDY ���ţ��ж��Ƿ������һ��ת��
 Low for 250 ��sec when data is placed in the data output registers. 
���������  ��
���������  ������ת���������1  ������� 0
*******************************************************************************/
unsigned char HMC5883_IS_newdata(void)
{
 	//if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==Bit_SET){
	  return 1;
	 //}
	 //else return 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void HMC58X3_FIFO_init(void)
*��������:	   ������ȡ100�����ݣ��Գ�ʼ��FIFO����
���������  ��
���������  ��
*******************************************************************************/
void HMC58X3_FIFO_init(void)
{
  int16_t temp[3];
  unsigned char i;
  for(i=0;i<50;i++){
  HMC58X3_getRaw(&temp[0],&temp[1],&temp[2]);
  delay_us(200);  //��ʱ�ٶ�ȡ����
  }
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
*��������:	   ����һ�����ݵ�FIFO����
���������  �������������Ӧ��ADCֵ
���������  ��
*******************************************************************************/
void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	int32_t sum=0;

	for(i=1;i<10;i++){
		HMC5883_FIFO[0][i-1]=HMC5883_FIFO[0][i];
		HMC5883_FIFO[1][i-1]=HMC5883_FIFO[1][i];
		HMC5883_FIFO[2][i-1]=HMC5883_FIFO[2][i];
	}

	HMC5883_FIFO[0][9]=x;
	HMC5883_FIFO[1][9]=y;
	HMC5883_FIFO[2][9]=z;

	sum=0;
	for(i=0;i<10;i++){	//ȡ�����ڵ�ֵ���������ȡƽ��
   		sum+=HMC5883_FIFO[0][i];
	}
	HMC5883_FIFO[0][10]=sum/10;	//��ƽ��ֵ����

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5883_FIFO[1][i];
	}
	HMC5883_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[2][10]=sum/10;
} //HMC58X3_newValues

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void HMC58X3_writeReg(unsigned char reg, unsigned char val)
*��������:	   дHMC5883L�ļĴ���
���������    reg  �Ĵ�����ַ
			  val   Ҫд���ֵ	
���������  ��
*******************************************************************************/
void HMC58X3_writeReg(unsigned char reg, unsigned char val) {
  //IICwriteByte(HMC58X3_ADDR,reg,val);                                             ////////////////////
	HAL_I2C_Master_Transmit(&hi2c1, HMC58X3_ADDR, &val, 1, 1000);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
*��������:	   дHMC5883L�ļĴ���
���������    reg  �Ĵ�����ַ
			  val   Ҫд���ֵ	
���������  ��
*******************************************************************************/
void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z) {
   unsigned char vbuff[6];
   vbuff[0]=vbuff[1]=vbuff[2]=vbuff[3]=vbuff[4]=vbuff[5]=0;
  //IICreadBytes(HMC58X3_ADDR,HMC58X3_R_XM,6,vbuff);                                   ////////////////
	//IICreadBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
	 HAL_I2C_Mem_Read(&hi2c1, 0x3C, 0x03, 1, vbuff,6, 1000);
   HMC58X3_newValues(((int16_t)vbuff[0] << 8) | vbuff[1],((int16_t)vbuff[4] << 8) | vbuff[5],((int16_t)vbuff[2] << 8) | vbuff[3]);
   *x = HMC5883_FIFO[0][10];
   *y = HMC5883_FIFO[1][10];
   *z = HMC5883_FIFO[2][10];
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z)
*��������:	   ��ȡ �����Ƶĵ�ǰADCֵ
���������    �������Ӧ�����ָ��	
���������  ��
*******************************************************************************/
void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z) {
  *x = HMC5883_FIFO[0][10];
  *y = HMC5883_FIFO[1][10]; 
  *z = HMC5883_FIFO[2][10]; 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_mgetValues(float *arry)
*��������:	   ��ȡ У����� ������ADCֵ
���������    �������ָ��	
���������  ��
*******************************************************************************/
void HMC58X3_mgetValues(float *arry) {
  int16_t xr,yr,zr;
  HMC58X3_getRaw(&xr, &yr, &zr);
	
  arry[0]= HMC5883_lastx=(float)(xr);
  arry[1]= HMC5883_lasty=(float)(yr);
  arry[2]= HMC5883_lastz=(float)(zr);
	//printf("qwe: ");
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_setGain(unsigned char gain)
*��������:	   ���� 5883L������
���������     Ŀ������ 0-7
���������  ��
*******************************************************************************/
void HMC58X3_setGain(unsigned char gain) { 
  // 0-7, 1 default
  if (gain > 7) return;
  HMC58X3_writeReg(HMC58X3_R_CONFB, gain << 5);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_setMode(unsigned char mode)
*��������:	   ���� 5883L�Ĺ���ģʽ
���������     ģʽ
���������  ��
*******************************************************************************/
void HMC58X3_setMode(unsigned char mode) {
  if (mode > 2) {
    return;
  }
  HMC58X3_writeReg(HMC58X3_R_MODE, mode);
  delay_us(100);
	//HAL_Delay(100);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_init(u8 setmode)
*��������:	   ���� 5883L�Ĺ���ģʽ
���������     ģʽ
���������  ��
*******************************************************************************/
void HMC58X3_init(u8 setmode) {

  if (setmode) {
    HMC58X3_setMode(0);
  }

  HMC58X3_writeReg(HMC58X3_R_CONFA, 0x70); // 8 samples averaged, 75Hz frequency, no artificial bias.
  HMC58X3_writeReg(HMC58X3_R_CONFB, 0xA0);
  HMC58X3_writeReg(HMC58X3_R_MODE, 0x00);

}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_setDOR(unsigned char DOR)
*��������:	   ���� 5883L�� �����������
���������     ����ֵ
0 -> 0.75Hz  |   1 -> 1.5Hz
2 -> 3Hz     |   3 -> 7.5Hz
4 -> 15Hz    |   5 -> 30Hz
6 -> 75Hz  
���������  ��
*******************************************************************************/
void HMC58X3_setDOR(unsigned char DOR) {
  if (DOR>6) return;
  HMC58X3_writeReg(HMC58X3_R_CONFA,DOR<<2);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_getID(char id[3])
*��������:	   ��ȡоƬ��ID
���������     	ID��ŵ�����
���������  ��
*******************************************************************************/
void HMC58X3_getID(char id[3]) 
{
	//HAL_I2C_Mem_Read(&hi2c1, HMC58X3_ADDR, HMC58X3_R_IDA, I2C_MEMADD_SIZE_8BIT,&id[0],8, 1000);

      id[0]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDA);  
      id[1]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDB);                                        ////////////////////
      id[2]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDC);
}   // getID().

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5883L_SetUp(void)
*��������:	   ��ʼ�� HMC5883L ʹ֮�������״̬
���������     	
���������  ��
*******************************************************************************/
void HMC5883L_SetUp(void)
{ 
	
//  HMC58X3_init(0); // Don't set mode yet, we'll do that later on.
//	
//  HMC58X3_setMode(0);
//	//
//  HMC58X3_setDOR(6);  //75hz ������
//  HMC58X3_FIFO_init();
//	//printf("a: ");
	  unsigned char cdata[3]={0x70,0xA0,0X00};
	//	  HMC58X3_writeReg(HMC58X3_R_MODE, mode);
	//	HMC58X3_writeReg(HMC58X3_R_CONFA,DOR<<2);
		HAL_I2C_Mem_Write(&hi2c1,HMC58X3_ADDR,HMC58X3_R_CONFA, I2C_MEMADD_SIZE_8BIT,&cdata[0], 1, 1000);            //
		HAL_I2C_Mem_Write(&hi2c1,HMC58X3_ADDR,HMC58X3_R_CONFB, I2C_MEMADD_SIZE_8BIT, &cdata[1], 1, 1000);       //�������棬      
    HAL_I2C_Mem_Write(&hi2c1,HMC58X3_ADDR,HMC58X3_R_MODE, I2C_MEMADD_SIZE_8BIT, &cdata[2], 1, 1000);     //���ò���ģʽ����������ģʽ
		HAL_I2C_Mem_Write(&hi2c1,HMC58X3_ADDR,HMC58X3_R_CONFB, I2C_MEMADD_SIZE_8BIT, 0x00, 1, 1000);       //�������棬    
    HMC58X3_FIFO_init();
}


//------------------End of File----------------------------
