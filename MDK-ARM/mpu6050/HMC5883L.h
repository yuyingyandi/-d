#ifndef __HMC5883L_H
#define __HMC5883L_H

//#include "stm32f10x.h"
#include "IOI2C.h"
#include "delay.h"


#define HMC58X3_ADDR 0x3C // 7 bit address of the HMC58X3 used with the Wire library
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2
// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0x00
#define HMC58X3_R_CONFB 0x01
#define HMC58X3_R_MODE 0x02
#define HMC58X3_R_XM 0x03
#define HMC58X3_R_XL 4

#define HMC58X3_R_YM (7)  //!< Register address for YM.
#define HMC58X3_R_YL (8)  //!< Register address for YL.
#define HMC58X3_R_ZM (5)  //!< Register address for ZM.
#define HMC58X3_R_ZL (6)  //!< Register address for ZL.

#define HMC58X3_R_STATUS 9
#define HMC58X3_R_IDA 10
#define HMC58X3_R_IDB 11
#define HMC58X3_R_IDC 12

void HMC5883L_SetUp(void);	//��ʼ��
void HMC58X3_getID(char id[3]);	//��оƬID
void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z); //��ADC
void HMC58X3_mgetValues(float *arry); //IMU ר�õĶ�ȡ������ֵ
void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z);

#endif

//------------------End of File----------------------------
