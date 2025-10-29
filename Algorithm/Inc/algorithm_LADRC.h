#ifndef _ADRC_H
#define _ADRC_H

#include "dev_system.h"

#define fp32 float

typedef __packed struct 	//b0=1,dt=0.001
{
	fp32 kp;
	fp32 kd;
	fp32 dt;
	fp32 b0;
	
} ADRC_TypeDef;

typedef __packed struct 
{
	fp32 z1,z2,z3;
	fp32 wo;
} ESO_TypeDef;

typedef __packed struct 
{
	fp32 r0;
	fp32 s;
	fp32 ds;
}TD_TypeDef;

void ADRC_Init( ADRC_TypeDef *adrc ,fp32 w,fp32 b,fp32 t);
//void ADRC_Init( ADRC_TypeDef *adrc );
void TD_Init( TD_TypeDef *td , fp32 r);
//void ESO_Init( ESO_TypeDef *eso );
void ESO_Init( ESO_TypeDef *eso, fp32 w);

void LTD_ADRC( TD_TypeDef *td , fp32 dt , fp32 u );
void TransProcess( TD_TypeDef *tp  , fp32 dt , fp32 u );
void LESO_ADRC( ESO_TypeDef *eso , fp32 h , fp32 b0 , fp32 u , fp32 y);

/*  ADRC计算：
	LADRC的ESO部分不使用非线性函数，计算更快
*/
fp32 LADRC_Calc( TD_TypeDef *td , ESO_TypeDef *eso , ADRC_TypeDef *adrc , fp32 u , fp32 y );

fp32 Fhan_ADRC( fp32 x1 , fp32 x2 , fp32 r0 , fp32 h0 );
fp32 LTD( fp32 e , fp32 x2 , fp32 r0 );

extern TD_TypeDef td1,td2;
extern ESO_TypeDef eso1,eso2;
extern ADRC_TypeDef adrc1,adrc2;
#endif

