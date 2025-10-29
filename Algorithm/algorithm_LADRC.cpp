#include "algorithm_LADRC.h"
#include "math.h"
#include "Robot_Task.h"
//extern MotorPosype	  YawPos;
float z3_last=0;
void ADRC_Init( ADRC_TypeDef *adrc ,fp32 w,fp32 b,fp32 t)		//adrc参数初始化
{
	fp32 wc  =  w;//10;			//控制器带宽，使用带宽法调参，Kp = wc^2，Kd = 2*wc
	adrc->kp =	wc*wc;  //对应公式的β1
	adrc->kd =	2*wc;   //对应公式的β2
	adrc->b0 = 	b;//1;			//补偿因子	根据控制量从小调整，控制eso的总扰动的大小
	adrc->dt =	0.01;		//时间间隔，对应公式的h
}

void TD_Init( TD_TypeDef *td , fp32 r)
{
	td->s  = 0;		//s是输入信号初始值，对应公式的x1
	td->ds = 0;		//ds = 0不用改,，对应公式的x2
	td->r0 = r;//5000;	//参数r0根据经验判断，越大输入变化越快，越小输入会越缓慢
}

void ESO_Init( ESO_TypeDef *eso,fp32 w)
{
	eso->z1  =	0;		
	eso->z2  =	0;
	eso->z3  =	0;
	eso->wo	 =	w;//15;		//wo越大，抗扰性越好，但是噪声会变大（比如会抖动）
}

/* 跟踪微分器 */
//	TransProcess( td , adrc->dt , u);	
//TransProcess( &td1 , 0.01 , yaw_set.ecd)
//TransProcess( &td2 , 0.01 , pitch_set.ecd);
void TransProcess( TD_TypeDef *tp  , fp32 dt , fp32 u ) 
{ 
	tp->s  += dt * tp->ds;
	tp->ds += dt * Fhan_ADRC( tp->s-u , tp->ds , tp->r0 , dt);
}

/* 线性跟踪微分器 */
void LTD_ADRC( TD_TypeDef *td , fp32 dt , fp32 u )
{
	td->s  += dt * td->ds;
	td->ds += dt * LTD( td->s-u , td->ds , td->r0 );
}

/*  */
  //LESO_ADRC( &eso2 , 0.01 , 1.5 , pitch_out , pitch_get.ecd );
  //LESO_ADRC( &eso1 , 0.01 , 1 , yaw_out , yaw_get.ecd );
	//LESO_ADRC( eso , adrc->dt , adrc->b0 , u0 , y);
void LESO_ADRC(ESO_TypeDef *eso , fp32 dt , fp32 b0 , fp32 u , fp32 y)
{
	fp32 e ;
	fp32 beta1 , beta2 , beta3;
	
	beta1 = 3 * eso->wo;//β01
	beta2 = 3 * eso->wo * eso->wo;//β02
	beta3 = eso->wo * eso->wo * eso->wo;//β03
	
	e = eso->z1 - y;//z1-x1
	eso->z1 += dt *( eso->z2 - beta1 * e );        //z1(k+1)=z1(k)＋h*[z2(k)—β01*e(k)]
	eso->z2 += dt *( eso->z3 - beta2 * e + b0 * u);//z2(k+1)=z2(k)＋h*[z3(k)—β02*fal(e,1/2,δ)+bu]
//	if(YawPos.PosOutNew-y>0.5||YawPos.PosOutNew-y<-0.5)
	eso->z3 -= dt * beta3 * e;                     //z3(k+1)=z3(k)-h*β03*fal(e,1/4,δ)
//   if(eso->z3>2000000)
//		 eso->z3=2000000;
//	 else if(eso->z3<-2000000)
//		 eso->z3=-200000;
	

}

/* 线性ADRC计算过程 */
fp32 LADRC_Calc( TD_TypeDef *td , ESO_TypeDef *eso , ADRC_TypeDef *adrc , fp32 u , fp32 y )
{
	fp32 LADRC_Output,u0;
	
	/* 第一步，安排输入信号过渡过程 */
	TransProcess( td , adrc->dt , u);			
	
	/* 第二步，PD控制和线性扩张状态观测器，得到输出信号的近似微分信号和总扰动 */
	/* 微分项如果抑制效果不好，可以把td->ds去掉，这样就只是一个标准的pd控制，z2是eso提取的输出的微分信息 */
	u0 = adrc->kp*( td->s - eso->z1 ) + adrc->kd*( td->ds - eso->z2 );   //PD控制//kp*(x1-z1)+kd*(x2-z2)//β1fal(e1,a1,δ)+β2fal(e2,a2,δ)
  //u0=Fhan_ADRC(  ,  , tp->r0 , dt);
	
	LESO_ADRC( eso , adrc->dt , adrc->b0 , u0 , y);
	
	/* 第三步，消除总扰动 */
	LADRC_Output = ( u0 + eso->z3 )/ adrc->b0;///  不是减z3吗？？
	//LADRC_Output = ( u0 - eso->z3 )/ adrc->b0;
	return LADRC_Output;
}

//Fhan_ADRC( tp->s-u , tp->ds , tp->r0 , dt);
fp32 Fhan_ADRC( fp32 x1 , fp32 x2 , fp32 r0 , fp32 dt ) //r0
{
	fp32 d , y , a0 , a1 , a2 , Sy , Sa , a , fhan;
	
	d  = r0 * dt * dt;
	a0 = x2 * dt;
	y  = x1 + a0;
	a1 = 1 / invSqrt( d * ( d + 8*fabs(y) ) );//计算浮点数x的绝对值,返回x的绝对值
	a2 = a0 + sign(y) * (a1 - d)/2;
	Sy = ( sign( y+d ) - sign( y-d ) )/2;
	a  = ( a0 + y - a2 ) * Sy + a2;
	Sa = ( sign( a+d ) - sign( a-d ) )/2;
	fhan = -r0 * ( a/d - sign(a) ) * Sa - r0*sign(a); 
	
	/* 
		x1 += dt * x2; 
		x2 += dt * fhan;
	*/
	
	return fhan;
}

//LTD( td->s-u , td->ds , td->r0 );
fp32 LTD( fp32 e , fp32 x2 , fp32 r0 )
{
	fp32 res;
	res =  -r0*r0*e - 2*r0*x2;
	
	return res;
}



