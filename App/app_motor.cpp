#include "dev_can.h"
#include "app_motor.h"
#include "Message_Task.h"

CAN_Ctrl CAN_Cmd;

int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
    
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }
void CAN1_Send(CanRxMsg *FDCAN_RxID)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	ID_Data[CanData1].Data_Ptr = FDCAN_RxID;
	xQueueSendFromISR(CAN1_Rx_Queue, &ID_Data[CanData1], &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void CAN2_Send(CanRxMsg *FDCAN_RxID)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	ID_Data[CanData2].Data_Ptr = FDCAN_RxID;
	xQueueSendFromISR(CAN2_Rx_Queue, &ID_Data[CanData2], &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void CAN3_Send(CanRxMsg *FDCAN_RxID)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	ID_Data[CanData3].Data_Ptr = FDCAN_RxID;
	xQueueSendFromISR(CAN3_Rx_Queue, &ID_Data[CanData3], &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void CAN_ALL_Init(void)
{
	CAN1_Ctrl.attachInterrupt(CAN1_Send);
	CAN2_Ctrl.attachInterrupt(CAN2_Send);
	CAN3_Ctrl.attachInterrupt(CAN3_Send);
}

void CAN_Ctrl::SendData(CANctrl *CANx_Ctrl, uint32_t StdID, const void *buf, uint8_t len)
{
	CANx_Ctrl->ChangeID(StdID);
	CANx_Ctrl->SendData(buf, len);
}



void CAN_Ctrl::SendData(FDCAN_HandleTypeDef *CANx, uint32_t StdID, const void *buf, uint8_t len)
{
	if(CANx == &hfdcan1)
	{
		SendData(&CAN1_Ctrl, StdID, buf, len);
	}
	if(CANx == &hfdcan2)
	{
		SendData(&CAN2_Ctrl, StdID, buf, len);
	}
	if(CANx == &hfdcan3)
	{
		SendData(&CAN3_Ctrl, StdID, buf, len);
	}
}

//void CAN_Ctrl::SendData(FDCAN_HandleTypeDef *CANx,uint16_t id ,float P_des, float V_des)
//{
//	uint8_t *p_buf,*v_buf;

//	p_buf=(uint8_t*)&P_des;
//	v_buf=(uint8_t*)&V_des;

//	uint32_t StdID;
//	uint8_t Num;
//	uint8_t FDCAN_TxData[8];
//	
//	FDCAN_TxData[0] = *p_buf;
//	FDCAN_TxData[1] = *(p_buf+1);
//	FDCAN_TxData[2] = *(p_buf+2);
//	FDCAN_TxData[3] = *(p_buf+3);
//	FDCAN_TxData[4] = *(v_buf);
//	FDCAN_TxData[5] = *(v_buf+1);
//	FDCAN_TxData[6] = *(v_buf+2);
//	FDCAN_TxData[7] = *(v_buf+3);
//	
//	StdID=id;
//	SendData(CANx, StdID, FDCAN_TxData, 8);
//}
void CAN_Ctrl::SendData(FDCAN_HandleTypeDef *CANx,uint16_t id,uint8_t frame)
{
	uint32_t StdID;
	uint8_t Num;
	uint8_t FDCAN_TxData[8];
	
	FDCAN_TxData[0] = 0xFF;
	FDCAN_TxData[1] = 0xFF;
	FDCAN_TxData[2] = 0xFF;
	FDCAN_TxData[3] = 0xFF;
	FDCAN_TxData[4] = 0xFF;
	FDCAN_TxData[5] = 0xFF;
	FDCAN_TxData[6] = 0xFF;
	FDCAN_TxData[7] = frame;
	
	StdID=id;
	SendData(CANx, StdID, FDCAN_TxData, 8);
}
void CAN_Ctrl::SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1, int16_t Motor2, int16_t Motor3, int16_t Motor4)
{
	FDCAN_HandleTypeDef *CANx;
	uint32_t StdID;
	uint8_t Num;
	uint8_t FDCAN_TxData[8];
	FDCAN_TxData[0] = Motor1 >> 8;
	FDCAN_TxData[1] = Motor1;
	FDCAN_TxData[2] = Motor2 >> 8;
	FDCAN_TxData[3] = Motor2;
	FDCAN_TxData[4] = Motor3 >> 8;
	FDCAN_TxData[5] = Motor3;
	FDCAN_TxData[6] = Motor4 >> 8;
	FDCAN_TxData[7] = Motor4;

	Motor->GetData(StdID, Num);
	CANx = Motor->CANx;
	SendData(CANx, StdID, FDCAN_TxData, 8);
}

void CAN_Ctrl::SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1, int16_t Motor2, int16_t Motor3)
{
	FDCAN_HandleTypeDef *CANx;
	uint32_t StdID;
	uint8_t Num;
	uint8_t FDCAN_TxData[8];
	FDCAN_TxData[0] = Motor1 >> 8;
	FDCAN_TxData[1] = Motor1;
	FDCAN_TxData[2] = Motor2 >> 8;
	FDCAN_TxData[3] = Motor2;
	FDCAN_TxData[4] = Motor3 >> 8;
	FDCAN_TxData[5] = Motor3;
	FDCAN_TxData[6] = 0;
	FDCAN_TxData[7] = 0;

	Motor->GetData(StdID, Num);
	CANx = Motor->CANx;
	SendData(CANx, StdID, FDCAN_TxData, 6);
}

void CAN_Ctrl::SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1, int16_t Motor2)
{
	FDCAN_HandleTypeDef *CANx;
	uint32_t StdID;
	uint8_t Num;
	uint8_t FDCAN_TxData[8];
	FDCAN_TxData[0] = Motor1 >> 8;
	FDCAN_TxData[1] = Motor1;
	FDCAN_TxData[2] = Motor2 >> 8;
	FDCAN_TxData[3] = Motor2;
	FDCAN_TxData[4] = 0;
	FDCAN_TxData[5] = 0;
	FDCAN_TxData[6] = 0;
	FDCAN_TxData[7] = 0;

	Motor->GetData(StdID, Num);
	CANx = Motor->CANx;
	SendData(CANx, StdID, FDCAN_TxData, 4);
}

void CAN_Ctrl::SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1)
{
	FDCAN_HandleTypeDef *CANx;
	uint32_t StdID;
	uint8_t Num;
	uint8_t FDCAN_TxData[8];
	FDCAN_TxData[0] = Motor1 >> 8;
	FDCAN_TxData[1] = Motor1;
	FDCAN_TxData[2] = 0;
	FDCAN_TxData[3] = 0;
	FDCAN_TxData[4] = 0;
	FDCAN_TxData[5] = 0;
	FDCAN_TxData[6] = 0;
	FDCAN_TxData[7] = 0;

	Motor->GetData(StdID, Num);
	CANx = Motor->CANx;
	SendData(CANx, StdID, FDCAN_TxData, 2);
}


void CAN_Ctrl::DM_MIT_SendData(DM_Motor_CAN_Ctrl *DM_Motor,float P_des, float V_des,float KP_des,float KD_des,float T_ff)
{
	FDCAN_HandleTypeDef *CANx;

	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(P_des, DM_Motor->LIMIT.P_MIN, DM_Motor->LIMIT.P_MAX, 16);
	vel_tmp = float_to_uint(V_des, DM_Motor->LIMIT.V_MIN, DM_Motor->LIMIT.V_MAX, 12);
	kp_tmp  = float_to_uint(KP_des,DM_Motor->LIMIT.KP_MIN,DM_Motor->LIMIT.KP_MAX,12);
	kd_tmp  = float_to_uint(KD_des,DM_Motor->LIMIT.KD_MIN,DM_Motor->LIMIT.KD_MAX,12);
	tor_tmp = float_to_uint(T_ff,  DM_Motor->LIMIT.T_MIN, DM_Motor->LIMIT.T_MAX, 12);

	uint32_t StdID;
	uint8_t  FDCAN_TxData[8];

	FDCAN_TxData[0] = pos_tmp >> 8;
	FDCAN_TxData[1] = pos_tmp;
	FDCAN_TxData[2] = vel_tmp >> 4;
	FDCAN_TxData[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	FDCAN_TxData[4] = kp_tmp;
	FDCAN_TxData[5] = (kd_tmp>>4);
	FDCAN_TxData[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	FDCAN_TxData[7] = tor_tmp;

	DM_Motor->ID_GetData(StdID);
	CANx=DM_Motor->CANx;

	SendData(CANx,StdID, FDCAN_TxData, 8);

}
void CAN_Ctrl::DM_Motor_UnEnable(DM_Motor_CAN_Ctrl *DM_Motor)
{
	
	FDCAN_HandleTypeDef *CANx;
	uint32_t StdID;

	DM_Motor->ID_GetData(StdID);
	CANx=DM_Motor->CANx;

	CAN_Cmd.SendData(CANx,StdID,LOCK_ID);
	
}
void CAN_Ctrl::DM_Motor_Enable(DM_Motor_CAN_Ctrl *DM_Motor)
{
	FDCAN_HandleTypeDef *CANx;
	uint32_t StdID;
	
	DM_Motor->ID_GetData(StdID);
	CANx=DM_Motor->CANx;
	
	SendData(CANx,StdID,START_ID);
}
void CAN_Ctrl::DM_Motor_clear_error(DM_Motor_CAN_Ctrl *DM_Motor)
{
	
	FDCAN_HandleTypeDef *CANx;
	uint32_t StdID;
	
	DM_Motor->ID_GetData(StdID);
	CANx=DM_Motor->CANx;
	
	SendData(CANx,StdID,ERR_ID);
	
}
void CAN_Ctrl::DM_Motor_SetZeroT(DM_Motor_CAN_Ctrl *DM_Motor)
{
	FDCAN_HandleTypeDef *CANx;
	uint32_t StdID;
	
	DM_Motor->ID_GetData(StdID);
	CANx=DM_Motor->CANx;
	CAN_Cmd.SendData(CANx,StdID,SETPOINT_ID);
}


//void CAN_Ctrl::SendData(FDCAN_HandleTypeDef *CANx,uint16_t id,uint8_t frame)
//{
//	uint32_t StdID;
//	uint8_t Num;
//	uint8_t FDCAN_TxData[8];
//	
//	FDCAN_TxData[0] = 0xFF;
//	FDCAN_TxData[1] = 0xFF;
//	FDCAN_TxData[2] = 0xFF;
//	FDCAN_TxData[3] = 0xFF;
//	FDCAN_TxData[4] = 0xFF;
//	FDCAN_TxData[5] = 0xFF;
//	FDCAN_TxData[6] = 0xFF;
//	FDCAN_TxData[7] = frame;
//	
//	StdID=id;
//	SendData(CANx, StdID, FDCAN_TxData, 8);
//}




