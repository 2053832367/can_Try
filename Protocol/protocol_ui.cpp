#include "protocol_ui.h"
#include "string.h"
#include "stdio.h"
#include "tasks.h"

ext_client_custom_graphic_single_t  ext_client_custom_graphic_single;
ext_client_custom_graphic_delete_t  graphic_delete;
ext_client_custom_character_t       ext_client_custom_characte;
draw_data_struct_t		draw_data_struct;
char_data_struct_t    char_data_struct;
graph_data_struct_t   graph_data_struct;
Num_data_struct_t   	num_data_struct;
map_data_struct_t     map_data_struct;
MA_UIgraphic_struct_t MA_UIgraphic_struct;
MA_UInumber_struct_t  MA_UInumber_struct;

uint8_t judgeTX_buf[255u];

void Graph_Painter(char name[3], uint32_t Operate_tpye, uint32_t Graphic_tpye, uint16_t Sender_ID, uint16_t Receiver_ID,
	uint32_t Layer, uint32_t Color, uint32_t Width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y,
	uint32_t Radius, uint32_t start_angle, uint32_t end_angle)
{
	uint8_t src, UI_Seq;

	switch(Graphic_tpye)
	{
	case UI_Graph_Line:
	{
		graph_data_struct.graphic_data.start_x = start_x;
		graph_data_struct.graphic_data.start_y = start_y;
		graph_data_struct.graphic_data.end_x = end_x;
		graph_data_struct.graphic_data.end_y = end_y;
		UI_Seq = 0x08;
		break;
	}
	case UI_Graph_Rectangle:
	{
		graph_data_struct.graphic_data.start_x = start_x;
		graph_data_struct.graphic_data.start_y = start_y;
		graph_data_struct.graphic_data.end_x = end_x; 		//对顶角坐标					 
		graph_data_struct.graphic_data.end_y = end_y; 		//对顶角坐标	
		UI_Seq = 0x09;
		break;
	}
	case UI_Graph_Circle:
	{
		graph_data_struct.graphic_data.start_x = start_x;
		graph_data_struct.graphic_data.start_y = start_y;
		graph_data_struct.graphic_data.radius = Radius;  //半径
		UI_Seq = 0x0A;
		break;
	}
	case UI_Graph_Ellipse:
	{
		graph_data_struct.graphic_data.start_x = start_x;
		graph_data_struct.graphic_data.start_y = start_y;
		graph_data_struct.graphic_data.end_x = end_x; 	  //x半轴长度					 
		graph_data_struct.graphic_data.end_y = end_y; 		//y半轴长度
		UI_Seq = 0x0B;
		break;
	}
	case UI_Graph_Arc:
	{
		graph_data_struct.graphic_data.start_x = start_x;
		graph_data_struct.graphic_data.start_y = start_y;
		graph_data_struct.graphic_data.start_angle = start_angle;
		graph_data_struct.graphic_data.end_angle = end_angle;
		graph_data_struct.graphic_data.end_x = end_x;
		graph_data_struct.graphic_data.end_y = end_y;
		UI_Seq = 0x0C;
		break;
	}
	default: break;
	}

	graph_data_struct.UIMsg_head.SOF = 0xA5;
	graph_data_struct.UIMsg_head.DataLength = 21;
	graph_data_struct.UIMsg_head.Seq = UI_Seq;

	Append_CRC8_Check_Sum((uint8_t *)&graph_data_struct.UIMsg_head, sizeof(graph_data_struct.UIMsg_head));

	graph_data_struct.CmdID = 0x0301;

	graph_data_struct.UIdraw_header_id.data_cmd_id = 0x0101;
	graph_data_struct.UIdraw_header_id.sender_ID = Sender_ID;
	graph_data_struct.UIdraw_header_id.receiver_ID = Receiver_ID;

	for(src = 0;src < 3 && name[src] != '\0';src++)
		graph_data_struct.graphic_data.graphic_name[2 - src] = name[src];

	graph_data_struct.graphic_data.operate_tpye = Operate_tpye;
	graph_data_struct.graphic_data.graphic_tpye = Graphic_tpye;

	graph_data_struct.graphic_data.layer = Layer;
	graph_data_struct.graphic_data.color = Color;
	graph_data_struct.graphic_data.width = Width;

	Append_CRC16_Check_Sum((uint8_t *)&graph_data_struct, sizeof(graph_data_struct));
	memcpy(&judgeTX_buf[0], (uint8_t *)&graph_data_struct, sizeof(graph_data_struct));
	Usart_SendBuff((uint8_t *)&judgeTX_buf, sizeof(graph_data_struct));

	memcpy(judgeTX_buf, "\0", sizeof(judgeTX_buf));
	memset(&graph_data_struct, 0, sizeof(graph_data_struct));

}


void Num_Painter(char name[3], uint32_t Operate_tpye, uint32_t Graphic_tpye,
	uint16_t Sender_ID, uint16_t Receiver_ID, uint32_t Layer, uint32_t Color, uint32_t Width, uint32_t start_x, uint32_t start_y,
	uint32_t start_angle, uint32_t end_angle, int Int, float Float)
{
	uint8_t src;

	switch(Graphic_tpye)
	{
	case UI_Graph_Int:
	{
		num_data_struct.UIMsg_head.SOF = 0xA5;
		num_data_struct.UIMsg_head.DataLength = 21;
		num_data_struct.UIMsg_head.Seq = 0x0D;

		Append_CRC8_Check_Sum((uint8_t *)&num_data_struct.UIMsg_head, sizeof(num_data_struct.UIMsg_head));

		num_data_struct.CmdID = 0x0301;

		num_data_struct.UIdraw_header_id.data_cmd_id = 0x0101;
		num_data_struct.UIdraw_header_id.sender_ID = Sender_ID;
		num_data_struct.UIdraw_header_id.receiver_ID = Receiver_ID;

		for(src = 0;src < 3 && name[src] != '\0';src++)
			num_data_struct.graphic_name[2 - src] = name[src];

		num_data_struct.operate_tpye = Operate_tpye;
		num_data_struct.graphic_tpye = Graphic_tpye;

		num_data_struct.layer = Layer;
		num_data_struct.color = Color;
		num_data_struct.width = Width;

		num_data_struct.start_angle = start_angle;
		num_data_struct.end_angle = end_angle;
		num_data_struct.start_x = start_x;
		num_data_struct.start_y = start_y;
		num_data_struct.graph_num = Int;

		Append_CRC16_Check_Sum((uint8_t *)&num_data_struct, sizeof(num_data_struct));
		memcpy(&judgeTX_buf[0], (uint8_t *)&num_data_struct, sizeof(num_data_struct));
		Usart_SendBuff((uint8_t *)&judgeTX_buf, sizeof(num_data_struct));
		memcpy(judgeTX_buf, "\0", sizeof(judgeTX_buf));

		break;
	}
	case UI_Graph_Float:
	{
		num_data_struct.UIMsg_head.SOF = 0xA5;
		num_data_struct.UIMsg_head.DataLength = 21;
		num_data_struct.UIMsg_head.Seq = 0x0D;

		Append_CRC8_Check_Sum((uint8_t *)&num_data_struct.UIMsg_head, sizeof(num_data_struct.UIMsg_head));

		num_data_struct.CmdID = 0x0301;

		num_data_struct.UIdraw_header_id.data_cmd_id = 0x0101;
		num_data_struct.UIdraw_header_id.sender_ID = Sender_ID;
		num_data_struct.UIdraw_header_id.receiver_ID = Receiver_ID;

		for(src = 0;src < 3 && name[src] != '\0';src++)
			num_data_struct.graphic_name[2 - src] = name[src];

		num_data_struct.operate_tpye = Operate_tpye;
		num_data_struct.graphic_tpye = Graphic_tpye;

		num_data_struct.layer = Layer;
		num_data_struct.color = Color;
		num_data_struct.width = Width;

		num_data_struct.start_angle = start_angle;
		num_data_struct.end_angle = end_angle;
		num_data_struct.start_x = start_x;
		num_data_struct.start_y = start_y;
		num_data_struct.graph_num = Float * 1000;

		Append_CRC16_Check_Sum((uint8_t *)&num_data_struct, sizeof(num_data_struct));
		memcpy(&judgeTX_buf[0], (uint8_t *)&num_data_struct, sizeof(num_data_struct));
		Usart_SendBuff((uint8_t *)&judgeTX_buf, sizeof(num_data_struct));

		memcpy(judgeTX_buf, "\0", sizeof(judgeTX_buf));

		break;
	}
	default: break;
	}

	memset(&num_data_struct, 0, sizeof(num_data_struct));

}

void UI_Delete(uint8_t Del_Operate, uint8_t Del_Layer, uint16_t Sender_ID, uint16_t Receiver_ID)
{
	uint8_t del_buf[255u];

	graphic_delete.Del_head.SOF = 0xA5;
	graphic_delete.Del_head.DataLength = 8;
	graphic_delete.Del_head.Seq = 0x10;

	Append_CRC8_Check_Sum((uint8_t *)&graphic_delete.Del_head, sizeof(graphic_delete.Del_head));

	graphic_delete.CmdID = 0x0301;

	graphic_delete.UIdraw_header_id.data_cmd_id = 0x0100;
	graphic_delete.UIdraw_header_id.sender_ID = Sender_ID;
	graphic_delete.UIdraw_header_id.receiver_ID = Receiver_ID;

	graphic_delete.Delete_Operate = Del_Operate;
	graphic_delete.Layer = Del_Layer;

	Append_CRC16_Check_Sum((uint8_t *)&graphic_delete, sizeof(graphic_delete));
	memcpy(&del_buf[0], (uint8_t *)&graphic_delete, sizeof(graphic_delete));
	Usart_SendBuff((uint8_t *)&del_buf, sizeof(graphic_delete));

	memcpy(del_buf, "\0", sizeof(del_buf));
}

void UI_Map(uint16_t Target_Robot_ID, float Target_Position_x, float Target_Position_y, float Reserverd)
{
	uint8_t map_buf[255u];

	map_data_struct.Map_head.SOF = 0xA5;
	map_data_struct.Map_head.DataLength = 14;
	map_data_struct.Map_head.Seq = 0x11;

	Append_CRC8_Check_Sum((uint8_t *)&map_data_struct.Map_head, sizeof(map_data_struct.Map_head));

	map_data_struct.CmdID = 0x0305;

	map_data_struct.Target_Robot_ID = Target_Robot_ID;
	map_data_struct.Target_Position_x = Target_Position_x;
	map_data_struct.Target_Position_y = Target_Position_y;
	map_data_struct.Reserverd = Reserverd;

	Append_CRC16_Check_Sum((uint8_t *)&map_data_struct, sizeof(map_data_struct));
	memcpy(&map_buf[0], (uint8_t *)&map_data_struct, sizeof(map_data_struct));
	Usart_SendBuff((uint8_t *)&map_buf, sizeof(map_data_struct));

	memcpy(map_buf, "\0", sizeof(map_buf));
	memset(&map_data_struct, 0, sizeof(map_data_struct));
}

void UIGraph::Draw_Graphic(
	char name0[3] , uint32_t Operate_tpye0, uint32_t Graphic_tpye0, uint32_t Layer0, uint32_t Color0, uint32_t start_angle0, uint32_t end_angle0, 
	uint32_t Width0, uint32_t start_x0, uint32_t start_y0, uint32_t radius0, uint32_t end_x0, uint32_t end_y0, 
	
	char name1[3],uint32_t Operate_tpye1, uint32_t Graphic_tpye1, uint32_t Layer1, uint32_t Color1, uint32_t start_angle1, uint32_t end_angle1,
	uint32_t Width1, uint32_t start_x1, uint32_t start_y1, uint32_t radius1, uint32_t end_x1, uint32_t end_y1, 
	
	char name2[3],uint32_t Operate_tpye2, uint32_t Graphic_tpye2, uint32_t Layer2, uint32_t Color2, uint32_t start_angle2, uint32_t end_angle2,
	uint32_t Width2, uint32_t start_x2, uint32_t start_y2, uint32_t radius2, uint32_t end_x2, uint32_t end_y2, 
	
	char name3[3],uint32_t Operate_tpye3, uint32_t Graphic_tpye3, uint32_t Layer3, uint32_t Color3, uint32_t start_angle3, uint32_t end_angle3,
	uint32_t Width3, uint32_t start_x3, uint32_t start_y3, uint32_t radius3, uint32_t end_x3, uint32_t end_y3, 
	
	char name4[3],uint32_t Operate_tpye4, uint32_t Graphic_tpye4, uint32_t Layer4, uint32_t Color4, uint32_t start_angle4, uint32_t end_angle4,
	uint32_t Width4, uint32_t start_x4, uint32_t start_y4, uint32_t radius4, uint32_t end_x4, uint32_t end_y4, 
	
	char name5[3],uint32_t Operate_tpye5, uint32_t Graphic_tpye5, uint32_t Layer5, uint32_t Color5, uint32_t start_angle5, uint32_t end_angle5,
	uint32_t Width5, uint32_t start_x5, uint32_t start_y5, uint32_t radius5, uint32_t end_x5, uint32_t end_y5, 
	
	char name6[3],uint32_t Operate_tpye6, uint32_t Graphic_tpye6, uint32_t Layer6, uint32_t Color6, uint32_t start_angle6, uint32_t end_angle6,
	uint32_t Width6, uint32_t start_x6, uint32_t start_y6, uint32_t radius6, uint32_t end_x6, uint32_t end_y6)
{
	uint8_t src, UI_Seq = 0x08;
	
	MA_UIgraphic_struct.UIMsg_head.SOF = 0xA5;
	MA_UIgraphic_struct.UIMsg_head.DataLength = 111;
	MA_UIgraphic_struct.UIMsg_head.Seq = UI_Seq;

	Append_CRC8_Check_Sum((uint8_t *)&MA_UIgraphic_struct.UIMsg_head, sizeof(MA_UIgraphic_struct.UIMsg_head));

//-----------------------------------------------------------------------------------------

		for(src = 0;src < 3 && name0[src] != '\0';src++)
		MA_UIgraphic_struct.MA_UI_data[0].graphic_name[2 - src] = name0[src];

		MA_UIgraphic_struct.MA_UI_data[0].operate_tpye = Operate_tpye0;
		MA_UIgraphic_struct.MA_UI_data[0].graphic_tpye = Graphic_tpye0;

		MA_UIgraphic_struct.MA_UI_data[0].layer = Layer0;
		MA_UIgraphic_struct.MA_UI_data[0].color = Color0;
		
		MA_UIgraphic_struct.MA_UI_data[0].start_angle = start_angle0;
		MA_UIgraphic_struct.MA_UI_data[0].end_angle = end_angle0;
		MA_UIgraphic_struct.MA_UI_data[0].width = Width0;
		MA_UIgraphic_struct.MA_UI_data[0].start_x = start_x0;
		MA_UIgraphic_struct.MA_UI_data[0].start_y = start_y0;
		MA_UIgraphic_struct.MA_UI_data[0].radius = radius0;
		MA_UIgraphic_struct.MA_UI_data[0].end_x = end_x0;
		MA_UIgraphic_struct.MA_UI_data[0].end_y = end_y0;
	
//-----------------------------------------------------------------------------------------

		for(src = 0;src < 3 && name1[src] != '\0';src++)
		MA_UIgraphic_struct.MA_UI_data[1].graphic_name[2 - src] = name1[src];

		MA_UIgraphic_struct.MA_UI_data[1].operate_tpye = Operate_tpye1;
		MA_UIgraphic_struct.MA_UI_data[1].graphic_tpye = Graphic_tpye1;

		MA_UIgraphic_struct.MA_UI_data[1].layer = Layer1;
		MA_UIgraphic_struct.MA_UI_data[1].color = Color1;
		
		MA_UIgraphic_struct.MA_UI_data[1].start_angle = start_angle1;
		MA_UIgraphic_struct.MA_UI_data[1].end_angle = end_angle1;
		MA_UIgraphic_struct.MA_UI_data[1].width = Width1;
		MA_UIgraphic_struct.MA_UI_data[1].start_x = start_x1;
		MA_UIgraphic_struct.MA_UI_data[1].start_y = start_y1;
		MA_UIgraphic_struct.MA_UI_data[1].radius = radius1;
		MA_UIgraphic_struct.MA_UI_data[1].end_x = end_x1;
		MA_UIgraphic_struct.MA_UI_data[1].end_y = end_y1;

//-----------------------------------------------------------------------------------------
		
		for(src = 0;src < 3 && name2[src] != '\0';src++)
		MA_UIgraphic_struct.MA_UI_data[2].graphic_name[2 - src] = name2[src];

		MA_UIgraphic_struct.MA_UI_data[2].operate_tpye = Operate_tpye2;
		MA_UIgraphic_struct.MA_UI_data[2].graphic_tpye = Graphic_tpye2;

		MA_UIgraphic_struct.MA_UI_data[2].layer = Layer2;
		MA_UIgraphic_struct.MA_UI_data[2].color = Color2;
		
		MA_UIgraphic_struct.MA_UI_data[2].start_angle = start_angle2;
		MA_UIgraphic_struct.MA_UI_data[2].end_angle = end_angle2;
		MA_UIgraphic_struct.MA_UI_data[2].width = Width2;
		MA_UIgraphic_struct.MA_UI_data[2].start_x = start_x2;
		MA_UIgraphic_struct.MA_UI_data[2].start_y = start_y2;
		MA_UIgraphic_struct.MA_UI_data[2].radius = radius2;
		MA_UIgraphic_struct.MA_UI_data[2].end_x = end_x2;
		MA_UIgraphic_struct.MA_UI_data[2].end_y = end_y2;
		
//-----------------------------------------------------------------------------------------
	
		for(src = 0;src < 3 && name3[src] != '\0';src++)
		MA_UIgraphic_struct.MA_UI_data[3].graphic_name[2 - src] = name3[src];

		MA_UIgraphic_struct.MA_UI_data[3].operate_tpye = Operate_tpye3;
		MA_UIgraphic_struct.MA_UI_data[3].graphic_tpye = Graphic_tpye3;

		MA_UIgraphic_struct.MA_UI_data[3].layer = Layer3;
		MA_UIgraphic_struct.MA_UI_data[3].color = Color3;
		
		MA_UIgraphic_struct.MA_UI_data[3].start_angle = start_angle3;
		MA_UIgraphic_struct.MA_UI_data[3].end_angle = end_angle3;
		MA_UIgraphic_struct.MA_UI_data[3].width = Width3;
		MA_UIgraphic_struct.MA_UI_data[3].start_x = start_x3;
		MA_UIgraphic_struct.MA_UI_data[3].start_y = start_y3;
		MA_UIgraphic_struct.MA_UI_data[3].radius = radius3;
		MA_UIgraphic_struct.MA_UI_data[3].end_x = end_x3;
		MA_UIgraphic_struct.MA_UI_data[3].end_y = end_y3;

//-----------------------------------------------------------------------------------------

		for(src = 0;src < 3 && name4[src] != '\0';src++)
		MA_UIgraphic_struct.MA_UI_data[4].graphic_name[2 - src] = name4[src];

		MA_UIgraphic_struct.MA_UI_data[4].operate_tpye = Operate_tpye4;
		MA_UIgraphic_struct.MA_UI_data[4].graphic_tpye = Graphic_tpye4;

		MA_UIgraphic_struct.MA_UI_data[4].layer = Layer4;
		MA_UIgraphic_struct.MA_UI_data[4].color = Color4;
		
		MA_UIgraphic_struct.MA_UI_data[4].start_angle = start_angle4;
		MA_UIgraphic_struct.MA_UI_data[4].end_angle = end_angle4;
		MA_UIgraphic_struct.MA_UI_data[4].width = Width4;
		MA_UIgraphic_struct.MA_UI_data[4].start_x = start_x4;
		MA_UIgraphic_struct.MA_UI_data[4].start_y = start_y4;
		MA_UIgraphic_struct.MA_UI_data[4].radius = radius4;
		MA_UIgraphic_struct.MA_UI_data[4].end_x = end_x4;
		MA_UIgraphic_struct.MA_UI_data[4].end_y = end_y4;

	
//-----------------------------------------------------------------------------------------

		for(src = 0;src < 3 && name5[src] != '\0';src++)
		MA_UIgraphic_struct.MA_UI_data[5].graphic_name[2 - src] = name5[src];

		MA_UIgraphic_struct.MA_UI_data[5].operate_tpye = Operate_tpye5;
		MA_UIgraphic_struct.MA_UI_data[5].graphic_tpye = Graphic_tpye5;

		MA_UIgraphic_struct.MA_UI_data[5].layer = Layer5;
		MA_UIgraphic_struct.MA_UI_data[5].color = Color5;
		
		MA_UIgraphic_struct.MA_UI_data[5].start_angle = start_angle5;
		MA_UIgraphic_struct.MA_UI_data[5].end_angle = end_angle5;
		MA_UIgraphic_struct.MA_UI_data[5].width = Width5;
		MA_UIgraphic_struct.MA_UI_data[5].start_x = start_x5;
		MA_UIgraphic_struct.MA_UI_data[5].start_y = start_y5;
		MA_UIgraphic_struct.MA_UI_data[5].radius = radius5;
		MA_UIgraphic_struct.MA_UI_data[5].end_x = end_x5;
		MA_UIgraphic_struct.MA_UI_data[5].end_y = end_y5;
		
//-----------------------------------------------------------------------------------------
	
		for(src = 0;src < 3 && name6[src] != '\0';src++)
		MA_UIgraphic_struct.MA_UI_data[6].graphic_name[2 - src] = name6[src];

		MA_UIgraphic_struct.MA_UI_data[6].operate_tpye = Operate_tpye6;
		MA_UIgraphic_struct.MA_UI_data[6].graphic_tpye = Graphic_tpye6;

		MA_UIgraphic_struct.MA_UI_data[6].layer = Layer6;
		MA_UIgraphic_struct.MA_UI_data[6].color = Color6;
		
		MA_UIgraphic_struct.MA_UI_data[6].start_angle = start_angle6;
		MA_UIgraphic_struct.MA_UI_data[6].end_angle = end_angle6;
		MA_UIgraphic_struct.MA_UI_data[6].width = Width6;
		MA_UIgraphic_struct.MA_UI_data[6].start_x = start_x6;
		MA_UIgraphic_struct.MA_UI_data[6].start_y = start_y6;
		MA_UIgraphic_struct.MA_UI_data[6].radius = radius6;
		MA_UIgraphic_struct.MA_UI_data[6].end_x = end_x6;
		MA_UIgraphic_struct.MA_UI_data[6].end_y = end_y6;
	

	MA_UIgraphic_struct.CmdID = 0x0301;

	MA_UIgraphic_struct.UIdraw_header_id.data_cmd_id = 0x0104;
	MA_UIgraphic_struct.UIdraw_header_id.sender_ID = Sender_ID;
	MA_UIgraphic_struct.UIdraw_header_id.receiver_ID = Receiver_ID;

	Append_CRC16_Check_Sum((uint8_t *)&MA_UIgraphic_struct, sizeof(MA_UIgraphic_struct));
	memcpy(&judgeTX_buf[0], (uint8_t *)&MA_UIgraphic_struct, sizeof(MA_UIgraphic_struct));
	
//	Usart_SendBuff((uint8_t *)&judgeTX_buf, sizeof(MA_UIgraphic_struct));

	memcpy(judgeTX_buf, "\0", sizeof(judgeTX_buf));
	memset(&MA_UIgraphic_struct, 0, sizeof(MA_UIgraphic_struct));

}

void UIGraph::Draw_Number(
	char name0[3] , uint32_t Operate_tpye0, uint32_t Graphic_tpye0, uint32_t Layer0, uint32_t Color0, uint32_t start_angle0, uint32_t end_angle0, 
	uint32_t Width0, uint32_t start_x0, uint32_t start_y0, int Number0,  
	
	char name1[3],uint32_t Operate_tpye1, uint32_t Graphic_tpye1, uint32_t Layer1, uint32_t Color1, uint32_t start_angle1, uint32_t end_angle1,
	uint32_t Width1, uint32_t start_x1, uint32_t start_y1, int Number1, 
	
	char name2[3],uint32_t Operate_tpye2, uint32_t Graphic_tpye2, uint32_t Layer2, uint32_t Color2, uint32_t start_angle2, uint32_t end_angle2,
	uint32_t Width2, uint32_t start_x2, uint32_t start_y2, int Number2, 
	
	char name3[3],uint32_t Operate_tpye3, uint32_t Graphic_tpye3, uint32_t Layer3, uint32_t Color3, uint32_t start_angle3, uint32_t end_angle3,
	uint32_t Width3, uint32_t start_x3, uint32_t start_y3, int Number3, 
	
	char name4[3],uint32_t Operate_tpye4, uint32_t Graphic_tpye4, uint32_t Layer4, uint32_t Color4, uint32_t start_angle4, uint32_t end_angle4,
	uint32_t Width4, uint32_t start_x4, uint32_t start_y4, int Number4, 
	
	char name5[3],uint32_t Operate_tpye5, uint32_t Graphic_tpye5, uint32_t Layer5, uint32_t Color5, uint32_t start_angle5, uint32_t end_angle5,
	uint32_t Width5, uint32_t start_x5, uint32_t start_y5, int Number5, 
	
	char name6[3],uint32_t Operate_tpye6, uint32_t Graphic_tpye6, uint32_t Layer6, uint32_t Color6, uint32_t start_angle6, uint32_t end_angle6,
	uint32_t Width6, uint32_t start_x6, uint32_t start_y6, int Number6)
{
	uint8_t src, UI_Seq = 0x08;
	
	MA_UInumber_struct.UIMsg_head.SOF = 0xA5;
	MA_UInumber_struct.UIMsg_head.DataLength = 111;
	MA_UInumber_struct.UIMsg_head.Seq = UI_Seq;

	Append_CRC8_Check_Sum((uint8_t *)&MA_UInumber_struct.UIMsg_head, sizeof(MA_UInumber_struct.UIMsg_head));

//-----------------------------------------------------------------------------------------

		for(src = 0;src < 3 && name0[src] != '\0';src++)
		MA_UInumber_struct.MA_UI_data[0].graphic_name[2 - src] = name0[src];

		MA_UInumber_struct.MA_UI_data[0].operate_tpye = Operate_tpye0;
		MA_UInumber_struct.MA_UI_data[0].graphic_tpye = Graphic_tpye0;

		MA_UInumber_struct.MA_UI_data[0].layer = Layer0;
		MA_UInumber_struct.MA_UI_data[0].color = Color0;
		
		MA_UInumber_struct.MA_UI_data[0].start_angle = start_angle0;
		MA_UInumber_struct.MA_UI_data[0].end_angle = end_angle0;
		MA_UInumber_struct.MA_UI_data[0].width = Width0;
		MA_UInumber_struct.MA_UI_data[0].start_x = start_x0;
		MA_UInumber_struct.MA_UI_data[0].start_y = start_y0;
		MA_UInumber_struct.MA_UI_data[0].Number = Number0;
	
//-----------------------------------------------------------------------------------------

		for(src = 0;src < 3 && name1[src] != '\0';src++)
		MA_UInumber_struct.MA_UI_data[1].graphic_name[2 - src] = name1[src];

		MA_UInumber_struct.MA_UI_data[1].operate_tpye = Operate_tpye1;
		MA_UInumber_struct.MA_UI_data[1].graphic_tpye = Graphic_tpye1;

		MA_UInumber_struct.MA_UI_data[1].layer = Layer1;
		MA_UInumber_struct.MA_UI_data[1].color = Color1;
		
		MA_UInumber_struct.MA_UI_data[1].start_angle = start_angle1;
		MA_UInumber_struct.MA_UI_data[1].end_angle = end_angle1;
		MA_UInumber_struct.MA_UI_data[1].width = Width1;
		MA_UInumber_struct.MA_UI_data[1].start_x = start_x1;
		MA_UInumber_struct.MA_UI_data[1].start_y = start_y1;
		MA_UInumber_struct.MA_UI_data[1].Number = Number1;

//-----------------------------------------------------------------------------------------
		
		for(src = 0;src < 3 && name2[src] != '\0';src++)
		MA_UInumber_struct.MA_UI_data[2].graphic_name[2 - src] = name2[src];

		MA_UInumber_struct.MA_UI_data[2].operate_tpye = Operate_tpye2;
		MA_UInumber_struct.MA_UI_data[2].graphic_tpye = Graphic_tpye2;

		MA_UInumber_struct.MA_UI_data[2].layer = Layer2;
		MA_UInumber_struct.MA_UI_data[2].color = Color2;
		
		MA_UInumber_struct.MA_UI_data[2].start_angle = start_angle2;
		MA_UInumber_struct.MA_UI_data[2].end_angle = end_angle2;
		MA_UInumber_struct.MA_UI_data[2].width = Width2;
		MA_UInumber_struct.MA_UI_data[2].start_x = start_x2;
		MA_UInumber_struct.MA_UI_data[2].start_y = start_y2;
		MA_UInumber_struct.MA_UI_data[2].Number = Number2;
		
//-----------------------------------------------------------------------------------------
	
		for(src = 0;src < 3 && name3[src] != '\0';src++)
		MA_UInumber_struct.MA_UI_data[3].graphic_name[2 - src] = name3[src];

		MA_UInumber_struct.MA_UI_data[3].operate_tpye = Operate_tpye3;
		MA_UInumber_struct.MA_UI_data[3].graphic_tpye = Graphic_tpye3;

		MA_UInumber_struct.MA_UI_data[3].layer = Layer3;
		MA_UInumber_struct.MA_UI_data[3].color = Color3;
		
		MA_UInumber_struct.MA_UI_data[3].start_angle = start_angle3;
		MA_UInumber_struct.MA_UI_data[3].end_angle = end_angle3;
		MA_UInumber_struct.MA_UI_data[3].width = Width3;
		MA_UInumber_struct.MA_UI_data[3].start_x = start_x3;
		MA_UInumber_struct.MA_UI_data[3].start_y = start_y3;
		MA_UInumber_struct.MA_UI_data[3].Number = Number3;

//-----------------------------------------------------------------------------------------

		for(src = 0;src < 3 && name4[src] != '\0';src++)
		MA_UInumber_struct.MA_UI_data[4].graphic_name[2 - src] = name4[src];

		MA_UInumber_struct.MA_UI_data[4].operate_tpye = Operate_tpye4;
		MA_UInumber_struct.MA_UI_data[4].graphic_tpye = Graphic_tpye4;

		MA_UInumber_struct.MA_UI_data[4].layer = Layer4;
		MA_UInumber_struct.MA_UI_data[4].color = Color4;
		
		MA_UInumber_struct.MA_UI_data[4].start_angle = start_angle4;
		MA_UInumber_struct.MA_UI_data[4].end_angle = end_angle4;
		MA_UInumber_struct.MA_UI_data[4].width = Width4;
		MA_UInumber_struct.MA_UI_data[4].start_x = start_x4;
		MA_UInumber_struct.MA_UI_data[4].start_y = start_y4;
		MA_UInumber_struct.MA_UI_data[4].Number = Number4;
	
//-----------------------------------------------------------------------------------------

		for(src = 0;src < 3 && name5[src] != '\0';src++)
		MA_UInumber_struct.MA_UI_data[5].graphic_name[2 - src] = name5[src];

		MA_UInumber_struct.MA_UI_data[5].operate_tpye = Operate_tpye5;
		MA_UInumber_struct.MA_UI_data[5].graphic_tpye = Graphic_tpye5;

		MA_UInumber_struct.MA_UI_data[5].layer = Layer5;
		MA_UInumber_struct.MA_UI_data[5].color = Color5;
		
		MA_UInumber_struct.MA_UI_data[5].start_angle = start_angle5;
		MA_UInumber_struct.MA_UI_data[5].end_angle = end_angle5;
		MA_UInumber_struct.MA_UI_data[5].width = Width5;
		MA_UInumber_struct.MA_UI_data[5].start_x = start_x5;
		MA_UInumber_struct.MA_UI_data[5].start_y = start_y5;
		MA_UInumber_struct.MA_UI_data[5].Number = Number5;
		
//-----------------------------------------------------------------------------------------
	
		for(src = 0;src < 3 && name6[src] != '\0';src++)
		MA_UInumber_struct.MA_UI_data[6].graphic_name[2 - src] = name6[src];

		MA_UInumber_struct.MA_UI_data[6].operate_tpye = Operate_tpye6;
		MA_UInumber_struct.MA_UI_data[6].graphic_tpye = Graphic_tpye6;

		MA_UInumber_struct.MA_UI_data[6].layer = Layer6;
		MA_UInumber_struct.MA_UI_data[6].color = Color6;
		
		MA_UInumber_struct.MA_UI_data[6].start_angle = start_angle6;
		MA_UInumber_struct.MA_UI_data[6].end_angle = end_angle6;
		MA_UInumber_struct.MA_UI_data[6].width = Width6;
		MA_UInumber_struct.MA_UI_data[6].start_x = start_x6;
		MA_UInumber_struct.MA_UI_data[6].start_y = start_y6;
		MA_UInumber_struct.MA_UI_data[6].Number = Number6;
	

	MA_UInumber_struct.CmdID = 0x0301;

	MA_UInumber_struct.UIdraw_header_id.data_cmd_id = 0x0104;
	MA_UInumber_struct.UIdraw_header_id.sender_ID = Sender_ID;
	MA_UInumber_struct.UIdraw_header_id.receiver_ID = Receiver_ID;

	Append_CRC16_Check_Sum((uint8_t *)&MA_UInumber_struct,  sizeof(MA_UInumber_struct));
	memcpy(&judgeTX_buf[0], (uint8_t *)&MA_UInumber_struct, sizeof(MA_UInumber_struct));
	
//	Usart_SendBuff((uint8_t *)&judgeTX_buf, sizeof(MA_UInumber_struct));

	memcpy(judgeTX_buf, "\0", sizeof(judgeTX_buf));
	memset(&MA_UInumber_struct, 0, sizeof(MA_UInumber_struct));

}
