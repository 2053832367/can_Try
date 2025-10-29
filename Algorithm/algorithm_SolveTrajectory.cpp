// 近点只考虑水平方向的空气阻力

//TODO 完整弹道模型
//TODO 适配英雄机器人弹道解算

#include "tasks.h"
#include "algorithm_SolveTrajectory.h"

struct SolveTrajectoryParams st;
struct fire_control_data  f1={.vter=0.4283152f};
struct tar_pos tar_position[4]; //最多只有四块装甲板
float t = 0.5f; // 飞行时间
uint8_t Flaggg=0;
// 决策后选择击打的装甲板编号
int idx;

float linkscope;
// 全局变量，有用可视化调试使用
float aim_array[3] = {};
bool debug = true;
float y_error;
float v_outpost;
float t_to_center;	
bool fire_control; 
	
void  process_angle(float visual_angle,float yaw_gyro,float *aim_angle) {
    if(visual_angle-yaw_gyro >= 180)
      *aim_angle=(visual_angle-yaw_gyro)-360;
		
		else if(visual_angle-yaw_gyro <=-180)
      *aim_angle= (visual_angle-yaw_gyro)+360;
		
		else *aim_angle=visual_angle-yaw_gyro;
   
}

/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
	
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float  z;
    //t为给定v与angle时的飞行时间
    t = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle)));
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);

    //printf("model %f %f\n", t, z);
    return z;
}


/*
@brief 完整弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
//TODO 完整弹道模型
float completeAirResistanceModel(float s, float v, float angle);



/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;
    // iteration
    for (i = 0; i < 20; i++)
    {
        angle_pitch = atan2(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        dz = 0.3f*(z - z_actual);
        z_temp = z_temp + dz;
       // printf("iteration num %d: angle_pitch %f, temp target z:%f, err of z:%f, s:%f\n",
        //    i + 1, angle_pitch * 180 / PI, z_temp, dz,s);
        if (fabsf(dz) < 0.00001f)
        {
            break;
        }
    }
    return angle_pitch;
}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
void autoSolveTrajectory(Visual_New_Receive_Data_t *New_Visusal,Visual_Caculate_Data*AIM_data,fp32* Gyro)
{
    float caculate_z,caculate_x,caculate_y,temp_yaw,temp_pitch;
	  //New_Visusal->New_Visual_Yaw += 1;
    // 线性预测
		if(isnan(t)){
			t = 0.5;
		}		
		
    float timeDelay = st.bias_time/1000.0 + t;
	
	  //AIM_data->Visual_Caculate_yaw=New_Visusal->yaw;
	
	  AIM_data->Visual_Caculate_yaw= New_Visusal->yaw;
    AIM_data->Visual_Caculate_yaw+= New_Visusal->v_yaw * timeDelay;

    //计算四块装甲板的位置
    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
	int use_1 = 1;
	int i = 0;
  idx = 0; // 被击打的装甲板编号，取决于选择逻辑，4号步兵选择距离最近的装甲板击打
    //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
    if (st.armor_num == ARMOR_NUM_BALANCE) {
        for (i = 0; i<2; i++) {
            float tmp_yaw = New_Visusal->yaw + i * PI;
            float r = st.r1;
            tar_position[i].x = New_Visusal->x - r*cos(tmp_yaw);
            tar_position[i].y = New_Visusal->y - r*sin(tmp_yaw);
            tar_position[i].z = New_Visusal->z;
            tar_position[i].yaw = tmp_yaw;
        }

        float yaw_diff_min = fabsf(New_Visusal->New_Visual_Yaw - tar_position[0].yaw);

        //因为是平衡步兵 只需判断两块装甲板即可
        float temp_yaw_diff = fabsf(New_Visusal->New_Visual_Yaw - tar_position[1].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }


    } else if (st.armor_num == ARMOR_NUM_OUTPOST) {  //前哨站
        for (i = 0; i<3; i++) {
            float tmp_yaw = New_Visusal->yaw + i * 2.0 * PI/3.0;  // 2/3PI
            float r =  (st.r1 + st.r2)/2;   //理论上r1=r2 这里取个平均值
            tar_position[i].x = New_Visusal->x - r*cos(tmp_yaw);
            tar_position[i].y =  New_Visusal->y - r*sin(tmp_yaw);
            tar_position[i].z = New_Visusal->z;
            tar_position[i].yaw = tmp_yaw;
        }

        //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用
				
    } else {

        for (i = 0; i<4; i++) {
//            float tmp_yaw = AIM_data->Visual_Caculate_yaw + i * PI/2.0;
						float tmp_yaw = New_Visusal->yaw + i * PI/2.0f;
            float r = use_1 ? st.r1 : st.r2;
            tar_position[i].x = New_Visusal->x - r*cos(tmp_yaw);
            tar_position[i].y = New_Visusal->y - r*sin(tmp_yaw);
            tar_position[i].z = use_1 ? New_Visusal->z : New_Visusal->z + New_Visusal->dz;
            tar_position[i].yaw = tmp_yaw;
            use_1 = !use_1;
        }

            //2种常见决策方案：
            //1.计算枪管到目标装甲板yaw最小的那个装甲板
            //2.计算距离最近的装甲板

            //计算距离最近的装甲板
        	float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
        	idx = 0;
        	for (i = 0; i<4; i++)
        	{
        		float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
        		if (temp_dis_diff < dis_diff_min)
        		{
        			dis_diff_min = temp_dis_diff;
        			idx = i;
        		}
        	}
        

				#if 0
        //计算枪管到目标装甲板yaw最小的那个装甲板
				// TODO： 用当前电机返回的Yaw减去目标的yaw, 4个相比得到偏角最小的yaw作为被击打的目标
        float yaw_diff_min = fabsf(Gyro->Yaw_angle - tar_position[0].yaw);
        for (i = 1; i<4; i++) {
            float temp_yaw_diff = fabsf( Gyro->Yaw_angle - tar_position[i].yaw);
            if (temp_yaw_diff < yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i; 
            }
        }
				#endif

    }

	
		// 采用 CV(constant velocity) 模型，运用积分思想（在极短时间内，变速也可以理解为匀速运动）
    AIM_data->Visual_Caculate_Aim_z = tar_position[idx].z + New_Visusal->vz * timeDelay;
    AIM_data->Visual_Caculate_Aim_x = tar_position[idx].x + New_Visusal->vx * timeDelay;
    AIM_data->Visual_Caculate_Aim_y = tar_position[idx].y + New_Visusal->vy * timeDelay;
//		if(debug){
//				aim_array[0] = tar_position[idx].x + New_Visusal->vx * timeDelay;
//				aim_array[1] = tar_position[idx].y +New_Visusal->vy * timeDelay;
//				aim_array[2] = tar_position[idx].z + New_Visusal->vz * timeDelay;
//		}
		caculate_z=AIM_data->Visual_Caculate_Aim_z;
		caculate_x=AIM_data->Visual_Caculate_Aim_x;
		caculate_y=AIM_data->Visual_Caculate_Aim_y;
		
		if(st.armor_num == ARMOR_NUM_NORMAL)
		{
		
		
//跟随		
    //这里符号给错了， s_bias:枪口前推的距离
		  temp_pitch=(pitchTrajectoryCompensation(sqrt(caculate_x * caculate_x +caculate_y * caculate_y),
      caculate_z - st.z_bias, st.current_v))*180/PI ;
      process_angle(float(atan2(caculate_y,caculate_x))*180/PI,*Gyro,&temp_yaw);

		//	temp_yaw=	float(atan2(caculate_y,caculate_x))*180/PI - *Gyro;

    New_Visusal->New_Visual_Yaw = temp_yaw;
    New_Visusal->New_Visual_Pitch = (temp_pitch)*PI/180;
		
		}
		
		else if(st.armor_num == ARMOR_NUM_OUTPOST)
		{
		
	 process_angle(float(atan2(Message.New_VisualR.y,Message.New_VisualR.x))*180/PI,*Gyro,&temp_yaw);
		
	 temp_pitch=(pitchTrajectoryCompensation(sqrt(Message.New_VisualR.x * Message.New_VisualR.x + Message.New_VisualR.y* Message.New_VisualR.y),
   Message.New_VisualR.z - st.z_bias, st.current_v))*180/PI ;
		
	 New_Visusal->New_Visual_Yaw = temp_yaw+0.5f;
   New_Visusal->New_Visual_Pitch = (temp_pitch+2)*PI/180 ;

	 
		y_error = Message.New_VisualR.y - caculate_y;
		v_outpost = 0.42f;
		t_to_center = (y_error / v_outpost);
			linkscope=t_to_center + t-0.1f;
		if((linkscope) > - 0.03f && (linkscope) < 0)
		{
			fire_control=true;		
		}
		else
		{
		 fire_control=false;
		}
		
		 

}


//		f1.angle_aim = New_Visusal->New_Visual_Yaw * PI / 180;
//		f1.angle_out_center = (asin((caculate_x * sin(f1.angle_aim)) / Message.New_VisualR.r1)) * 180 / PI;
//		f1.angle_next = f1.angle_aim * 180 / PI + 90;
//		f1.angle_t_run = (Message.New_VisualR.vy * timeDelay / Message.New_VisualR.r1) * 180 / PI;
		
		
		
}

 // 从坐标轴正向看向原点，逆时针方向为正

