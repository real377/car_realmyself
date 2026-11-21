#include "zf_common_headfile.h"
uint8_t list[7] = {0};
uint8_t temp[7] = {0}; 
//float weight[7] ={-40,-35,-10,0,10,30,40};
float weight[7] ={-50,-25,-10,0,10,25,50};
float err = 0;
int cross_flag = 0,T_mode = 0,T_flag = 0,traget_v = 80,a = 0;
int mode[2] = {0};

int corner_state = 0;  // ★新增：拐角状态 0=正常, 1=左拐, -1=右拐, 2=全丢

uint8_t imu_correction_enable = 0;   // 默认guanbi

void huidu_init()
{
		gpio_init(B6,  GPI, 0, GPI_PULL_DOWN);	//L1
		gpio_init(B8,  GPI, 0, GPI_PULL_DOWN);	//L2
		gpio_init(B9,  GPI, 0, GPI_PULL_DOWN);	//L3
	
		gpio_init(B7,  GPI, 0, GPI_PULL_DOWN);	//MID
	
		gpio_init(B26, GPI, 0, GPI_PULL_DOWN);	//R1	
		gpio_init(A12, GPI, 0, GPI_PULL_DOWN);	//R2	
		gpio_init(A13,  GPI, 0, GPI_PULL_DOWN);	//R3
		
}

void get_state()
{
	
	list[0] = !gpio_get_level(B6);  
	list[1] = !gpio_get_level(B8);  	
	list[2] = !gpio_get_level(B9);  
	list[3] = !gpio_get_level(B7);  
	list[4] = !gpio_get_level(B26); 
	list[5] = !gpio_get_level(A12); 
	list[6] = !gpio_get_level(A13); 


}
void huidu_err()
{
    int i = 0, j = 0;
	
    // ===== ★ 拐角检测 =====
    uint8_t left_all_off  = (list[0] == 0 && list[1] == 0 && list[2] == 0);
    uint8_t right_all_off = (list[4] == 0 && list[5] == 0 && list[6] == 0);
    
    // 更新拐角状态（供 tracking 减速使用）
    if (left_all_off && !right_all_off)
        corner_state = 1;   // 左拐角：左侧3个全丢线 → 应左转
    else if (right_all_off && !left_all_off)
        corner_state = -1;  // 右拐角：右侧3个全丢线 → 应右转
    else if (left_all_off && right_all_off)
        corner_state = 2;   // 两侧都丢（十字或完全离线）
    else
        corner_state = 0;   // 正常巡线

		
	
	
    for (i = 0; i < 7; i++)  // 修复：i < 7 而不是 i < 8
    {
        if (list[i] == 1)
        {
            temp[j] = i;
            j++;
        }
    }

    // ===== 加权平均计算偏差 =====
    for (i = 0; i < 7; i++)
    {
        if (list[i] == 1)
        {
            temp[j] = i;
            j++;
        }
    }
    if (j > 0)
    {
        float sum_weight = 0;
        for (i = 0; i < j; i++)
        {
            sum_weight += weight[temp[i]];
        }
        err = sum_weight / j;
    }
    else
    {
        err = 0;
    }
    
    // ===== ★ 拐角平滑限幅 =====
    // 关键：当一侧全丢线时，限制 err 最大值，避免瞬间大力转向
    if (corner_state == 1)  // 左拐角
    {
        // 原始 err 为正（右侧传感器权重），-err 后为负 = 左转
        // 原来是 ±100 的量级，现在限幅到 ±35，转弯力度降为 1/3
        if (err > 35)  err = 5;
        if (err < -35) err = -5;
    }
    else if (corner_state == -1)  // 右拐角
    {
        if (err < -35) err = -5;
        if (err > 35)  err = 5;
    }
    else if (corner_state == 2)  // 全丢线，保持上一个方向小幅度
    {
        // err 保持为 0 或上一次的小值，不突变
        err = 0;
    }
    // corner_state == 0：正常巡线，不加限制
    
    err = -err;
    price.in_ek = err;

		

}

