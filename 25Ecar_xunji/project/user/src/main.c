/*********************************************************************************************************************
* MSPM0G3507 Opensource Library 即（MSPM0G3507 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MSPM0G3507 开源库的一部分
* 
* MSPM0G3507 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          mian
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK 5.37
* 适用平台          MSPM0G3507
* 店铺链接          https://seekfree.taobao.com/
********************************************************************************************************************/

#include "zf_common_headfile.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
void a_handle (uint32 state, void *ptr);
void pit_handler (uint32 state, void *ptr);
int count = 0;
uint32 ips200pro_stack_size;
// 保存创建的页面ID，当需要对页面进行操作的时候则需要填写此ID
uint16 page_id[3];
int b = 0;
uint16 label_id[4];
char buffer[20];  // 准备一个字符数组
int main (void)
{
	
    clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>
    debug_init();					// 调试串口信息初始化
		// 此处编写用户代码 例如外设初始化代码等
		Car_Init();
//		IMU_AHRS_Init();
	
		key_init2(); 
	//IMUdata.flag = 1;
		pit_ms_init(PIT_TIM_G0, 10, pit_handler, NULL);
		pit_ms_init(PIT_TIM_G6, 5, a_handle, NULL);

    while(true)
    {
			
        // ★ 修改 OLED 显示：圈数信息
        sprintf(buffer, "Lap:%d/%d C:%d", lap_current, lap_target, corner_cnt);
        OLED_ShowString(1, 1, buffer);
        sprintf(buffer, "ek:%.1f", price.in_ek);
        OLED_ShowString(2, 1, buffer);
        sprintf(buffer, "%d",avg_dist);
        OLED_ShowString(3, 1, buffer);
        sprintf(buffer, "%d", cp_state);
        OLED_ShowString(4, 1, buffer);

    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 的周期中断处理函数 这个函数将在 PIT 对应的定时器中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
void pit_handler (uint32 state, void *ptr)
{
    static int a = 0;
    a++;
    get_state();
    Get_Speed();
    huidu_err();
    // ★ 每 10ms：按键扫描 + 处理事件
    key_scan();
    if (key_get_event() == KEY_EVENT_CLICK)
    {
        if (!lap_running)                       // 仅在未运行时允许修改圈数
        {
            lap_target++;
            if (lap_target > 5) lap_target = 1;
        }
    }
		
//								out_pwm(1100, 1700);//1000，2500


    // ★ 每 10ms：拐角计数（不受 a>=100 限制）
    lap_counter();
    if (a >= 300)
    {
        // ★ 首次进入 tracking 时，标记开始计数
        if (!lap_running && !lap_stop)
            lap_running = 1;
						tracking();

				
				
    }


}

void a_handle (uint32 state, void *ptr)
{
//		IMU_AHRS_Update();

}