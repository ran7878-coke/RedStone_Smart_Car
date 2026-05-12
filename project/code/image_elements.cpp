#include "zf_common_headfile.hpp"

void draw_3x3(int x, int y, int color)
{
    ips200.draw_point(x-1, y-1, color);
    ips200.draw_point(x  , y-1, color);
    ips200.draw_point(x+1, y-1, color);
    ips200.draw_point(x-1, y  , color);
    ips200.draw_point(x  , y  , color);
    ips200.draw_point(x+1, y  , color);
    ips200.draw_point(x-1, y+1, color);
    ips200.draw_point(x  , y+1, color);
    ips200.draw_point(x+1, y+1, color);
} 

/** 
* @brief 最小二乘法
* @param uint8 begin				输入起点
* @param uint8 end					输入终点
* @param uint8 *border				输入需要计算斜率的边界首地址
*  @see CTest		Slope_Calculate(start, end, border);//斜率
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
float Slope_Calculate(uint8 begin, uint8 end, uint8 *border)
{
	float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
	int16 i = 0;
	float result = 0;
	static float resultlast;

	for (i = begin; i < end; i++)
	{
		xsum += i;
		ysum += border[i];
		xysum += i * (border[i]);
		x2sum += i * i;

	}
	if ((end - begin)*x2sum - xsum * xsum) //判断除数是否为零
	{
		result = ((end - begin)*xysum - xsum * ysum) / ((end - begin)*x2sum - xsum * xsum);
		resultlast = result;
	}
	else
	{
		result = resultlast;
	}
	return result;
}

/** 
* @brief 计算斜率截距
* @param uint8 start				输入起点
* @param uint8 end					输入终点
* @param uint8 *border				输入需要计算斜率的边界
* @param float *slope_rate			输入斜率地址
* @param float *intercept			输入截距地址
*  @see CTest		calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
void calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept)
{
	uint16 i, num = 0;
	uint16 xsum = 0, ysum = 0;
	float y_average, x_average;

	num = 0;
	xsum = 0;
	ysum = 0;
	y_average = 0;
	x_average = 0;
	for (i = start; i < end; i++)
	{
		xsum += i;
		ysum += border[i];
		num++;
	}

	//计算各个平均数
	if (num)
	{
		x_average = (float)(xsum / num);
		y_average = (float)(ysum / num);

	}

	/*计算斜率*/
	*slope_rate = Slope_Calculate(start, end, border);//斜率
	*intercept = y_average - (*slope_rate)*x_average;//截距
}

/**
 * @brief 两点连线补线函数（基于最小二乘法拟合直线）
 * @param start   输入：起点坐标 [X,Y]
 * @param end     输入：终点坐标 [X,Y]
 * @param border  输入：需要补线的边界数组
 * @return 无
 * @note  复用 calculate_s_i 计算斜率截距，自动填充两点间所有边界点
 * @see   calculate_s_i
 */
void connect_points(uint16 start[2], uint16 end[2], uint8 *border)
{
    // 无效点
    if(start[1] == 0 || end[1] == 0) return;
    if(start[1] == end[1]) return;

    // 取出你的两个断裂点（已知有效点）
    float x1 = start[0], y1 = start[1];
    float x2 = end[0],   y2 = end[1];

    // 范围：从 上断点 到 下断点
    uint16 y_start = end[1];
    uint16 y_end   = start[1];

    // 两点式直线插值（百分百能连上）
    for(uint16 y = y_start; y <= y_end; y++)
    {
        // 直线公式：用两个断点直接算 x
        float x = x1 + (x2 - x1) * (y - y1) / (y2 - y1);

        // 限幅
        if(x < 0) x = 0;
        if(x >= IMAGE_W) x = IMAGE_W - 1;

        // 赋值边界，补线成功
        border[y] = (uint8)x;
    }
}

/**
 * @brief 通用查找坐标(x,y)在边界点数组中的索引
 * @param target_x: 目标点X坐标
 * @param target_y: 目标点Y坐标
 * @param points: 边界点数组指针 (points_l / points_r，类型uint16 [][2])
 * @param point_num: 数组有效点数量 (data_stastics_l / data_stastics_r)
 * @return 找到返回索引i，未找到返回0xFFFF
 */
uint16 find_point_index(uint16 target_x, uint16 target_y, uint16 (*points)[2], uint16 point_num)
{
    for (uint16 i = 0; i < point_num; i++)
    {
        uint16 curr_y = points[i][1];
        
        // 优化：points是Y从大到小排列，更小Y直接退出，速度翻倍
        if (curr_y < target_y)
            break;

        // 精准匹配 X+Y 唯一坐标
        if (points[i][0] == target_x && curr_y == target_y)
        {
            return i;
        }
    }
    return 0xFFFF; // 未找到匹配点
}

//变量
uint16 r_v_down[2] ={0, 0};  //右圆环下v点
uint16 l_v_down[2] ={0, 0};  //左圆环下v点
uint16 r_v_up[2] ={0, 0};    //右圆环上v点
uint16 l_v_up[2] ={0, 0};    //左圆环上v点
uint16 r_p[2] ={0, 0};       //右圆环切点P
uint16 l_p[2] ={0, 0};       //左圆环切点P

//屏幕四角
uint16 screen_l_up[2]     = {0, 0};        // 屏幕左上角 
uint16 screen_r_up[2]     = {160, 0};      // 屏幕右上角 
uint16 screen_l_down[2]   = {0, 120};      // 屏幕左下角 
uint16 screen_r_down[2]   = {160, 120};    // 屏幕右下角 

uint16 r_break_down[2] = {0, 0};  // 右下断裂点
uint16 l_break_down[2] = {0, 0};  // 左下断裂点
uint16 r_break_up[2]   = {0, 0};  // 右上断裂点
uint16 l_break_up[2]   = {0, 0};  // 左上断裂点

bool flag_r_v_down = 0;     //找到右圆环下v点标志
bool flag_l_v_down = 0;     //找到左圆环下v点标志
bool flag_r_v_up = 0;       //找到右圆环上v点标志
bool flag_l_v_up = 0;       //找到左圆环上v点标志
bool flag_r_p = 0;          //找到右圆环切点P标志
bool flag_l_p = 0;          //找到左圆环切点P标志

bool flag_r_conti = 1;      // 右边界连续标志
bool flag_l_conti = 1;      // 左边界连续标志

uint8 start_check = IMAGE_H - 20;  // 从底部往上20行开始

void find_flagpoint(void)
{
    uint16 i;
    // 初始化标志位
    flag_r_v_down = 0;     
    flag_l_v_down = 0;     
    flag_r_v_up = 0;       
    flag_l_v_up = 0;      
    flag_r_p = 0;          
    flag_l_p = 0;         

    flag_r_conti = 1;   // 右边界默认连续
    flag_l_conti = 1;   // 左边界默认连续

    r_v_down[0] = 0;
    r_v_down[1] = 0;
    l_v_down[0] = 0;
    l_v_down[1] = 0;
    r_v_up[0] = 0;
    r_v_up[1] = 0;
    l_v_up[0] = 0;
    l_v_up[1] = 0;
    r_p[0] = 0;
    r_p[1] = 0;
    l_p[0] = 0;
    l_p[1] = 0;
    r_break_down[0] = 0;
    r_break_down[1] = 0;
    l_break_down[0] = 0;
    l_break_down[1] = 0;
    r_break_up[0] = 0;
    r_break_up[1] = 0;
    l_break_up[0] = 0;
    l_break_up[1] = 0;

    // 边界连续性检查
    // 从下往上遍历（底部 → 顶部）
    for(i = start_check; i > 30; i--) 
    {
        // ===================== 右边界 上下断裂检测 =====================
        if( i+1 < IMAGE_H && (my_abs((int16)r_border[i] - (int16)r_border[i+1]) > 10 || r_border[i] == border_max) )
        {
            flag_r_conti = 0;
            // 下断裂点：只赋值第一次
            if(r_break_down[1] == 0)
            {
                r_break_down[0] = r_border[i+1]; 
                r_break_down[1] = i+1;
                //printf("右下：X=%d, Y=%d\r\n", r_break_down[0], r_break_down[1]);
                //draw_3x3(r_break_down[0], r_break_down[1],uesr_RED);
            }
            // 上断裂点
            else if(r_break_up[1] == 0)
            {
                r_break_up[0] = r_border[i]; 
                r_break_up[1] = i;
                //printf("右上：X=%d, Y=%d\r\n", r_break_up[0], r_break_up[1]);
                //draw_3x3(r_break_up[0], r_break_up[1],uesr_RED);
            }
        }

        // ===================== 左边界 上下断裂检测 =====================
        if( i+1 < IMAGE_H && (my_abs((int16)l_border[i] - (int16)l_border[i+1]) > 10 || l_border[i] == border_min) )
        {
            flag_l_conti = 0;
            // 下断裂点：只赋值第一次
            if(l_break_down[1] == 0)
            {
                l_break_down[0] = l_border[i+1]; 
                l_break_down[1] = i+1;
                //printf("左下：X=%d, Y=%d\r\n", l_break_down[0], l_break_down[1]);
                //draw_3x3(l_break_down[0], l_break_down[1],uesr_RED);
            }
            // 上断裂点
            else if(l_break_up[1] == 0)
            {
                l_break_up[0] = l_border[i]; 
                l_break_up[1] = i;
                //printf("左上：X=%d, Y=%d\r\n", l_break_up[0], l_break_up[1]);
                //draw_3x3(l_break_up[0], l_break_up[1],uesr_RED);
            }
        }

        if(r_border[i]>r_border[i+4] && r_border[i]>r_border[i-4] && flag_l_conti)
        {
            flag_r_p = 1;
            r_p[0]=r_border[i];
            r_p[1]=i;
            printf("rp\n");
        }

        if(l_border[i]>l_border[i+4]&&l_border[i]>l_border[i-4] && flag_r_conti)
        {
            flag_l_p = 1;
            l_p[0]=l_border[i];
            l_p[1]=i;
            printf("lp\n");
        }
    }

    // 在断裂处的邻域内寻找拐点
    if ((!flag_r_v_down) && flag_l_conti && r_break_down[1] != 0)  //右下v点未找到且左边线连续
    {
        uint16 start = r_break_down[1] + 10;
        uint16 end   = r_break_down[1] - 10;
        if(start > IMAGE_H-1) start = IMAGE_H-1;
        if(end < 31) end = 31;

        for (i = start_check; i > 30; i--) 
        {
                uint16 index = find_point_index(r_border[i], i, points_r, data_stastics_r);   //这里寻找r_border在points_r的位置
                //补充一下points数组的定义
                //points_l[l_data_statics][0] = center_point_l[0];//x
                //points_l[l_data_statics][1] = center_point_l[1];//y

                // 防越界保护
                if(index == 0xFFFF || index < 3 || index + 3 >= data_stastics_r)
                    continue;

                if (points_r[index-3][0]>points_r[index][0]
                &&points_r[index-3][1]>points_r[index][1]
                &&points_r[index+3][0]>points_r[index][0]
                &&points_r[index+3][1]>points_r[index][1])
                {
                    r_v_down[0] = points_r[index][0];
                    r_v_down[1] = points_r[index][1];
                    flag_r_v_down = 1;
                    printf("rvdown\n");
                    //ips200.draw_point(r_v_down[0], r_v_down[1], uesr_GREEN);
                    break;
                }
            
        }  
    }

    if ((!flag_r_v_up) && flag_l_conti && r_break_up[1] != 0)  //右上v点未找到且左边线连续
    {
        uint16 start = r_break_up[1] + 10;
        uint16 end   = r_break_up[1] - 10;
        if(start > IMAGE_H-1) start = IMAGE_H-1;
        if(end < 31) end = 31;

        for (i = start_check; i > 30; i--) 
        {
                uint16 index = find_point_index(r_border[i], i, points_r, data_stastics_r);   //这里寻找r_border在points_r的位置

                // 防越界保护
                if(index == 0xFFFF || index < 3 || index + 3 >= data_stastics_r)
                    continue;

                if (points_r[index-3][0]>points_r[index][0]
                &&points_r[index-3][1]>points_r[index][1]
                &&points_r[index+3][0]>points_r[index][0]
                &&points_r[index+3][1]>points_r[index][1])
                {
                    r_v_up[0] = points_r[index][0];
                    r_v_up[1] = points_r[index][1];
                    flag_r_v_up = 1;
                    printf("rvup\n");
                    //ips200.draw_point(r_v_up[0], r_v_up[1], uesr_GREEN);
                    break;
                }
            
        }  
    }

    if ((!flag_l_v_down) && flag_r_conti && l_break_down[1] != 0)  //左下v点未找到且右边线连续
    {
        uint16 start = l_break_down[1] + 10;
        uint16 end   = l_break_down[1] - 10;
        if(start > IMAGE_H-1) start = IMAGE_H-1;
        if(end < 31) end = 31;

        for (i = start; i > end; i--) 
        {
                uint16 index = find_point_index(l_border[i], i, points_l, data_stastics_l);   //这里寻找r_border在points_r的位置

                // 防越界保护
                if(index == 0xFFFF || index < 3 || index + 3 >= data_stastics_l)
                    continue;

                if (points_l[index-3][0]>points_l[index][0]
                &&points_l[index-3][1]<points_l[index][1]
                &&points_l[index+3][0]<points_l[index][0]
                &&points_l[index+3][1]<points_l[index][1])
                {
                    l_v_down[0] = points_l[index][0];
                    l_v_down[1] = points_l[index][1];
                    flag_l_v_down = 1;
                    printf("lvdown\n");
                    //ips200.draw_point(l_v_down[0], l_v_down[1], uesr_GREEN);
                    break;
                }
            
        }  
    }

    if ((!flag_l_v_up) && flag_r_conti && l_break_up[1] != 0)  //左上v点未找到且右边线连续
    {
        uint16 start = l_break_up[1] + 10;
        uint16 end   = l_break_up[1] - 10;
        if(start > IMAGE_H-1) start = IMAGE_H-1;
        if(end < 31) end = 31;

        for (i = start; i > end; i--) 
        {
                uint16 index = find_point_index(l_border[i], i, points_l, data_stastics_l);   //这里寻找r_border在points_r的位置

                // 防越界保护
                if(index == 0xFFFF || index < 3 || index + 3 >= data_stastics_l)
                    continue;

                if (points_l[index-3][0]<points_l[index][0]
                &&points_l[index-3][1]>points_l[index][1]
                &&points_l[index+3][0]>points_l[index][0]
                &&points_l[index+3][1]>points_l[index][1])
                {
                    l_v_up[0] = points_l[index][0];
                    l_v_up[1] = points_l[index][1];
                    flag_l_v_up = 1;
                    printf("lvup\n");
                    //ips200.draw_point(l_v_up[0], l_v_up[1], uesr_GREEN);
                    break;
                }
            
        }  
    }  
}

void cross_fill(void)
{
   if(flag_r_conti == 0 && flag_l_conti == 0)
    {
        // 右断裂点有效
        bool r_break_ok = (r_break_down[1] != 0 && l_break_up[1] != 0)
                    && (r_break_down[1] > 10 && r_break_down[1] < IMAGE_H - 10)
                    && (r_break_up[1] > 10 && r_break_up[1] < IMAGE_H - 10)
                    && (r_break_down[1] - r_break_up[1] > 5);  

        // 左断裂点有效
        bool l_break_ok = (l_break_down[1] != 0 && l_break_up[1] != 0)
                    && (l_break_down[1] > 10 && l_break_down[1] < IMAGE_H - 10)
                    && (l_break_up[1] > 10 && l_break_up[1] < IMAGE_H - 10)
                    && (l_break_down[1] - l_break_up[1] > 5);  // 足够高度 → 自动保证下>上
        // 左右断裂位置对齐
        bool y_pos_close = (my_abs((int16)r_break_down[1] - l_break_down[1]) < 20)
                        && (my_abs((int16)r_break_up[1] - l_break_up[1]) < 20);

        if(r_break_ok && l_break_ok && y_pos_close)
        {
            connect_points(r_break_down, r_break_up, r_border);
            connect_points(l_break_down, l_break_up, l_border);
            printf("补线\n");
        }
    }
}


/*
 * @brief 圆环识别状态机
 *        state 0: 待机，检测圆环入口特征
 *        state 1: 检测到特征，等待车到达入口
 *        state 2: 入圆确认（通过丢线深度+拐点消失判断）
 *        state 3: 圆内行驶，等待对侧出现出口特征
 *        state 4: 接近出口，等待两侧恢复直线
 *        state 5: 完全出圆，恢复直行
 * 调用：image_process() 中 get_left()/get_right() 之后，求 center_line 之前
 */
uint8 left_ring;
uint8 right_ring;

void ring_recognize(void)
{
    //  左圆环状态机
    switch (left_ring)
    {
        case 0:
            // 【初见环岛】：右边界连续 + 左边界断裂 + 找到左下V拐点 + 左P点
            if(flag_r_conti && !flag_l_conti
                && flag_l_v_down && flag_l_p
                && l_v_down[1] != 0 && l_p[1] != 0)
            {
                // 满足环岛入口唯一特征，切换状态
                left_ring = 1;
                connect_points(l_v_down, l_p, l_border);
                printf("状态1\n");
            }
            break;

        case 1:
            // 【初入环岛】：左拐点消失，断裂稳定，准备入环
            if(!flag_l_v_down && !flag_l_conti
                && l_p[1] != 0)
            {
                left_ring = 2;
                connect_points(screen_l_down, l_p, l_border);
                printf("状态2\n");
            }
            // 特征消失，退回待机
            else if(flag_l_conti && flag_r_conti)
            {
                left_ring = 0;
            }
            break;

        case 2:
            // 【第一次到环岛出口】：检测到左上出口V拐点
            if(flag_l_v_up && !flag_l_conti && flag_r_conti)
            {
                left_ring = 3;
                connect_points(l_p, l_v_up, l_border);
                printf("状态3\n");
            }
            break;

        case 3:
            // 【即将入环】：出口拐点稳定，断裂持续
            if(flag_l_v_up && !flag_l_conti)
            {
                left_ring = 4;
                connect_points(l_v_up, screen_r_down, r_border);
                printf("状态4\n");
            }
            break;

        case 4:
            // 【完全入环】：双侧边界恢复有效连续
            if(flag_r_conti && flag_l_conti
                && l_break_down[1] == 0 && r_break_down[1] == 0)
            {
                left_ring = 5;
                printf("状态5\n");
            }
            break;

        case 5:
            // 【即将出环】：对侧（右）出现断裂，出环特征
            if(!flag_r_conti && flag_l_conti)
            {
                left_ring = 6;
                connect_points(r_break_down, screen_l_up, r_border);
            }
            break;

        case 6:
            // 【完全出环】：双侧全恢复，清除状态
            if(flag_r_conti && flag_l_conti
                && !flag_l_v_up && !flag_l_v_down)
            {
                left_ring = 0;
            }
            break;

        default:
            left_ring = 0;
            break;
    }

    
    //  右圆环状态机
    switch (right_ring)
    {
        case 0:
            // 【初见环岛】：左边界连续 + 右边界断裂 + 找到右下V拐点 + 右P点
            if(flag_l_conti && !flag_r_conti
                && flag_r_v_down && flag_r_p
                && r_v_down[1] != 0 && r_p[1] != 0)
            {
                right_ring = 1;
                connect_points(r_v_down, r_p, r_border);
            }
            break;

        case 1:
            // 【初入环岛】：右拐点消失，断裂稳定
            if(!flag_r_v_down && !flag_r_conti)
            {
                right_ring = 2;
                connect_points(r_p, screen_r_down, r_border);
            }
            // 特征消失，退回待机
            else if(flag_l_conti && flag_r_conti)
            {
                right_ring = 0;
            }
            break;

        case 2:
            // 【第一次到环岛出口】：检测到右上出口V拐点
            if(flag_r_v_up && !flag_r_conti && flag_l_conti)
            {
                right_ring = 3;
                connect_points(r_v_up, r_p, r_border);
            }
            break;

        case 3:
            // 【即将入环】：出口拐点稳定，断裂持续
            if(flag_r_v_up && !flag_r_conti)
            {
                right_ring = 4;
                connect_points(r_v_up, screen_l_down, l_border);
            }
            break;

        case 4:
            // 【完全入环】：双侧边界恢复有效连续
            if(flag_r_conti && flag_l_conti
                && r_break_down[1] == 0 && l_break_down[1] == 0)
            {
                right_ring = 5;
            }
            break;

        case 5:
            // 【即将出环】：对侧（左）出现断裂，出环特征
            if(!flag_l_conti && flag_r_conti)
            {
                right_ring = 6;
                connect_points(l_break_down, screen_r_up, l_border);
            }
            break;

        case 6:
            // 【完全出环】：双侧全恢复，清除状态
            if(flag_r_conti && flag_l_conti
                && !flag_r_v_up && !flag_r_v_down)
            {
                right_ring = 0;
            }
            break;

        default:
            right_ring = 0;
            break;
    }
}
