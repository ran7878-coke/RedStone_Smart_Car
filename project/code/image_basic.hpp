#ifndef _IMAGE_BASIC_HPP
#define _IMAGE_BASIC_HPP


//数据类型声明（方便移植——移植的时候可以删掉，改成你自己的） 
typedef   signed          char int8;
typedef   signed short     int int16;
typedef   signed           int int32;
typedef unsigned          char uint8;
typedef unsigned short     int uint16;
typedef unsigned           int uint32;

//颜色定义  因为有先例，连颜色都改不来，我直接放这了
#define uesr_RED     0XF800    //红色
#define uesr_GREEN   0X07E0    //绿色
#define uesr_BLUE    0X001F    //蓝色


//宏定义
#define IMAGE_H	120//图像高度
#define IMAGE_W	160//图像宽度

#define white_pixel	255
#define black_pixel	0

#define bin_jump_num	1//跳过的点数
#define border_max	IMAGE_W-2 //边界最大值
#define border_min	1	//边界最小值	
extern uint8 original_image[IMAGE_H][IMAGE_W];
extern uint8 bin_image[IMAGE_H][IMAGE_W];//图像数组
extern uint8 center_line[IMAGE_H];//中线数组
extern uint8  l_border[IMAGE_H];
extern uint8  r_border[IMAGE_H];
extern uint16 data_stastics_l;
extern uint16 data_stastics_r;
extern uint16 points_l[IMAGE_H * 3][2];
extern uint16 points_r[IMAGE_H * 3][2];
extern uint16 dir_l[IMAGE_H * 3];
extern uint16 dir_r[IMAGE_H * 3];

void image_process(void); //直接在中断或循环里调用此程序就可以循环执行了
int16 limit_a_b(int16 x, int a, int b);
int my_abs(int value);

extern zf_device_ips200 ips200;
extern zf_device_uvc    uvc_dev;


#endif /*_IMAGE_H*/