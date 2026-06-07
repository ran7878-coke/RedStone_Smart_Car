# RedStone_Smart_Car
基于龙芯 LS2K0300 开发的视觉循迹智能小车，包含电机控制、编码器测速、陀螺仪稳向、串级 PD 循迹算法。

## 功能特性
- 视觉循迹（图像中线提取）
- 串级 PD 闭环控制
- 编码器速度 / 里程 / 加速度计算
- DRV8701E 双路电机驱动
- 差速转向控制
- 10ms 系统定时中断
- 安全退出 + 资源自动清理
- CMake 构建，跨平台编译

## 硬件平台
- 主控：龙芯 LS2K0300
- 电机驱动：DRV8701E
- 编码器：正交编码器
- 陀螺仪：IMU660B
- 摄像头：USB 摄像头

## 项目结构
RedStone_Smart_Car 

├── libraries/   # zf库     

├── project/     # 项目   

│   ├── code/        # 自有代码    

│   ├── model/       # 推理模型定义  

│   ├── out/         # 输出目录    

│   └── user/        # 用户相关配置/代码   

├── build.sh         # 构建脚本    

├── CMakeLists.txt   # CMake 配置文件   

├── cross.cmake      # 交叉编译配置    

└── main.cpp         # 主程序入口    

## 代码简析
code/文件夹中有如下文件
### image_basic.cpp
- 图像的大津法二值化
```uint8 otsuThreshold(...)```
- 八邻域寻找左右边界
```void search_l_r(...)```
- 图像处理并在显示屏上绘制边线
```void image_process(void)```
### image_elements.cpp
- 十字补线函数
```void cross_fill(...)```
- 环岛补线函数（未编写）  
**image_elements中的函数需要在void image_process(void)的指定位置调用**
### control.cpp
- 将PID进行串级
```void line_follow_pid_control(void)```
**放在main中的pit 10ms定时回调中**
### encoder.cpp
- 将编码器收集到的数据转换为里程，速度，加速度
### MahonyAHRS.cpp
- 将IMU数据融合后输出欧拉角
### motor.cpp
- 电机设置速度
### pid.cpp
- PID结构体和算法
## 编译运行
### 编译
- 在VScode终端中转到目录
- cd /home/raku/LS2K0300_Library/LS2K300_Library/RedStone_raku_project/project/user
- ./build.sh 编译 （注意build.sh文件中远程设备IP正确）  
### 运行
- 在连接核心板后在MobaXterm_Personal中
- ./project 运行
## 目前存在的问题
- 在图像处理中会卡死阻塞程序
## 开发者
rakurakudesu
GitHub：https://github.com/rakurakudesu
