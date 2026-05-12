#ifndef _IMAGE_ELEMENTS_HPP
#define _IMAGE_ELEMENTS_HPP

// 圆环识别状态（0=未识别, 1-5=各阶段）
extern uint8 left_ring;
extern uint8 right_ring;
void find_flagpoint(void);
void cross_fill(void);
void ring_recognize(void);     
 
#endif