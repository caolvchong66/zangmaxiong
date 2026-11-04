//****************************************Copyright (c)***********************************//
//原子哥在线教学平台：www.yuanzige.com
//技术支持：http://www.openedv.com/forum.php
//淘宝店铺：https://zhengdianyuanzi.tmall.com
//关注微信公众平台微信号："正点原子"，免费获取ZYNQ & FPGA & STM32 & LINUX资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2023-2033
//All rights reserved
//----------------------------------------------------------------------------------------
// File name:           ov5640_init
// Created by:          正点原子
// Created date:        2023年9月26日10:17:45
// Version:             V1.0
// Descriptions:        OV5640配置初始化
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

#include "xil_types.h"
#include"sleep.h"

#ifndef OV5640_INIT_H_
#define OV5640_INIT_H_


u8 ov5640_init(u8 cam_ch,u16 cmos_h_pixel, u16 cmos_v_pixel, u16 total_h_pixel, u16 total_v_pixel );


#endif /* OV5640_INIT_H_ */
