//****************************************Copyright (c)***********************************//
//原子哥在线教学平台：www.yuanzige.com
//技术支持：http://www.openedv.com/forum.php
//淘宝店铺：https://zhengdianyuanzi.tmall.com
//关注微信公众平台微信号："正点原子"，免费获取ZYNQ & FPGA & STM32 & LINUX资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2023-2033
//All rights reserved
//----------------------------------------------------------------------------------------
// File name:           emio_sccb_cfg
// Created by:          正点原子
// Created date:        2023年9月26日10:17:45
// Version:             V1.0
// Descriptions:        SCCB配置
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

#include"xgpiops.h"
#include"sleep.h"

#ifndef sccb_EMIO_CFG_
#define sccb_EMIO_CFG_

#define EMIO_SCL0_NUM 54
#define EMIO_SDA0_NUM 55

#define EMIO_SCL1_NUM 56
#define EMIO_SDA1_NUM 57

#define CAM0_CH0      0  //CAM0
#define CAM1_CH1      1  //CAM1

void emio_init(void);
void sccb_start(u8 cam_ch);
void sccb_stop(u8 cam_ch);
void sccb_ack(u8 cam_ch);
void sccb_send_byte(u8 cam,u8 txd);
u8  sccb_rece_byte(u8 cam);

#endif /* sccb_EMIO_CFG_ */
