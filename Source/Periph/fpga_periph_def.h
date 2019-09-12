/*
 * fpga_periph_def.h
 *
 *  Created on: 2019-5-28
 *      Author: DELL
 */

#ifndef FPGA_PERIPH_DEF_H_
#define FPGA_PERIPH_DEF_H_

#include "soc_C6748.h"

#define FPGA_BASE_ADDR SOC_EMIFA_CS2_ADDR

/*
 * FPGA版本信息和测试寄存器
 */
#define FPGA_VER_YEAR   (FPGA_BASE_ADDR + 0x0)
#define FPGA_VER_MOTH   (FPGA_BASE_ADDR + (0x1<<1))
#define FPGA_VER_DAY    (FPGA_BASE_ADDR + (0x2<<1))
#define FPGA_VER_LOGIC  (FPGA_BASE_ADDR + (0x3<<1))
#define FPGA_VER_DEBUG  (FPGA_BASE_ADDR + (0x4<<1))
#define FPGA_TEST_REG   (FPGA_BASE_ADDR + (0x5<<1))

/*
 *  AD7606寄存器：
 *    复位
 *    内外基准选择（暂未使用，硬件只支持内部基准）
 */
#define FPGA_AD7606_RESET   (FPGA_BASE_ADDR + (0x6<<1))
#define FPGA_AD7606_REF_SEL (FPGA_BASE_ADDR + (0x7<<1))

/*
 * 继电器寄存器
 */
#define FPGA_RELAY_CON      (FPGA_BASE_ADDR + (0x9<<1))
#define FPGA_RELAY_OEN      (FPGA_BASE_ADDR + (0xD<<1))

/*
 * TTL寄存器
 * TTL_IN/OUT: 5v,未隔离
 * LVTTL_IN: 9~36v输入, 隔离
 */
#define FPGA_TTL_IN         (FPGA_BASE_ADDR + (0x10<<1))
#define FPGA_TTL_OUT        (FPGA_BASE_ADDR + (0x11<<1))
#define FPGA_TTL_EN         (FPGA_BASE_ADDR + (0x12<<1))
#define FPGA_LVTTL_IN       (FPGA_BASE_ADDR + (0x13<<1))
#define FPGA_LVTTL_EN       (FPGA_BASE_ADDR + (0x14<<1))

/*
 * lan8710复位寄存器
 */
#define FPGA_LAN8710_RST    (FPGA_BASE_ADDR + (0x18<<1))

/*
 * CAN相关寄存器
 */
#define FPGA_CAN_INT      (FPGA_BASE_ADDR + (0x21<<1))
#define FPGA_CAN_RESET    (FPGA_BASE_ADDR + (0x23<<1))
#define FPGA_CAN_INT_ENB  (FPGA_BASE_ADDR + (0x26<<1))
#define FPGA_CAN_INT_MASK (FPGA_BASE_ADDR + (0x27<<1))
#define FPGA_CAN_DEV0     (FPGA_BASE_ADDR + 0x1000)
#define FPGA_CAN_DEV1     (FPGA_BASE_ADDR + 0x1200)
#define FPGA_CAN_DEV2     (FPGA_BASE_ADDR + 0x1400)
#define FPGA_CAN_DEV3     (FPGA_BASE_ADDR + 0x1600)
#define FPGA_CAN_DEV4     (FPGA_BASE_ADDR + 0x1800)
#define FPGA_CAN_DEV5     (FPGA_BASE_ADDR + 0x1A00)
#define FPGA_CAN_DEV6     (FPGA_BASE_ADDR + 0x1C00)
#define FPGA_CAN_DEV7     (FPGA_BASE_ADDR + 0x1E00)

/*
 *  RS232相关寄存器
 */
#define FPGA_RS232_INT      (FPGA_BASE_ADDR + (0x20<<1))
#define FPGA_RS232_RESET    (FPGA_BASE_ADDR + (0x22<<1))
#define FPGA_RS232_INT_ENB  (FPGA_BASE_ADDR + (0x24<<1))
#define FPGA_RS232_INT_MASK (FPGA_BASE_ADDR + (0x25<<1))
#define FPGA_RS232_DEV0     (FPGA_BASE_ADDR + 0x200)
#define FPGA_RS232_DEV1     (FPGA_BASE_ADDR + 0x220)
#define FPGA_RS232_DEV2     (FPGA_BASE_ADDR + 0x240)
#define FPGA_RS232_DEV3     (FPGA_BASE_ADDR + 0x260)
#define FPGA_RS232_DEV4     (FPGA_BASE_ADDR + 0x280)
#define FPGA_RS232_DEV5     (FPGA_BASE_ADDR + 0x2A0)

/*
 * PWM寄存器（SPEAK）
 */
#define FPGA_PWM_FTW7_0     (FPGA_BASE_ADDR + (0x30<<1))
#define FPGA_PWM_FTW15_8    (FPGA_BASE_ADDR + (0x31<<1))
#define FPGA_PWM_FTW23_16   (FPGA_BASE_ADDR + (0x32<<1))
#define FPGA_PWM_FTW31_24   (FPGA_BASE_ADDR + (0x33<<1))
#define FPGA_PWM_DUTY7_0    (FPGA_BASE_ADDR + (0x34<<1))
#define FPGA_PWM_DUTY15_8   (FPGA_BASE_ADDR + (0x35<<1))
#define FPGA_PWM_DUTY23_16  (FPGA_BASE_ADDR + (0x36<<1))
#define FPGA_PWM_DUTY31_24  (FPGA_BASE_ADDR + (0x37<<1))
#define FPGA_PWM_LOAD       (FPGA_BASE_ADDR + (0x38<<1))
#define FPGA_PWM_ENABLE     (FPGA_BASE_ADDR + (0x39<<1))

/*
 *  RS485相关寄存器
 */
#define FPGA_RS485_DE       (FPGA_BASE_ADDR + (0x8<<1))
#define FPGA_RS485_INT      (FPGA_BASE_ADDR + (0x40<<1))
#define FPGA_RS485_RESET    (FPGA_BASE_ADDR + (0x41<<1))
#define FPGA_RS485_INT_ENB  (FPGA_BASE_ADDR + (0x42<<1))
#define FPGA_RS485_INT_MASK (FPGA_BASE_ADDR + (0x43<<1))
#define FPGA_RS485_DEV0     (FPGA_BASE_ADDR + 0x2C0)
#define FPGA_RS485_DEV1     (FPGA_BASE_ADDR + 0x2E0)
#define FPGA_RS485_DEV2     (FPGA_BASE_ADDR + 0x300)
#define FPGA_RS485_DEV3     (FPGA_BASE_ADDR + 0x320)

/*
 *  GPS相关寄存器
 */
#define FPGA_GPS_INT      (FPGA_BASE_ADDR + (0x44<<1))
#define FPGA_GPS_RESET    (FPGA_BASE_ADDR + (0x45<<1))
#define FPGA_GPS_INT_ENB  (FPGA_BASE_ADDR + (0x46<<1))
#define FPGA_GPS_INT_MASK (FPGA_BASE_ADDR + (0x47<<1))
#define FPGA_GPS_DEV      (FPGA_BASE_ADDR + 0x340)

/*
 * AD(0~7通道)采样数据
 * 先读低位，再读高位
 */
#define FPGA_AD_CH0_DATAL    (FPGA_BASE_ADDR + (0x50<<1))
#define FPGA_AD_CH0_DATaH    (FPGA_BASE_ADDR + (0x51<<1))
#define FPGA_AD_CH1_DATAL    (FPGA_BASE_ADDR + (0x52<<1))
#define FPGA_AD_CH1_DATaH    (FPGA_BASE_ADDR + (0x53<<1))
#define FPGA_AD_CH2_DATAL    (FPGA_BASE_ADDR + (0x54<<1))
#define FPGA_AD_CH2_DATaH    (FPGA_BASE_ADDR + (0x55<<1))
#define FPGA_AD_CH3_DATAL    (FPGA_BASE_ADDR + (0x56<<1))
#define FPGA_AD_CH3_DATaH    (FPGA_BASE_ADDR + (0x57<<1))
#define FPGA_AD_CH4_DATAL    (FPGA_BASE_ADDR + (0x58<<1))
#define FPGA_AD_CH4_DATaH    (FPGA_BASE_ADDR + (0x59<<1))
#define FPGA_AD_CH5_DATAL    (FPGA_BASE_ADDR + (0x5A<<1))
#define FPGA_AD_CH5_DATaH    (FPGA_BASE_ADDR + (0x5B<<1))
#define FPGA_AD_CH6_DATAL    (FPGA_BASE_ADDR + (0x5C<<1))
#define FPGA_AD_CH6_DATaH    (FPGA_BASE_ADDR + (0x5D<<1))
#define FPGA_AD_CH7_DATAL    (FPGA_BASE_ADDR + (0x5E<<1))
#define FPGA_AD_CH7_DATaH    (FPGA_BASE_ADDR + (0x5F<<1))

#define FPGA_SIGNAL_TYPE     (FPGA_BASE_ADDR + (0x60<<1))
#define FPGA_SIGNAL0_FILTER  (FPGA_BASE_ADDR + (0x61<<1))
#define FPGA_SIGNAL1_FILTER  (FPGA_BASE_ADDR + (0x62<<1))
#define FPGA_SIGNAL2_FILTER  (FPGA_BASE_ADDR + (0x63<<1))
#define FPGA_SIGNAL3_FILTER  (FPGA_BASE_ADDR + (0x64<<1))
#define FPGA_SIGNAL_INT      (FPGA_BASE_ADDR + (0x65<<1))
#define FPGA_SIGNAL_INT_ENB  (FPGA_BASE_ADDR + (0x66<<1))
#endif /* FPGA_PERIPH_DEF_H_ */
