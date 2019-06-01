#include "emifa/emifa_app.h"
#include "fpga_ttl.h"
#include "soc_C6748.h"
#include "fpga_periph_def.h"

void TTLInit()
{
	/*
	 * 1.关闭继电器使能;
	 * 2.设置默认输出;
	 * 3.使能继电器;
	 * 4.使能LVTTL;
	 */
	EMIFAWriteWord(FPGA_RELAY_OEN, 0, 0x01);
	TTLWrite(0xFF);
	EMIFAWriteWord(FPGA_RELAY_OEN, 0, 0x00);
	EMIFAWriteWord(FPGA_LVTTL_EN, 0, 0x00);
}

void TTLWrite(uint8_t value)
{
	EMIFAWriteWord(FPGA_RELAY_CON, 0, value);
}

uint8_t TTLRead()
{
    uint8_t tmp;
    tmp = EMIFAReadWord(FPGA_LVTTL_IN, 0);
	return (~tmp);
}

void TTLWriteBit(uint8_t bitNum, uint8_t value)
{
	uint8_t regv;
	regv = EMIFAReadWord(FPGA_RELAY_CON, 0);
	if(value == 0)
		EMIFAWriteWord(FPGA_RELAY_CON, 0, regv & (~(0x01 << bitNum)));
	else
		EMIFAWriteWord(FPGA_RELAY_CON, 0, regv | (0x01 << bitNum));
}

uint8_t TTLReadBit(uint8_t bitNum)
{
	uint8_t regv;
	regv = EMIFAReadWord(FPGA_LVTTL_IN,0);

	if((regv & (0x01 << bitNum)) == 0)
		return 0;
	else
		return 1;
}
