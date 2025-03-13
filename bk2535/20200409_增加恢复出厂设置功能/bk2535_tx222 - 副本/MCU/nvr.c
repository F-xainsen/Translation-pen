#include "includes.h"


void __mcu_write_flash(uint8 space, uint16 addr, uint8 wr_data)
{
	if(space == NVR_SPACE) {
		nvr_wp0 = 0xa5;  // nvr_wp0 = 0x00; write protect
		nvr_wp1 = 0xc3;
	}
//    MTP_CTL = 0x01;
	MTP_KEY = 0xa5;
	MTP_KEY = 0x49;
	MTP_ADR = addr & 0xff;
	MTP_ADR_H = (addr >> 8) & 0xff;
	MTP_DAT = wr_data;
	MTP_CTL = 0x81 | (space << 1);
	while(MTP_CTL & 0x80);
	//nvr_wp0 = 0x00;  // nvr_wp0 = 0x00;
}

uint8 __mcu_read_flash(uint8 space, uint16 addr)
{
//	if(space == NVR_SPACE) {
//		nvr_wp0 = 0xa5;
//		nvr_wp1 = 0xc3;
//	}
//  MTP_CTL = 0x01;
  	MTP_KEY = 0xa5;							//在上电的第一次读取， 有可能数据是0， 因此要多读一次。
	MTP_KEY = 0x49;
	MTP_ADR = addr & 0xff;
	MTP_ADR_H = (addr >> 8) & 0xff;
	MTP_CTL = 0x41 | (space << 1);
	while(MTP_CTL & 0x40);

  	MTP_KEY = 0xa5;
	MTP_KEY = 0x49;
	MTP_ADR = addr & 0xff;
	MTP_ADR_H = (addr >> 8) & 0xff;
	MTP_CTL = 0x41 | (space << 1);
	while(MTP_CTL & 0x40);
	return MTP_DAT;
}


void __mcu_erase_flash(uint8 space, uint16 addr)
{
	if(space == NVR_SPACE) {
		nvr_wp0 = 0xa5;
		nvr_wp1 = 0xc3;
	}
    MTP_CTL = 0x01;
	MTP_KEY = 0xa5;
	MTP_KEY = 0x49;
	MTP_ADR = addr & 0xff;
	MTP_ADR_H = (addr >> 8) & 0xff;
	MTP_CTL = 0x21 | (space << 1);
	while(MTP_CTL & 0x20);
	//nvr_wp0 = 0x00;  // nvr_wp0 = 0x00;
}


void mcu_flash_init(void)
{
	__mcu_read_flash(NVR_SPACE, 0);		//修复上电第一次读数据为0的bug。
}
