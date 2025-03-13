#ifndef _NVR_H_
#define _NVR_H_

#define MAIN_SPACE 0x1
#define NVR_SPACE  0x0


void  __mcu_erase_flash(uint8 space, uint16 addr);
void  __mcu_write_flash(uint8 space, uint16 addr, uint8 wr_data);
uint8 __mcu_read_flash(uint8 space, uint16 addr);

#define mcu_erase_flash(addr)       __mcu_erase_flash(NVR_SPACE, addr)
#define mcu_write_flash(addr, dat)  __mcu_write_flash(NVR_SPACE, addr, dat)
#define mcu_read_flash(addr)        __mcu_read_flash(NVR_SPACE, addr)

#endif
