#ifndef _RF_H_
#define _RF_H_

#include "_rf.h"

extern void process_rf_communication(void);
extern UINT8 Send_Packet_Ack(UINT8 *buf,UINT8 length);
extern void RF_id_set(void);
extern void RF_Data_Send(void);

#endif
