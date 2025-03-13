#include <stdio.h>
#include <stdint.h>
#include "driver_rf.h"

#define TRX_IRQ_STATUS 0x01
#define STATUS_RX_DR 0x01
#define STATUS_RX_EMPTY 0x00
#define MAX_PACKET_LEN 32
#define DATATYPE_PAGE_START 0x01
#define DATATYPE_PAGE_END_MOUSE 0x02

uint8_t rf_fifo_data[MAX_PACKET_LEN];
uint8_t RF_flag;
uint8_t uRXDataLens;

void R_RX_PAYLOAD(uint8_t *pBuf, uint8_t bytes) {
}

void RF_CMD_FLUSH_RX(void) {
}

int main(void) {
    return 0;
}
