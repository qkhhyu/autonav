#ifndef __DFU_H
#define __DFU_H

#define DFU_TYPE_APP    0
#define DFU_TYPE_NRF1   1
#define DFU_TYPE_NRF2   2
#define DFU_TYPE_NRF    3
#define DFU_SIZE_MAX    0x40000

int dfu_setup(int type, void *data, uint32_t size);

#endif
