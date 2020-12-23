#ifndef __MEMMAP_H
#define __MEMMAP_H

//IROM MAP
#define MBRLDR_ADDR 0x08000000
#define MBRLDR_SIZE 0x00004000	//32KB

#define DFUMBR_ADDR 0x08008000
#define DFUMBR_SIZE 0x00004000  //32KB

#define NOTUSE_ADDR 0x08010000
#define NOTUSE_SIZE 0x00010000  //64KB

#define DFUNRF_ADDR 0x08020000
#define DFUNRF_SIZE 0x00020000  //128KB

#define DFUAPP_ADDR 0x08040000
#define DFUAPP_SIZE 0x00040000	//256KB

#define APPROM_ADDR 0x08080000
#define APPROM_SIZE 0x00040000	//256KB


//FLASH MAP
#define SYSCFG_ADDR 0x00000000	//ϵͳ����(����MAC�����к�)
#define SYSCFG_SIZE 0x00001000

#define APPCFG_ADDR 0x00001000	//Ӧ������(���޸ĵ�����)
#define APPCFG_SIZE 0x00001000

#define TRACKS_ADDR 0x00004000  //Ѳ���켣
#define TRACKS_SIZE 0x00004000

#endif