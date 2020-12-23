#ifndef __GT102_DEF_H
#define __GT102_DEF_H

//I2C addr
#define I2C_ADDR			0xD0	/* 0x68<<1 */

//address
#define	ADDR_PAGE0			0x00  	 			// PAGE0 start address
#define	ADDR_PAGE1			0x20  				// PAGE1 start address
#define	ADDR_PAGE2			0x40 				// PAGE2 start address
#define	ADDR_PAGE3			0x60  				// PAGE3 start address
#define	ADDR_KEY			0x80  
#define	ADDR_USID			0x90  
#define	ADDR_WDGCNT			0xA0  	/* (0xA0~0xA2)WDOG喂狗间隔时间（4MHz时钟）--*/
#define	ADDR_WDGRSTCTRL		0xA3  
#define	ADDR_RSTCNT			0xA4  	/*（0xA4~0xA6）RST管脚输出有效复位信号脉冲宽度 */
#define	ADDR_PAGE0PRO		0xA8  
#define	ADDR_PAGE1PRO		0xA9  
#define	ADDR_PAGE2PRO		0xAA  
#define	ADDR_PAGE3PRO		0xAB  
#define	ADDR_KEYPRO			0xAC  
#define	ADDR_UIDPRO			0xAD  
#define	ADDR_PRT_CTRL		0xAE  
#define	ADDR_DIS_INITPAGE 	0xAF 
#define	ADDR_CMD			0xB0	/* command reg:write command to it when we want to access (0x00-0xaf)area. */
#define	ADDR_TA_SRC			0xB1	/* source address reg:specify the PAGEx(0,1,2,3) used for computing MAC. */
#define	ADDR_TA_DST			0xB2	/* target address reg:specify the address which you will access. */
#define	ADDR_ES				0xB3	/* command executing status reg(read only) */
#define	ADDR_SYSCTRL		0xB4
#define ADDR_VERSION0       0xB8	/* 0xB8-0xBB 版本号区 ------------------*/
#define	ADDR_MEMBUF			0xC0	//buffer
#define	ADDR_MACBUF			0xE0

//command
#define	CMD_INITUSID		0xAA	//设置用户ID
#define	CMD_AUTHDEV			0x55	//设备认证
#define	CMD_INITPAGE		0xA5	//设置PAGE数据
#define	CMD_INITKEY			0x5A	//设置密钥
#define	CMD_READMEM			0x0F	//读内存
#define	CMD_WRITEMEM		0xF0	//写内存
#define	CMD_GENKEY			0x33	//更新密钥
#define CMD_CMDCLR      	0x00	//清空缓存?

//WDGRSTCTRL
#define	WDOG_EN				(1U<<0)
#define	SHA_RD_BYPASS		(1U<<2)
#define RST_EN_N			(1U<<4)
#define RST_POLARITY		(1U<<5)

//SYSCTRL
#define WDOG_EN_REG			(1U<<4)

//ES bit
#define ES_DONE				(1U<<0)
#define ES_ERR				(1U<<4)

#endif
