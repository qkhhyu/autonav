/*
* STM32F429 SDRAMÇý¶¯
* ½¯Ïþ¸Ú<kerndev@foxmail.com>
* 2016.3.23
*/
#include "stm32f4xx.h"
#include "gpio.h"
#include "sdram.h"

static void sdram_init_pin(void)
{
	gpio_open(PB, 6,GPIO_MODE_AF,GPIO_AF_FMC);
	
	gpio_open(PD, 0,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PD, 1,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PD, 8,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PD, 9,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PD,10,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PD,14,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PD,15,GPIO_MODE_AF,GPIO_AF_FMC);

	gpio_open(PE, 0,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PE, 1,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PE, 7,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PE, 8,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PE, 9,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PE,10,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PE,11,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PE,12,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PE,13,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PE,14,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PE,15,GPIO_MODE_AF,GPIO_AF_FMC);

	gpio_open(PF, 0,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PF, 1,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PF, 2,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PF, 3,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PF, 4,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PF, 5,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PF,11,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PF,12,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PF,13,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PF,14,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PF,15,GPIO_MODE_AF,GPIO_AF_FMC);

	gpio_open(PG, 0,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PG, 1,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PG, 4,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PG, 5,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PG, 8,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PG,15,GPIO_MODE_AF,GPIO_AF_FMC);
	
	gpio_open(PH, 5,GPIO_MODE_AF,GPIO_AF_FMC);
	gpio_open(PH, 7,GPIO_MODE_AF,GPIO_AF_FMC);
}

static void sdram_init_fmc(void)
{
	FMC_SDRAMInitTypeDef init;
	FMC_SDRAMTimingInitTypeDef timing;
	FMC_SDRAMCommandTypeDef cmd;
	
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FMC, ENABLE);
	
	/* Step 1 ----------------------------------------------------*/ 
	/* Timing configuration for 90 Mhz of SD clock frequency (180Mhz/2) */
	/* TMRD: 2 Clock cycles */
	/* 1 clock cycle = 1 / 90MHz = 11.1ns */
	timing.FMC_LoadToActiveDelay	= 2;      
	/* TXSR: min=70ns (7x11.10ns) */
	timing.FMC_ExitSelfRefreshDelay	= 7;
	/* TRAS: min=42ns (4x11.10ns) max=120k (ns) */
	timing.FMC_SelfRefreshTime		= 4;
	/* TRC:  min=70 (7x11.10ns) */        
	timing.FMC_RowCycleDelay		= 7;         
	/* TWR:  min=1+ 7ns (1+1x11.10ns) */
	timing.FMC_WriteRecoveryTime	= 2;      
	/* TRP:  20ns => 2x11.10ns */
	timing.FMC_RPDelay				= 2;                
	/* TRCD: 20ns => 2x11.10ns */
	timing.FMC_RCDDelay				= 2;
	
	/* FMC SDRAM control configuration */
	init.FMC_Bank 						= FMC_Bank2_SDRAM;
	/* Row addressing: [7:0] */
	init.FMC_ColumnBitsNumber 			= FMC_ColumnBits_Number_8b;
	/* Column addressing: [11:0] */
	init.FMC_RowBitsNumber      		= FMC_RowBits_Number_12b;
	init.FMC_SDMemoryDataWidth  		= FMC_SDMemory_Width_16b;
	init.FMC_InternalBankNumber 		= FMC_InternalBank_Number_4;
	init.FMC_CASLatency               	= FMC_CAS_Latency_3;
	init.FMC_WriteProtection 			= FMC_Write_Protection_Disable;
	init.FMC_SDClockPeriod 				= FMC_SDClock_Period_2;
	init.FMC_ReadBurst 					= FMC_Read_Burst_Disable;
	init.FMC_ReadPipeDelay 				= FMC_ReadPipe_Delay_1;
	init.FMC_SDRAMTimingStruct 			= &timing;
	FMC_SDRAMInit(&init);
	
	/* SDRAM Init sequence */
	
	/* Configure a clock configuration enable command */
	cmd.FMC_CommandMode				= FMC_Command_Mode_CLK_Enabled;
	cmd.FMC_CommandTarget 			= FMC_Command_Target_bank2;
	cmd.FMC_AutoRefreshNumber 		= 1;
	cmd.FMC_ModeRegisterDefinition 	= 0;
	
	/* Wait until the SDRAM controller is ready */
	while (FMC_GetFlagStatus(FMC_Bank2_SDRAM, FMC_FLAG_Busy) != RESET);
	
	/* Send the command */
	FMC_SDRAMCmdConfig(&cmd);
	
	/* Configure a PALL (precharge all) command */ 
	cmd.FMC_CommandMode          	= FMC_Command_Mode_PALL;
	cmd.FMC_CommandTarget          	= FMC_Command_Target_bank2;
	cmd.FMC_AutoRefreshNumber      	= 1;
	cmd.FMC_ModeRegisterDefinition 	= 0;
	
	/* Wait until the SDRAM controller is ready */
	while (FMC_GetFlagStatus(FMC_Bank2_SDRAM, FMC_FLAG_Busy) != RESET);
	
	/* Send the command */
	FMC_SDRAMCmdConfig(&cmd);

	/* Configure a Auto-Refresh command */ 
	cmd.FMC_CommandMode            	= FMC_Command_Mode_AutoRefresh;
	cmd.FMC_CommandTarget          	= FMC_Command_Target_bank2;
	cmd.FMC_AutoRefreshNumber      	= 8;
	cmd.FMC_ModeRegisterDefinition 	= 0;
	
	/* Wait until the SDRAM controller is ready */
	while (FMC_GetFlagStatus(FMC_Bank2_SDRAM, FMC_FLAG_Busy) != RESET);
	
	/* Send the command */
	FMC_SDRAMCmdConfig(&cmd);
	
	/* Configure a load Mode register command */
	cmd.FMC_CommandMode            	= FMC_Command_Mode_LoadMode;
	cmd.FMC_CommandTarget          	= FMC_Command_Target_bank2;
	cmd.FMC_AutoRefreshNumber      	= 1;
	cmd.FMC_ModeRegisterDefinition 	= (uint32_t)0x0231;
	
	/* Wait until the SDRAM controller is ready */
	while (FMC_GetFlagStatus(FMC_Bank2_SDRAM, FMC_FLAG_Busy) != RESET);
	
	/* Send the command */
	FMC_SDRAMCmdConfig(&cmd);

	/* Set the refresh rate counter */
	/* (7.81 us x Freq) - 20 = (7.81 * 90MHz) - 20 = 683 */
	/* Set the device refresh counter */
	FMC_SetRefreshCount(680);
	
	/* Wait until the SDRAM controller is ready */
	while (FMC_GetFlagStatus(FMC_Bank2_SDRAM, FMC_FLAG_Busy) != RESET);
}

void sdram_init(void)
{
	sdram_init_pin();
	sdram_init_fmc();
}
