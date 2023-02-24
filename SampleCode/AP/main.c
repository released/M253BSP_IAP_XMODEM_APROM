/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "NuMicro.h"

#include "misc_config.h"


/*_____ D E C L A R A T I O N S ____________________________________________*/

struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_TIMER_PERIOD_500MS                   	(flag_PROJ_CTL.bit1)
#define FLAG_PROJ_REVERSE2                 				(flag_PROJ_CTL.bit2)
#define FLAG_PROJ_REVERSE3                              (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_REVERSE4                              (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_REVERSE5                              (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_REVERSE6                              (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_REVERSE7                              (flag_PROJ_CTL.bit7)


/*_____ D E F I N I T I O N S ______________________________________________*/

volatile unsigned int counter_systick = 0;
volatile uint32_t counter_tick = 0;


#define APROM_APPLICATION_START      		            0x00004800UL
#define APROM_APPLICATION_SIZE      		            0x0001B800UL

// #define APROM_1
#define APROM_2

#define DEBUG_UART_PORT							        (UART4)
#define DEBUG_UART_PORT_IRQn					        (UART4_IRQn)
#define DEBUG_UART_IRQHandler					        (UART4_IRQHandler)

// for MDK AC5
// #define __noinit__ __attribute__((zero_init))
// __noinit__ int flag_check_ISP_process __attribute__((at( 0x20003FF4)));
// __noinit__ int flag_simple_test __attribute__((at( 0x20003FF8)));

// for MDK AC6
#if defined ( __GNUC__ ) && !defined(__ARMCC_VERSION)
int flag_check_ISP_process __attribute__((section(".ram_no_init_bss.ARM.__at_0x20003FF4")));
#else
int flag_check_ISP_process __attribute__((section(".bss.ARM.__at_0x20003FF4")));
#endif
// int flag_simple_test __attribute__((section(".bss.ARM.__at_0x20003FF8")));

#define RST_ADDR_LDROM                                  (0)
#define RST_ADDR_APROM                                  (1)
#define RST_SEL_NVIC                                    (0)
#define RST_SEL_CPU                                     (1)
#define RST_SEL_CHIP                                    (2)


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

unsigned int get_systick(void)
{
	return (counter_systick);
}

void set_systick(unsigned int t)
{
	counter_systick = t;
}

void systick_counter(void)
{
	counter_systick++;
}

void SysTick_Handler(void)
{

    systick_counter();

    if (get_systick() >= 0xFFFFFFFF)
    {
        set_systick(0);      
    }

    // if ((get_systick() % 1000) == 0)
    // {
       
    // }

    #if defined (ENABLE_TICK_EVENT)
    TickCheckTickEvent();
    #endif    
}

void SysTick_delay(unsigned int delay)
{  
    
    unsigned int tickstart = get_systick(); 
    unsigned int wait = delay; 

    while((get_systick() - tickstart) < wait) 
    { 
    } 

}

void SysTick_enable(unsigned int ticks_per_second)
{
    set_systick(0);
    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while (1);
    }

    #if defined (ENABLE_TICK_EVENT)
    TickInitTickEvent();
    #endif
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}

// void delay_ms(uint16_t ms)
// {
// 	TIMER_Delay(TIMER0, 1000*ms);
// }

void SystemReboot_RST(unsigned char addr , unsigned char sel)
{
    while(!UART_IS_TX_EMPTY(DEBUG_UART_PORT));
        
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();

    switch(addr) // CONFIG: w/ IAP
    {
        case RST_ADDR_LDROM:
            /* Mask all interrupt before changing VECMAP to avoid wrong interrupt handler fetched */
            __set_PRIMASK(1);    
            FMC_SetVectorPageAddr(FMC_LDROM_BASE);
            FMC_SELECT_NEXT_BOOT(1);    //FMC_SET_LDROM_BOOT();        
            break;
        case RST_ADDR_APROM:
            /* Mask all interrupt before changing VECMAP to avoid wrong interrupt handler fetched */
            __set_PRIMASK(1);    
            FMC_SetVectorPageAddr(FMC_APROM_BASE);
            FMC_SELECT_NEXT_BOOT(0);    //FMC_SET_APROM_BOOT();        
            break;            
    }

    switch(sel)
    {
        case RST_SEL_NVIC:  // Reset I/O and peripherals , only check BS(FMC_ISPCTL[1])
            NVIC_SystemReset();
            break;
        case RST_SEL_CPU:   // Not reset I/O and peripherals
            SYS_ResetCPU();
            break;   
        case RST_SEL_CHIP:
            SYS_ResetChip();// Reset I/O and peripherals ,  BS(FMC_ISPCTL[1]) reload from CONFIG setting (CBS)
            break;                       
    } 
}

void FMC_ISP(uint32_t u32Cmd, uint32_t u32Addr, uint32_t u32Data)
{
    FMC_ENABLE_AP_UPDATE();    
    
    FMC->ISPCMD = u32Cmd;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    __ISB();
    //while(FMC->ISPTRG);
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }
    
    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {    
        
        printf("ISPCTL : ISPFF(0x%2X)\r\n" , FMC->ISPCTL);
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        while(1);
    }
}

void erase_checksum(uint32_t u32Addr, uint32_t u32Data)
{
    printf("%s:0x%8X,0x%8X\r\n",__FUNCTION__,u32Addr,u32Data);
    while(!UART_IS_TX_EMPTY(DEBUG_UART_PORT));

    SYS_UnlockReg();
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk | CLK_AHBCLK_EXSTCKEN_Msk;
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk;

    if ((u32Addr & (FMC_FLASH_PAGE_SIZE - 1)) == 0)
        FMC_ISP(FMC_ISPCMD_PAGE_ERASE, u32Addr, 0);


    FMC_ISP(FMC_ISPCMD_PROGRAM, u32Addr, u32Data);

}

uint8_t read_magic_tag(void)
{
    uint8_t tag = 0;

    tag = (uint8_t) flag_check_ISP_process;

    printf("Read MagicTag <0x%02X>\r\n", tag);
    
    return tag;
}

void write_magic_tag(uint8_t tag)
{
    flag_check_ISP_process = tag;    

    printf("Write MagicTag <0x%02X>\r\n", tag);
}

void TMR1_IRQHandler(void)
{
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_1000MS = 1;//set_flag(flag_timer_period_1000ms ,ENABLE);
		}
		
		if ((get_tick() % 500) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_500MS = 1;
		}		

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void loop(void)
{
	static uint32_t LOG = 0;

    if ((get_systick() % 1000) == 0)
    {
        // printf("%s(systick) : %4d\r\n",__FUNCTION__,LOG2++);    
    }
    
	#if defined (APROM_1)
	if (FLAG_PROJ_TIMER_PERIOD_1000MS)
	{
		FLAG_PROJ_TIMER_PERIOD_1000MS = 0;
    	printf("1)%s : %4d\r\n",__FUNCTION__,LOG++);
		PB0 ^= 1;
	}
	#endif

	#if defined (APROM_2)
	if (FLAG_PROJ_TIMER_PERIOD_500MS)
	{
		FLAG_PROJ_TIMER_PERIOD_500MS = 0;
    	printf("2)%s : %4d\r\n",__FUNCTION__,LOG++);
		PB0 ^= 1;
	}
	#endif    
    
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(DEBUG_UART_PORT);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		printf("press : %c\r\n" , res);
		switch(res)
		{
			case '1':
    			read_magic_tag();			
	            write_magic_tag(0xA5);
    			read_magic_tag();

	            printf("Perform RST to enter BOOTLOADER\r\n");
	            SystemReboot_RST(RST_ADDR_APROM,RST_SEL_CPU);
				break;

			case '2':
    			erase_checksum(0x1FFFC , 0x0000);   // 0x20000 - 4

	            printf("Perform RST to enter BOOTLOADER\r\n");
	            SystemReboot_RST(RST_ADDR_APROM,RST_SEL_CPU);
				break;                

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
				NVIC_SystemReset();		
				break;
		}
	}
}

void DEBUG_UART_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(DEBUG_UART_PORT, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(DEBUG_UART_PORT) == 0)
        {
			UARTx_Process();
        }
    }

    if(DEBUG_UART_PORT->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(DEBUG_UART_PORT, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void DEBUG_UART_Init(void)
{
    SYS_ResetModule(UART4_RST);

    /* Configure UART4 and set UART4 baud rate */
    UART_Open(DEBUG_UART_PORT, 115200);
    UART_EnableInt(DEBUG_UART_PORT, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(DEBUG_UART_PORT_IRQn);
	
	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif	

    #if 0
    printf("FLAG_PROJ_TIMER_PERIOD_1000MS : 0x%2X\r\n",FLAG_PROJ_TIMER_PERIOD_1000MS);
    printf("FLAG_PROJ_REVERSE1 : 0x%2X\r\n",FLAG_PROJ_REVERSE1);
    printf("FLAG_PROJ_REVERSE2 : 0x%2X\r\n",FLAG_PROJ_REVERSE2);
    printf("FLAG_PROJ_REVERSE3 : 0x%2X\r\n",FLAG_PROJ_REVERSE3);
    printf("FLAG_PROJ_REVERSE4 : 0x%2X\r\n",FLAG_PROJ_REVERSE4);
    printf("FLAG_PROJ_REVERSE5 : 0x%2X\r\n",FLAG_PROJ_REVERSE5);
    printf("FLAG_PROJ_REVERSE6 : 0x%2X\r\n",FLAG_PROJ_REVERSE6);
    printf("FLAG_PROJ_REVERSE7 : 0x%2X\r\n",FLAG_PROJ_REVERSE7);
    #endif

}

void GPIO_Init (void)
{
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk)) | (SYS_GPB_MFPL_PB0MFP_GPIO);

    GPIO_SetMode(PB, BIT0, GPIO_MODE_OUTPUT);

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);	

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);	

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    CLK_EnableModuleClock(GPB_MODULE);

    CLK_EnableModuleClock(TMR1_MODULE);
  	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    /* Debug UART clock setting */
    UartDebugCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    UartDebugMFP();


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M251 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART4 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	GPIO_Init();
	DEBUG_UART_Init();
	TIMER1_Init();

	write_magic_tag(0x00);//flag_check_ISP_process = 0x00;
    read_magic_tag();// for debug
    SysTick_enable(1000);
    #if defined (ENABLE_TICK_EVENT)
    TickSetTickEvent(1000, TickCallback_processA);  // 1000 ms
    TickSetTickEvent(5000, TickCallback_processB);  // 5000 ms
    #endif


    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
