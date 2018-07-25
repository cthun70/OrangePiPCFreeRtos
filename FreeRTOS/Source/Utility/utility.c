/*
 * (C) Copyright 2000
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */


#include <stdio.h>
#include <stdlib.h>
#include <cpu_sun4i.h>
#include <gic.h>
#include <timer.h>
#include <rt_misc.h>

#define uint32_t unsigned int 
#define	GICD_BASE		(SUNXI_GIC400_BASE + 0x1000)
#define	GICC_BASE		(SUNXI_GIC400_BASE + 0x2000)
#define	GICH_BASE		(SUNXI_GIC400_BASE + 0x4000)
#define	GICV_BASE		(SUNXI_GIC400_BASE + 0x5000)

extern void uart_init(int which, uint32_t port, int baud_rate);
extern void uart_char(char MyData);
//extern void __rt_lib_init(unsigned heapbase, unsigned heaptop);
static void config_GIC(void); 

unsigned long read_timer(int Timer);


unsigned int readl(unsigned int *x)
{
    return(*((volatile unsigned int*)x));
}

void writel(unsigned int *x, unsigned int y)
{
    *((volatile unsigned int*)x) = y;
}

#if 0
static void read_timer_status(void)
{
    unsigned int Test1,Test2=0;
    int i =0;
	struct sunxi_timer_reg *timers =(struct sunxi_timer_reg *)SUNXI_TIMER_BASE;
    Test1 = readl(&timers->tirqen);
    Test2 = readl(&timers->tirqsta);
	printf ("Timer Status - %x %x \n\r",Test1,Test2);
	writel(&timers->tirqsta,Test2);
	//    for(i=0x04; i < 0x40 ;i+=4)
    {
		i=0x04;
    	printf ("Read  0x%02x = 0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x \n\r",i,readl((unsigned int *)(GICD_BASE+i+0x100)),readl((unsigned int *)(GICD_BASE+i+0x180)),readl((unsigned int *)(GICD_BASE+i+0xd00)),readl((unsigned int *)(GICD_BASE+i+0x200)),readl((unsigned int *)(GICD_BASE+i+0x280)),readl((unsigned int *)(GICD_BASE+i+0x300)),readl((unsigned int *)(GICD_BASE+i+0x400)),readl((unsigned int *)(GICD_BASE+i+0x800)));
    }                                                                                              
}
#endif

int UtilityInit(void)
{
	uart_init(0,SUNXI_UART0_BASE,115200);
	config_GIC();
	printf ("\n\r");
	printf ("\n\r");
	printf ("UtilityInit Done \n\r");
}

int fputc(int ch, FILE *f)
{
  extern void uart_char(char MyData);
  uart_char(ch);
  return ch;
}

void config_interrupt (int N,int CPU_target) 
{ 
//  SUNXI_GIC400_BASE
	int reg_offset, index, value, address;
	/* Conﬁgure the Interrupt Set-Enable Registers (ICDISERn). * reg_offset = (integer_div(N / 32) * 4 * value = 1<<(N mod 32) */ 
	reg_offset = (N>>3) & 0xFFFFFFFC; 
	index = N & 0x1F; 
	value = 0x1<<index; 
	address = (GICD_BASE+0x100) + reg_offset; 
	printf ("ICDISERn address 0x%08x \n\r",address);
	/* Now that we know the register address and value, set the appropriate bit */ 
	*(int*)address|= value;
	/* Conﬁgure the Interrupt Processor Targets Register (ICDIPTRn) * reg_offset = integer_div(N / 4) * 4 * index = N mod 4 */ 
	reg_offset = (N & 0xFFFFFFFC); 
	index = N & 0x3; 
	address = (GICD_BASE+0x800) + reg_offset + index; 
	printf ("ICDIPTRn address 0x%08x \n\r",address);
	/* Now that we know the register address and value, write to (only) the appropriate byte */ 
	*(char*)address = (char) CPU_target;
}	

static void config_GIC(void) 
{ 
	config_interrupt (50, 1); // conﬁgure the KEYs parallel port (Interrupt ID = 73)
	config_interrupt (51, 1); // conﬁgure the KEYs parallel port (Interrupt ID = 73)
	// Set Interrupt Priority Mask Register (ICCPMR). Enable interrupts of all priorities 
	*((int*)(GICC_BASE+4)) = 0xFFFF;
	// Set CPU Interface Control Register (ICCICR). Enable signaling of interrupts 
	*((int*)(GICC_BASE+0)) = 3;
	// Conﬁgure the Distributor Control Register (ICDDCR) to send pending interrupts to CPUs 
	*((int*) (GICD_BASE+0x00) ) = 3;
    __enable_irq();
}

unsigned long read_timer(int Timer)
{
	struct sunxi_timer_reg *timers =(struct sunxi_timer_reg *)SUNXI_TIMER_BASE;
	struct sunxi_timer *timer = &timers->timer[Timer];

	/*
	 * The hardware timer counts down, therefore we invert to
	 * produce an incrementing timer.
	 */
	return ~readl(&timer->val);
}

void TimerInitialize()
{
//    int counter = 31422;
    int counter = 24000;
	struct sunxi_timer_reg *timers =(struct sunxi_timer_reg *)SUNXI_TIMER_BASE;
	struct sunxi_timer *timer = &timers->timer[0];
    writel(&timer->inter,counter);
    writel(&timer->ctl,0x07);
	timer = &timers->timer[1];
    writel(&timer->inter,0xffffffff);
    writel(&timer->ctl,0x07);
    writel(&timers->tirqen,0x01);
}

void FIQInterruptHandler(void)
{
	printf ("FIQInterruptHandler \n\r");
}
#if 0
static int Count = 0;
extern void FreeRTOS_Tick_Handler( void );
void IRQInterruptHandler(void)
{
  	struct sunxi_timer_reg *timers =(struct sunxi_timer_reg *)SUNXI_TIMER_BASE;
	unsigned int *Test =(unsigned int *)(GICC_BASE+0x0c);
	unsigned int IrqId = readl(Test);
    switch(IrqId)
    {
        case 50:
            writel(&timers->tirqsta,0x01);
        	printf ("IRQInterruptHandler %d \n\r",IrqId);
            break;
        case 51:
            writel(&timers->tirqsta,0x02);
            FreeRTOS_Tick_Handler();
//        	printf ("IRQInterruptHandler %d \n\r",IrqId);
            break;
        default:
        	printf ("Unknown IRQInterruptHandler %d \n\r",IrqId);
            break;
    }
	Test =(unsigned int *)(GICC_BASE+0x10);
	writel(Test,IrqId);	
    __enable_irq();
}
#endif

#if 0
void ServiceInterruptHandler(void)
{
	printf ("ServiceInterruptHandler \n\r");
}
#endif

void PreFetchInterruptHandler(void)
{
	printf ("PreFetchInterruptHandler \n\r");
}
void AbortInterruptHandler(void)
{
	printf ("AbortInterruptHandler \n\r");
}
void HypervisorInterruptHandler(void)
{
	printf ("HypervisorInterruptHandler \n\r");
}



