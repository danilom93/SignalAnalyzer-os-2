/*
 * sample.c - The simplest loadable kernel module.
 * Intended as a template for development of more
 * meaningful kernel modules.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>

/*
 * Definition of peripherals regs address and masks
*/
#define GPIOH_BASE_ADDR 	0x40021C00
#define TIMER12_BASE_ADDR 	0x40001800
#define RCC_BASE_ADDR 		0x40023800
#define RCC_AHB1ENR_OFFSET 	0x30
#define RCC_APB1ENR_OFFSET  0x40
#define GPIOH_RCC_MASK		0x80
#define TIMER12_RCC_MASK	0x40
#define TIMER12_IRQ_NUMBER	0x2B	
#define CC1IF_MASK 			0x02
#define UIF_MASK			0x01
#define PRS_MS 				0x270F
#define PRS_US				0x0063

/*
 * Definition of ioctl commands
*/
#define	INVALID		-1
#define START		1
#define STOP		0


struct GpioRegs{

	//0x00
	unsigned int moder;
	//0x04
	unsigned int otyper;
	//0x08
	unsigned int ospeedr;
	//0x0C
	unsigned int pupdr;
	//0x10
	unsigned int idr;
	//0x14
	unsigned int odr;
	//0x18
	unsigned int bsrr;
	//0x1C
	unsigned int lckr;
	//0x20
	unsigned int afrl;
	//0x24
	unsigned int afrh;
};

// registers are 16 bits 
struct TimerRegs{

	//0x00
	unsigned short 	cr1;
	//0x02
	unsigned short	reserved_0;
	//0x04
	unsigned int	reserved_1; 	
	//0x08
	unsigned int 	smcr;
	//0x0C
	unsigned int 	dier;
	//0x10
	unsigned int 	sr;
	//0x14
	unsigned int 	egr;
	//0x18
	unsigned int 	ccmr1;
	//0x1C
	unsigned int 	reserved_2;
	//0x20
	unsigned int 	ccer;
	//0x24
	unsigned int 	cnt;
	//0x28
	unsigned int 	psc;
	//0x2C
	unsigned int 	arr;
	//0x30
	unsigned int 	reserved_3;
	//0x34
	unsigned int 	ccr1;
	//0x38
	unsigned int 	ccr2;
};

volatile struct GpioRegs * const GpioH = (struct GpioRegs *) GPIOH_BASE_ADDR;
volatile struct TimerRegs * const Timer12 = (struct TimerRegs *) TIMER12_BASE_ADDR;
volatile unsigned int * const rcc_ahb1enr = (unsigned int *) (RCC_BASE_ADDR + RCC_AHB1ENR_OFFSET);
volatile unsigned int * const rcc_apb1enr = (unsigned int *) (RCC_BASE_ADDR + RCC_APB1ENR_OFFSET);

/*
 * Device major number
 */
static uint module_major = 166;

/*
 * Device name
 */
static char * module_name = "Input Capture";

/*
 * Device access lock. Only one process can access the driver at a time
 */
static int sample_lock = 0;

/*
 * Device variables
*/
static volatile unsigned int 	period; 
static volatile unsigned short 	h_period;

int		mode = INVALID;	// device mode: INVALID = do nothink
	
/*
 * Timer interrupt handler
*/
static irq_handler_t timer_handler( unsigned int irq, struct pt_regs *regs ){

/*	If is an update interrupt (Overflow if UDIS=0 in TIM12_CR1)																		*/
	if(Timer12->sr & UIF_MASK){		

/*		Increments the MSB variables																								*/
		h_period++;					
/*		Resets the interrupt flag																									*/
		Timer12->sr &= ~UIF_MASK;	
	}else{

/*		If is a capture interrupt (Rise edge)																						*/
		if(Timer12->sr & CC1IF_MASK){
		
/*			Saves the period read into a 32-bit variable (MSB<<16 + LSB)															*/
			period = (h_period << 16) | Timer12->ccr1;
/*			Resets the MSB because it is a new rising edge																			*/
			h_period = 0;
		}
	}
	return (irq_handler_t)IRQ_HANDLED;
}

/*
 * Device open
 */
static int sample_open(struct inode *inode, struct file *file){

	int ret = 0;

	/*
	 * One process at a time
	 */
	if (sample_lock > 0){

		ret = -EBUSY;
	}
	else{

		sample_lock++;

		/*
 	 	* Increment the module use counter
 	 	*/
		try_module_get(THIS_MODULE);

		#ifdef SAMPLE_DEBUG 
			printk( KERN_INFO "%s: %s\n", module_name, __func__ ); 
		#endif
	}
	return( ret );
}

/*
 * Device close
 */
static int sample_release(struct inode *inode, struct file *file){

	/*
 	 * Release device
 	 */
	sample_lock = 0;

	/*
 	 * Decrement module use counter
 	 */
	module_put(THIS_MODULE);
	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif
	return( 0 );
}

/* 
 * Device read
 */
static ssize_t sample_read(struct file *filp, char *buffer, size_t length, loff_t * offset){

	int ret = 1;
/*	Passes the period read to the application					 																	*/
	memcpy(buffer, &period, sizeof(period));
	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif
	return( ret );
}

/* 
 * Device write
 */
static ssize_t sample_write(struct file *filp, const char *buffer, size_t length, loff_t * offset){

	int ret = 0;
	switch(mode){

		case INVALID:

			ret = 0;
			break;
		default:

			break;
	}

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif
	return(ret);
}

static ssize_t sample_ioctl(struct inode *inode, struct file *filep, const unsigned int cmd, const unsigned long arg){

	int ret = 0;

	switch(cmd){

		case START:
				
/*			Starts the input capture mode																							*/
/*			Sets CC1E = 1 --> Capture enabled																						*/
			Timer12->ccer |= 0x01;		
/*			Resets interrupt flags																									*/
			Timer12->sr &= 0x00;
/*			Enables the interrupts for the events capture and Update(Overflow)														*/
			Timer12->dier |= 0x03;
			break;
		case STOP:

/*			Stops the input capture mode																							*/
/*			Sets CC1E = 0 --> Capture disabled																						*/
			Timer12->ccer = 0x00;
/*			Disables the interrupts 																								*/
			Timer12->dier = 0x00;
			break;
		default:

			mode = INVALID;
			break;
	}
	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif
	return( ret );
}


/*
 * Device operations
 */
static struct file_operations sample_fops = {
	.read = sample_read,
	.write = sample_write,
	.open = sample_open,
	.release = sample_release,
	.ioctl = sample_ioctl
};

static int __init sample_init_module(void){

	/*
 	 * Register device
 	 */
	int	ret;

	ret = register_chrdev(module_major, module_name, &sample_fops);
	if(ret < 0){

		printk(KERN_INFO "%s: registering device %s with major %d failed with %d\n", __func__, module_name, module_major, module_major );
		return( ret );
	}
	else{
/*		Enables the clock for GPIOH																									*/
		*rcc_ahb1enr |= GPIOH_RCC_MASK;
/*		Enables the clock for TIM12																									*/
		*rcc_apb1enr |= TIMER12_RCC_MASK;
/*		Sets the interrupt handler for TIM12																						*/
		if(request_irq(TIMER12_IRQ_NUMBER, (irq_handler_t)timer_handler, 0, module_name, NULL ) < 0 ){

			printk( KERN_INFO "%s: %s Unable to register Timer12 irq\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}

/*		Sets PH.6 moder as alternate function																						*/
		GpioH->moder |= 0x2000;			
/*		Sets PH.6 alternate function as AF9	(TIM12_CH1)																				*/
		GpioH->afrl |= 0x9000000;

/*		Sets CC1S = 1 --> IC1 is mapped on TI1 (Channel is configured as input)														*/
		Timer12->ccmr1 |= 0x01;	
/*		Sets the prescaler value equal to 99 because CK_CNT = (fCK/(PSC+1))															*/
/*		CK_CNT = 100 MHz / 100 = 1Mhz --> precision is equal to 1 us																*/
		Timer12->psc = PRS_US;
/*		Sets CEN = 1 (Counter enabled), Sets URS = 1 (Only counter overflow generates an update interrupt)							*/
		Timer12->cr1 |= 0x05;
/*		Sets SMS as reset mode (Rising edge reinitializes the counter)																*/
/*		Sets TS as filtered Timer Input 1																							*/
		Timer12->smcr |= 0x54;
		printk(KERN_INFO "%s: registering device %s with major %d\n",__func__, module_name, module_major );
	}
	return(ret);
}

static void __exit sample_cleanup_module(void){

	/*
	 * Free irq
	 */
	free_irq(TIMER12_IRQ_NUMBER, NULL );

	/*
	 * Unregister device
	 */
	unregister_chrdev(module_major, module_name);

	printk(KERN_INFO "%s: unregistering %s done\n", __func__, module_name );
}

module_init(sample_init_module);
module_exit(sample_cleanup_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Massimo Violante, massimo.violante@polito.it");
MODULE_DESCRIPTION("Device Driver Example 1");
