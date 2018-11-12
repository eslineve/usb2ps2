/*******************************************************************************
 Headers
*******************************************************************************/
#include <avr/io.h>        /* Device-specific I/O */
#include <avr/interrupt.h> /* Enable/Disable interrupts */
#include <util/delay.h>    /* Time delay functions */
#include "uart.h"
#include "usb_serial.h"/* USB serial support */

/*******************************************************************************
 Configure hardware
*******************************************************************************/

/* CPU */
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

/* LED */
#define LED_CONFIG      (DDRD  |=  (1<<6))
#define LED_OFF         (PORTD &= ~(1<<6))
#define LED_ON          (PORTD |=  (1<<6))

/* Attn */
#define ATT_PIN         ((PINB & 1) == 1)

/* Clk */
#define CLK_PIN         ((PINB & 2)  > 0)

/* Ack */
#define ACK_CONFIG      (DDRB  |=  (1<<7))
#define ACK_HIGH        (PORTB |=  (1<<7))
#define ACK_LOW         (PORTB &= ~(1<<7))

/* SPI */
#define DDR_SPI         (DDRB)
#define DD_MISO         (3)

/*******************************************************************************
 Utility functions
*******************************************************************************/

/*
	Precision delay macro functions
		- Since this is a simple (mostly) cycle per instruction 16MHz CPU we can
		  spam NOPs in order to do high-quality timing
		- Each NOP is 62.5 nanoseconds
*/
#define DELAY_1_US	{                       \
						__asm__("nop\n\t"); \
						__asm__("nop\n\t"); \
						__asm__("nop\n\t"); \
						__asm__("nop\n\t"); \
						__asm__("nop\n\t"); \
						__asm__("nop\n\t"); \
						__asm__("nop\n\t"); \
						__asm__("nop\n\t"); \
						__asm__("nop\n\t"); \
						__asm__("nop\n\t"); \
						__asm__("nop\n\t"); \
						__asm__("nop\n\t"); \
						__asm__("nop\n\t"); \
						__asm__("nop\n\t"); \
						__asm__("nop\n\t"); \
						__asm__("nop\n\t"); \
					}

#define DELAY_2_US	{               \
						DELAY_1_US; \
						DELAY_1_US  \
					}

/*******************************************************************************
 Controller structure
*******************************************************************************/
enum controller_constants
{
	BUTTON_SELECT     = 0x01,
	BUTTON_L3         = 0x02,
	BUTTON_R3         = 0x04,
	BUTTON_START      = 0x08,
	BUTTON_UP         = 0x10,
	BUTTON_RIGHT      = 0x20,
	BUTTON_DOWN       = 0x40,
	BUTTON_LEFT       = 0x80,
	
	BUTTON_L2         = 0x01,
	BUTTON_R2         = 0x02,
	BUTTON_L1         = 0x04,
	BUTTON_R1         = 0x08,
	BUTTON_TRIANGLE   = 0x10,
	BUTTON_CIRCLE     = 0x20,
	BUTTON_CROSS      = 0x40,
	BUTTON_SQUARE     = 0x80,
	
	JOY_RX            = 0,
	JOY_RY            = 1,
	JOY_LX            = 2,
	JOY_LY            = 3,
	
	PRESSURE_RIGHT    = 0,
	PRESSURE_LEFT     = 1,
	PRESSURE_UP       = 2,
	PRESSURE_DOWN     = 3,
	PRESSURE_TRIANGLE = 4,
	PRESSURE_CIRCLE   = 5,
	PRESSURE_CROSS    = 6,
	PRESSURE_SQUARE   = 7,
	PRESSURE_L1       = 8,
	PRESSURE_R1       = 9,
	PRESSURE_L2       = 10,
	PRESSURE_R2       = 11
};

struct
{
	unsigned char buttons[2];
	unsigned char joystick[4];
	unsigned char pressure[12];
	
	unsigned char control_mode;
	unsigned char config_mode;
	unsigned char motor[4];
} controller;

/*******************************************************************************
 SPI functions
*******************************************************************************/

void spi_init(void)
{
	/* Set MISO output, all others input */
	DDR_SPI = (1 << DD_MISO);
	
	/* Enable SPI */
	SPCR = 0;
	SPCR |= (1 << SPE);
	SPCR |= (1 << DORD);
	SPCR |= (1 << CPOL);
	SPCR |= (1 << CPHA);
	
	/* Clear Registers */
	{
		char clr;
		clr = SPSR;
		clr = SPDR;
	}
}

/*
  SPI communication macro functions for guaranteed inlining behavior
  
  - Functions are send and receive (simultaneous TX/RX)
  - We wait on (SPSR & 0x80) as completion condition
    - If at any point ATT_PIN goes high all transmission resets
  - ACK requires minimum 2us response (according to CI docs)
*/
#define SPI_ACK()				{                                                             \
									ACK_LOW;                                                  \
									DELAY_2_US;                                               \
									ACK_HIGH;                                                 \
								}
#define SPI_TXRX(out)			{	                                                          \
									SPDR = out;                                               \
									while (!(SPSR & 0x80)) if (ATT_PIN) goto restart_polling; \
								}
#define SPI_TXRX_IN(out, in)	{	                                                          \
									SPDR = out;                                               \
									while (!(SPSR & 0x80)) if (ATT_PIN) goto restart_polling; \
									in = SPDR;                                                \
								}								
#define SPI_TXRX_ACK(out)		{	                                                          \
									SPDR = out;                                               \
									while (!(SPSR & 0x80)) if (ATT_PIN) goto restart_polling; \
									SPI_ACK();                                                \
								}									
#define SPI_TXRX_IN_ACK(out, in){	                                                          \
									SPDR = out;                                               \
									while (!(SPSR & 0x80)) if (ATT_PIN) goto restart_polling; \
									in = SPDR;                                                \
									SPI_ACK();                                                \
								}

/*******************************************************************************
 Setup functions
*******************************************************************************/

void reset_controller()
{
	controller.config_mode  = 0x41;
	controller.control_mode = 0x41;
	controller.motor[0]     = 0xff;
	controller.motor[1]     = 0xff;
	controller.motor[2]     = 0xff;
	controller.motor[3]     = 0xff;
	controller.buttons[0]   = 0xff;
	controller.buttons[1]   = 0xff;
	controller.joystick[0]  = 0x80;
	controller.joystick[1]  = 0x80;
	controller.joystick[2]  = 0x80;
	controller.joystick[3]  = 0x80;
	controller.pressure[0]  = controller.pressure[1]  = controller.pressure[3]  = 0;
	controller.pressure[3]  = controller.pressure[4]  = controller.pressure[5]  = 0;
	controller.pressure[6]  = controller.pressure[7]  = controller.pressure[8]  = 0;
	controller.pressure[9]  = controller.pressure[10] = controller.pressure[11] = 0;
}

void setup()
{
	/* Setup pins */
	DDRB   = 0x00;
	PORTB |= 0x01;
	
	/* Confgure 16MHZ CPU Clock */
	CPU_PRESCALE(0); 
	
	/* Confgure LED */
	LED_CONFIG;
	LED_ON;
	
	/* Confgure SPI */
	spi_init();	
	
	/* Confgure ACK */
	ACK_CONFIG;
	ACK_HIGH;
	
	/* Confgure USB */
	uart_init(31250);
	_delay_ms(1000);
	
	/* Initialize controller data */
	reset_controller();
	
	/* Disable interrupts */
	cli();
}

/*******************************************************************************
 main()
*******************************************************************************/
int main(void)
{
	unsigned char buffer[5] = { 0, 0, 0, 0, 0 };
	
	int idle_counter[2] = { 0, 0 };
	int led_state = 1;
	int sequence = 0;
	
	
	/* Setup microcontroller */
  usb_init();
	setup();
	usb_serial_flush_input();
	/* Device loop */
	while (1)
	{
		/* Declarations */
		unsigned char cmd = 0;
		
		/* Begin loop body */
		restart_polling:
		
		/* No command issued - perform idle tasks */
		if (ATT_PIN)
		{
			/*
				Handle idle disconnects
				- After *approximately* one second of inactivity reset the controller's internal state
				- Note that two 16-bit counters are used since 'long' appears to be broken for some reason
			*/
			{
				if (idle_counter[0]++ >= 10000)
				{
					idle_counter[0] = 0;
					if (idle_counter[1]++ > 18)
					{
						reset_controller();
						
						if (led_state == 1)
						{
							LED_OFF;
							led_state = 0;
						}
						else
						{
							LED_ON;
							led_state = 1;	
						}
						
						idle_counter[1] = 0;
					}
				}
			}
			
			/*
				Handle USB controller updates
			*/
			{
				/* Enable interrupts */
				sei();
				DELAY_1_US;
				
				if (uart_available())
				{
					int	tmp;
					
					tmp = uart_getchar();
          
					
					if (tmp != -1)
					{
            char str[10];
            int tmpint = tmp;
            sprintf(str,"%02hhX, ", tmp);
            usb_serial_write(str, 10);
            
						switch (sequence)
						{
							case 0:
							{
								if (tmp != 0x5A)
								{
									break;
								}
								sequence++;
								break;
							}
							case 1:
							{
								buffer[0] = tmp;
								sequence++;
								break;
							}
							case 2:
							{
								buffer[1] = tmp;
								sequence++;
								break;
							}
							case 3:
							{
								buffer[2] = tmp;
								sequence++;
								break;
							}
							case 4:
							{
								buffer[3] = tmp;
								sequence++;
								break;
							}
							case 5:
							{
								buffer[4] = tmp;
								sequence++;
								break;
							}
							case 6:
							{
								controller.buttons[0] = buffer[0];
								controller.buttons[1] = buffer[1];
								controller.joystick[JOY_RX] = buffer[2];
								controller.joystick[JOY_RY] = buffer[3];
								controller.joystick[JOY_LX] = buffer[4];
								controller.joystick[JOY_LY] = tmp;
								
								controller.pressure[PRESSURE_RIGHT]    = (controller.buttons[0] & BUTTON_RIGHT)    ? 0x00 : 0xFF;
								controller.pressure[PRESSURE_LEFT]     = (controller.buttons[0] & BUTTON_LEFT)     ? 0x00 : 0xFF;
								controller.pressure[PRESSURE_UP]       = (controller.buttons[0] & BUTTON_UP)       ? 0x00 : 0xFF;
								controller.pressure[PRESSURE_DOWN]     = (controller.buttons[0] & BUTTON_DOWN)     ? 0x00 : 0xFF;
								controller.pressure[PRESSURE_TRIANGLE] = (controller.buttons[1] & BUTTON_TRIANGLE) ? 0x00 : 0xFF;
								controller.pressure[PRESSURE_CIRCLE]   = (controller.buttons[1] & BUTTON_CIRCLE)   ? 0x00 : 0xFF;
								controller.pressure[PRESSURE_CROSS]    = (controller.buttons[1] & BUTTON_CROSS)    ? 0x00 : 0xFF;
								controller.pressure[PRESSURE_SQUARE]   = (controller.buttons[1] & BUTTON_SQUARE)   ? 0x00 : 0xFF;
								controller.pressure[PRESSURE_L1]       = (controller.buttons[1] & BUTTON_L1)       ? 0x00 : 0xFF;
								controller.pressure[PRESSURE_R1]       = (controller.buttons[1] & BUTTON_R1)       ? 0x00 : 0xFF;
								controller.pressure[PRESSURE_L2]       = (controller.buttons[1] & BUTTON_L2)       ? 0x00 : 0xFF;
								controller.pressure[PRESSURE_R2]       = (controller.buttons[1] & BUTTON_R2)       ? 0x00 : 0xFF;
								
								sequence = 0;
                usb_serial_putchar('\n');
								break;
							}
						}
					}
					else
					{
						sequence = 0;
					}
				}
				
				/* Disable interrupts */
				DELAY_1_US;
				cli();
			}
			
			goto restart_polling;
		}
		
		/* LED reflects acitvity */
		LED_ON;
		
		/* Reset Idle Counters */
		idle_counter[0] = 0;
		idle_counter[1] = 0;
		
		/* First packet is port number */
		SPI_TXRX_IN(0xff, cmd);
		if (cmd != 0x01) goto restart_polling; /* Ignore packets not for port #1 */
		SPI_ACK();
		
		/* Second packet is command - valid commands begin with 0x4 as the high nibble */
		SPI_TXRX_IN(controller.control_mode, cmd);
		if ((cmd & 0xF0) != 0x40) goto restart_polling;
		
		cmd &= 0x0F;
		
		/* Jump table for the commands */
		{
			/* Only 0x42 and 0x43 are valid in normal mode */
			if (!(controller.control_mode & 0x80))
			{
				if (cmd != 0x02 && cmd != 0x03) goto restart_polling;
			}
			
			/* Ack the command packet */
			SPI_ACK();
			
			/* Third byte of header is always 0x5A (padding byte) */
			SPI_TXRX(0x5A);
			
			switch (cmd)
			{
				case 0x0: goto cmd_init_pressure;
				case 0x1: goto cmd_get_available_poll_results;
				case 0x2: goto cmd_poll;
				case 0x3: goto cmd_escape;
				case 0x4: goto cmd_set_major_mode;
				case 0x5: goto cmd_read_ext_status_1;
				case 0x6: goto cmd_read_const_1;
				case 0x7: goto cmd_read_const_2;
				case 0x8: goto restart_polling;
				case 0x9: goto restart_polling;
				case 0xA: goto restart_polling;
				case 0xB: goto restart_polling;
				case 0xC: goto cmd_read_const_3;
				case 0xD: goto cmd_set_poll_cmd_format;
				case 0xE: goto restart_polling;
				case 0xF: goto cmd_set_poll_result_format;
			}
		}
		
		/* 0x40 */
		cmd_init_pressure:
		{
			/* Ack the padding byte */
			SPI_ACK();
			
			/* This response is always the same */
			SPI_TXRX_ACK(0x00);
			SPI_TXRX_ACK(0x00);
			SPI_TXRX_ACK(0x02);
			SPI_TXRX_ACK(0x00);
			SPI_TXRX_ACK(0x00);
			SPI_TXRX(0x5A);
			
			goto restart_polling;
		}
		
		/* 0x41 */
		cmd_get_available_poll_results:
		{
			/* Ack the padding byte */
			SPI_ACK();
			
			/* This response is always the same- following dowty's implementation */
			SPI_TXRX_ACK(0xFF);
			SPI_TXRX_ACK(0xFF);
			SPI_TXRX_ACK(0x03);
			SPI_TXRX_ACK(0x00);
			SPI_TXRX_ACK(0x00);
			SPI_TXRX(0x5A);
			
			goto restart_polling;
		}
		
		/* 0x43 */
		cmd_escape:
		{
			/* If in config mode 0x43  */
			if (controller.control_mode & 0x80)
			{
				unsigned char param = 0;
				
				/* Ack the padding byte */
				SPI_ACK();
				
				/* If the parameter is zero then return to normal mode */
				SPI_TXRX_IN(0x0, param);
				param &= 0x01;
				SPI_ACK();
				
				SPI_TXRX_ACK(0x00);
				SPI_TXRX_ACK(0x00);
				SPI_TXRX_ACK(0x00);
				SPI_TXRX_ACK(0x00);
				SPI_TXRX(0x00);
				
				if (param == 0)
				{
					controller.control_mode = controller.config_mode;
				}
				
				goto restart_polling;
			}
			
			/* If not in config mode 0x43 is substantially 0x42 - so flow into it */
		}
		
		/* 0x42 */
		cmd_poll:
		{
			unsigned char param = 0;
			
			switch (controller.control_mode)
			{
				case 0x41: /* digital mode */
				{
					/* Ack the padding byte */
					SPI_ACK();
					
					SPI_TXRX_IN(controller.buttons[0], param);
					param &= 0x01;
					SPI_ACK();
					
					SPI_TXRX(controller.buttons[1]);
					
					break;
				}
				case 0xF3: /* config mode sends 3 words */
				case 0x73: /* digital buttons, analog joysticks */
				{
					/* Ack the padding byte */
					SPI_ACK();
					
					SPI_TXRX_IN(controller.buttons[0], param);
					param &= 0x01;
					SPI_ACK();
					
					SPI_TXRX_ACK(controller.buttons[1]);
					SPI_TXRX_ACK(controller.joystick[JOY_RX]);
					SPI_TXRX_ACK(controller.joystick[JOY_RY]);
					SPI_TXRX_ACK(controller.joystick[JOY_LX]);
					SPI_TXRX(controller.joystick[JOY_LY]);
					
					break;
				}
				case 0x79: /* analog joysticks, analog buttons */
				{
					/* Ack the padding byte */
					SPI_ACK();
					
					SPI_TXRX_IN(controller.buttons[0], param);
					param &= 0x01;
					SPI_ACK();
					
					SPI_TXRX_ACK(controller.buttons[1]);
					SPI_TXRX_ACK(controller.joystick[0]);
					SPI_TXRX_ACK(controller.joystick[1]);
					SPI_TXRX_ACK(controller.joystick[2]);
					SPI_TXRX_ACK(controller.joystick[3]);
					SPI_TXRX_ACK(controller.pressure[0]);
					SPI_TXRX_ACK(controller.pressure[1]);
					SPI_TXRX_ACK(controller.pressure[2]);
					SPI_TXRX_ACK(controller.pressure[3]);
					SPI_TXRX_ACK(controller.pressure[4]);
					SPI_TXRX_ACK(controller.pressure[5]);
					SPI_TXRX_ACK(controller.pressure[6]);
					SPI_TXRX_ACK(controller.pressure[7]);
					SPI_TXRX_ACK(controller.pressure[8]);
					SPI_TXRX_ACK(controller.pressure[9]);
					SPI_TXRX_ACK(controller.pressure[10]);
					SPI_TXRX(controller.pressure[11]);
					
					break;
				}
			}
			
			/* Enter configuration mode */
			if (cmd == 0x03 && param == 0x01)
			{
				controller.config_mode = controller.control_mode;
				controller.control_mode = 0xF3;
			}
			
			goto restart_polling;
		}
		
		/* 0x44 */
		cmd_set_major_mode:
		{
			unsigned char param = 0;
			
			/* Ack the padding byte */
			SPI_ACK();
			
			SPI_TXRX_IN(0x00, param);
			param &= 0x01;
			SPI_ACK();
			
			SPI_TXRX_ACK(0x00);
			SPI_TXRX_ACK(0x00);
			SPI_TXRX_ACK(0x00);
			SPI_TXRX_ACK(0x00);
			SPI_TXRX(0x00);
			
			if (param == 0x00)
			{
				controller.config_mode = 0x41;
			}
			else
			{
				controller.config_mode = 0x73;
			}
			
			goto restart_polling;
		}
		
		/* 0x45 */
		cmd_read_ext_status_1:
		{
			unsigned char led = ((controller.config_mode & 0x10) >> 4) & 0x01;
			
			/* Ack the padding byte */
			SPI_ACK();
			
			SPI_TXRX_ACK(0x03); /* This is 0x01 for GH */
			SPI_TXRX_ACK(0x02);
			SPI_TXRX_ACK(led);
			SPI_TXRX_ACK(0x02);
			SPI_TXRX_ACK(0x01);
			SPI_TXRX(0x00);
			
			goto restart_polling;
		}
		
		/* 0x46 */
		/* 0x47 */
		/* 0x4C */
		cmd_read_const_1:
		cmd_read_const_2:
		cmd_read_const_3:
		{
			const unsigned char offset[16] =
			{
				0, 0, 0, 0,
				0, 0, 0, 1,
				0, 0, 0, 0,
				2, 0, 0, 0
			};
			
			const unsigned char data[3][2][4] =
			{
				{
					{ 0x01, 0x02, 0x00, 0x0A },
					{ 0x01, 0x01, 0x01, 0x14 }
				},
				{
					{ 0x02, 0x00, 0x01, 0x00 },
					{ 0x02, 0x00, 0x01, 0x00 }
				},
				{
					{ 0x00, 0x04, 0x00, 0x00 },
					{ 0x00, 0x07, 0x00, 0x00 }
				}
			};
			
			unsigned char param = 0;
			
			/* Ack the padding byte */
			SPI_ACK();
			
			SPI_TXRX_IN(0x00, param);
			param &= 0x01;
			SPI_ACK();
			
			SPI_TXRX_ACK(0x00);
			
			SPI_TXRX_ACK(data[offset[cmd]][param][0]);
			SPI_TXRX_ACK(data[offset[cmd]][param][1]);
			SPI_TXRX_ACK(data[offset[cmd]][param][2]);
			SPI_TXRX(data[offset[cmd]][param][3]);
			
			goto restart_polling;
		}
		
		/* 0x4D */
		cmd_set_poll_cmd_format:
		{
			/* Ack the padding byte */
			SPI_ACK();
			
			SPI_TXRX_IN_ACK(controller.motor[0], controller.motor[0]);
			SPI_TXRX_IN_ACK(controller.motor[1], controller.motor[1]);
			SPI_TXRX_IN_ACK(controller.motor[2], controller.motor[2]);
			SPI_TXRX_IN_ACK(controller.motor[3], controller.motor[3]);
			SPI_TXRX_ACK(0xFF);
			SPI_TXRX(0xFF);
			
			goto restart_polling;
		}
		
		/* 0x4F */
		cmd_set_poll_result_format:
		{
			/* Ack the padding byte */
			SPI_ACK();
			
			SPI_TXRX_ACK(0x00);
			SPI_TXRX_ACK(0x00);
			SPI_TXRX_ACK(0x00);
			SPI_TXRX_ACK(0x00);
			SPI_TXRX_ACK(0x00);
			SPI_TXRX(0x5A);
			
			controller.config_mode = 0x79;
			
			goto restart_polling;
		}
	}
}

