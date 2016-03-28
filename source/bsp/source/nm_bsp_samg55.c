/**
 *
 * \file
 *
 * \brief This module contains SAMG55 BSP APIs implementation.
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#ifdef __CONF_ASF__
#include "asf.h"
#endif
#include "config/conf_winc.h"

#define  CUT_OFF 0

static tpfNmBspIsr gpfIsr;

#if CONF_MBED_SAMG55_BTN
extern void at_button_cb();
extern void at_oled_button1_cb();
extern void at_oled_button2_cb();
extern void at_oled_button2_cb();


static void sw0_isr(uint32 id, uint32 mask)
{
	if ((id == ID_PIOA) && (mask == PIO_PA2)) {
		at_button_cb();
	}
}

static void oled_btn1_isr(uint32 id, uint32 mask)
{
	int delay_cnt = 5;
	
	if ((id == ID_PIOB) && (mask == PIO_PB3)) {
		pio_disable_interrupt(PIOB, PIO_PB3);
		at_oled_button1_cb();

		while(delay_cnt--) {
			if ((PIOB->PIO_PDSR & mask) != PIO_PB3) {
				nm_bsp_sleep(100);
				if ((PIOB->PIO_PDSR & mask) != PIO_PB3) {
					nm_bsp_sleep(100);
					//printf("johnny d_cnt 1 = %d\n\r",delay_cnt);
					break;
				}
			}
			nm_bsp_sleep(100);
		}
		//printf("johnny d_cnt 2 = %d\n\r",delay_cnt);
		pio_enable_interrupt(PIOB, PIO_PB3);
	}	
}

static void oled_btn2_isr(uint32 id, uint32 mask)
{
	/*if ((id == ID_PIOA) && (mask == PIO_PA19)) {
		at_oled_button2_cb();
	}*/
	
	int delay_cnt = 5;
	
	if ((id == ID_PIOA) && (mask == PIO_PA19)) {
		pio_disable_interrupt(PIOB, PIO_PA19);
		at_oled_button2_cb();

		while(delay_cnt--) {
			if ((PIOA->PIO_PDSR & mask) != PIO_PA19) {
				nm_bsp_sleep(100);
				if ((PIOA->PIO_PDSR & mask) != PIO_PA19) {
					nm_bsp_sleep(100);
					//printf("johnny d_cnt 1 = %d\n\r",delay_cnt);
					break;
				}
			}
			nm_bsp_sleep(100);
		}
		//printf("johnny d_cnt 2 = %d\n\r",delay_cnt);
		pio_enable_interrupt(PIOA, PIO_PA19);
	}
}

static void oled_btn3_isr(uint32 id, uint32 mask)
{
	int delay_cnt = 5;

	if ((id == ID_PIOA) && (mask == PIO_PA20)) {
		pio_disable_interrupt(PIOA, PIO_PA20);
		at_oled_button3_cb();

		while(delay_cnt--) {
			if ((PIOA->PIO_PDSR & mask) != PIO_PA20) {
				nm_bsp_sleep(100);
				if ((PIOA->PIO_PDSR & mask) != PIO_PA20) {
					nm_bsp_sleep(100);
					//printf("johnny d_cnt 1 = %d\n\r",delay_cnt);
					break;
				}
			}
			nm_bsp_sleep(100);
		}
		//printf("johnny d_cnt 2 = %d\n\r",delay_cnt);
		pio_enable_interrupt(PIOA, PIO_PA20);
	}
}
#endif


static void chip_isr(uint32_t id, uint32_t mask)
{
	if ((id == CONF_WINC_SPI_INT_PIO_ID) && (mask == CONF_WINC_SPI_INT_MASK)) {
		if (gpfIsr) {
			gpfIsr();
		}
	}
}

/*
 *	@fn		init_chip_pins
 *	@brief	Initialize reset, chip enable and wake pin
 */
static void init_chip_pins(void)
{
	ioport_init();
	ioport_set_pin_dir(CONF_WINC_PIN_RESET, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CONF_WINC_PIN_RESET, IOPORT_PIN_LEVEL_HIGH);
	ioport_set_pin_dir(CONF_WINC_PIN_CHIP_ENABLE, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CONF_WINC_PIN_CHIP_ENABLE, IOPORT_PIN_LEVEL_HIGH);
	ioport_set_pin_dir(CONF_WINC_PIN_WAKE, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CONF_WINC_PIN_WAKE, IOPORT_PIN_LEVEL_HIGH);
}

/*
*	@fn		nm_bsp_init
*	@brief	Initialize BSP
*	@return	0 in case of success and -1 in case of failure
*/
sint8 nm_bsp_init(void)
{
	gpfIsr = NULL;

	/* Initialize chip IOs. */
	init_chip_pins();

    /* Make sure a 1ms Systick is configured. */
    if (!(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk && SysTick->CTRL & SysTick_CTRL_TICKINT_Msk)) {
	    delay_init();
    }

	/* Perform chip reset. */
	nm_bsp_reset();

	return 0;
}

/**
 *	@fn		nm_bsp_reset
 *	@brief	Reset WINC1500 SoC by setting CHIP_EN and RESET_N signals low,
 *           CHIP_EN high then RESET_N high
 */
void nm_bsp_reset(void)
{
	ioport_set_pin_level(CONF_WINC_PIN_CHIP_ENABLE, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_level(CONF_WINC_PIN_RESET, IOPORT_PIN_LEVEL_LOW);
	nm_bsp_sleep(100);
	ioport_set_pin_level(CONF_WINC_PIN_CHIP_ENABLE, IOPORT_PIN_LEVEL_HIGH);
	nm_bsp_sleep(100);
	ioport_set_pin_level(CONF_WINC_PIN_RESET, IOPORT_PIN_LEVEL_HIGH);
	nm_bsp_sleep(100);
}

/*
*	@fn		nm_bsp_sleep
*	@brief	Sleep in units of mSec
*	@param[IN]	u32TimeMsec
*				Time in milliseconds
*/
void nm_bsp_sleep(uint32 u32TimeMsec)
{
	while(u32TimeMsec--) {
		delay_ms(1);
	}
}

/*
*	@fn		nm_bsp_register_isr
*	@brief	Register interrupt service routine
*	@param[IN]	pfIsr
*				Pointer to ISR handler
*/
void nm_bsp_register_isr(tpfNmBspIsr pfIsr)
{
	gpfIsr = pfIsr;

	/* Configure PGIO pin for interrupt from SPI slave, used when slave has data to send. */
	//pmc_enable_periph_clk(CONF_WINC_SPI_INT_PIO_ID);
	pio_configure_pin(CONF_WINC_SPI_INT_PIN, PIO_TYPE_PIO_INPUT);
	pio_pull_up(CONF_WINC_SPI_INT_PIO, CONF_WINC_SPI_INT_MASK, PIO_PULLUP);
	/*Interrupt on falling edge*/
	pio_handler_set(CONF_WINC_SPI_INT_PIO, CONF_WINC_SPI_INT_PIO_ID,
	CONF_WINC_SPI_INT_MASK, PIO_PULLUP | PIO_IT_FALL_EDGE, chip_isr);
	pio_enable_interrupt(CONF_WINC_SPI_INT_PIO, CONF_WINC_SPI_INT_MASK);
	NVIC_EnableIRQ((IRQn_Type) CONF_WINC_SPI_INT_PIO_ID);
	pio_handler_set_priority(CONF_WINC_SPI_INT_PIO, (IRQn_Type)CONF_WINC_SPI_INT_PIO_ID,
			CONF_WINC_SPI_INT_PRIORITY);

#if CONF_MBED_SAMG55_BTN
#if 0
	// SW0 Button
	pio_configure_pin(IOPORT_CREATE_PIN(PIOA, 2), PIO_TYPE_PIO_INPUT);
	pio_pull_up(PIOA, PIO_PA2, PIO_PULLUP);
	pio_handler_set(PIOA, ID_PIOA, PIO_PA2, PIO_PULLUP | PIO_IT_FALL_EDGE, sw0_isr);
	pio_enable_interrupt(PIOA, PIO_PA2);
	NVIC_EnableIRQ((IRQn_Type) ID_PIOA);
	//pio_handler_set_priority(PIOA, (IRQn_Type)ID_PIOA, CONF_WINC_SPI_INT_PRIORITY);

	// OLED Button1
	pio_configure_pin(IOPORT_CREATE_PIN(PIOB, 3), PIO_TYPE_PIO_INPUT);
	pio_pull_up(PIOB, PIO_PB3, PIO_PULLUP);
	pio_handler_set(PIOB, ID_PIOB, PIO_PB3, PIO_PULLUP | PIO_IT_FALL_EDGE, oled_btn1_isr);
	pio_enable_interrupt(PIOB, PIO_PB3);
	NVIC_EnableIRQ((IRQn_Type) ID_PIOB);
	//pio_handler_set_priority(PIOA, (IRQn_Type)ID_PIOA, CONF_WINC_SPI_INT_PRIORITY);
	
	// OLED Button2
	pio_configure_pin(IOPORT_CREATE_PIN(PIOA, 19), PIO_TYPE_PIO_INPUT);
	pio_pull_up(PIOA, PIO_PA19, PIO_PULLUP);
	pio_handler_set(PIOA, ID_PIOA, PIO_PA19, PIO_PULLUP | PIO_IT_FALL_EDGE, oled_btn2_isr);
	pio_enable_interrupt(PIOA, PIO_PA19);
	NVIC_EnableIRQ((IRQn_Type) ID_PIOA);
	//pio_handler_set_priority(PIOA, (IRQn_Type)ID_PIOA, CONF_WINC_SPI_INT_PRIORITY);

	// OLED Button3
	pio_configure_pin(IOPORT_CREATE_PIN(PIOA, 20), PIO_TYPE_PIO_INPUT);
	pio_pull_up(PIOA, PIO_PA20, PIO_PULLUP);
	pio_handler_set(PIOA, ID_PIOA, PIO_PA20, PIO_PULLUP | PIO_IT_FALL_EDGE, oled_btn3_isr);
	pio_enable_interrupt(PIOA, PIO_PA20);
	NVIC_EnableIRQ((IRQn_Type) ID_PIOA);
	//pio_handler_set_priority(PIOA, (IRQn_Type)ID_PIOA, CONF_WINC_SPI_INT_PRIORITY);
#else
	// SW0 Button
	/* Adjust PIO debounce filter parameters, using 10 Hz filter. */
    pio_set_debounce_filter(PIOA, PIO_PA2, 10);
    /* Initialize PIO interrupt handlers, see PIO definition in board.h. */
    pio_handler_set(PIOA, ID_PIOA, PIO_PA2, PIO_PULLUP | PIO_IT_RISE_EDGE, sw0_isr);
    /* Enable PIO controller IRQs. */
    NVIC_EnableIRQ((IRQn_Type)ID_PIOA);
    /* Enable PIO line interrupts. */
    pio_enable_interrupt(PIOA, PIO_PA2);
	

	// OLED Button1
	/* Adjust PIO debounce filter parameters, using 10 Hz filter. */
    pio_set_debounce_filter(PIOB, PIO_PB3, CUT_OFF);
    /* Initialize PIO interrupt handlers, see PIO definition in board.h. */
    pio_handler_set(PIOB, ID_PIOB, PIO_PB3, PIO_PULLUP | PIO_IT_RISE_EDGE, oled_btn1_isr);
    /* Enable PIO controller IRQs. */
    NVIC_EnableIRQ((IRQn_Type) ID_PIOB);
    /* Enable PIO line interrupts. */
    pio_enable_interrupt(PIOB, PIO_PB3);
	
	
	// OLED Button2
	/* Adjust PIO debounce filter parameters, using 10 Hz filter. */
    pio_set_debounce_filter(PIOA, PIO_PA19, CUT_OFF);
    /* Initialize PIO interrupt handlers, see PIO definition in board.h. */
    pio_handler_set(PIOA, ID_PIOA, PIO_PA19, PIO_PULLUP | PIO_IT_RISE_EDGE, oled_btn2_isr);
    /* Enable PIO controller IRQs. */
    NVIC_EnableIRQ((IRQn_Type) ID_PIOA);
    /* Enable PIO line interrupts. */
    pio_enable_interrupt(PIOA, PIO_PA19);


	// OLED Button3
	/* Adjust PIO debounce filter parameters, using 10 Hz filter. */
    pio_set_debounce_filter(PIOA, PIO_PA20, CUT_OFF);
    /* Initialize PIO interrupt handlers, see PIO definition in board.h. */
    pio_handler_set(PIOA, ID_PIOA, PIO_PA20, PIO_PULLUP | PIO_IT_RISE_EDGE, oled_btn3_isr);
    /* Enable PIO controller IRQs. */
    NVIC_EnableIRQ((IRQn_Type) ID_PIOA);
    /* Enable PIO line interrupts. */
    pio_enable_interrupt(PIOA, PIO_PA20);

#endif

#endif
	
}

/*
*	@fn		nm_bsp_interrupt_ctrl
*	@brief	Enable/Disable interrupts
*	@param[IN]	u8Enable
*				'0' disable interrupts. '1' enable interrupts
*/
void nm_bsp_interrupt_ctrl(uint8 u8Enable)
{
	if (u8Enable) {
		pio_enable_interrupt(CONF_WINC_SPI_INT_PIO, CONF_WINC_SPI_INT_MASK);
	}
	else {
		pio_disable_interrupt(CONF_WINC_SPI_INT_PIO, CONF_WINC_SPI_INT_MASK);
	}
}

