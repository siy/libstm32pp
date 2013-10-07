/*******************************************************************************
 *
 * Copyright (C) 2012 Jorge Aparicio <jorge.aparicio.r@gmail.com>
 * Copyright (C) 2013 Rommel Marcelo <jaqueza@gmail.com>
 *
 * This file is part of libstm32pp.
 *
 * libstm32pp is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * libstm32pp is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with libstm32pp. If not, see <http://www.gnu.org/licenses/>.
 *
 ******************************************************************************/

// DO NOT INCLUDE THIS FILE ANYWHERE. THIS DEMO IS JUST A REFERENCE TO BE USED
// IN YOUR MAIN SOURCE FILE.

#include "clock.hpp"

#include "exception.hpp"
#include "core/stk.hpp"

#include "peripheral/gpio.hpp"
#include "peripheral/usart.hpp"

typedef PD13 LD3; // ORANGE
typedef PD12 LD4; // GREEN
typedef PD14 LD5; // RED
typedef PD15 LD6; // BLUE


typedef PA2 U2TXPIN;
typedef PA3 U2RXPIN;

#include "peripheral/tim.hpp"
#include "peripheral/rtc.hpp"

/* Needed for printf */
#include <stdio.h>

volatile u32 tick;

/* Seems needed if using HSE clock */
void clk::hseFailureHandler()
{
//  Do something if high speed clock fails
}

void initializeSystemTimer()
{
	tick = 0;
	/* Set SysTick to interrupt at twice a second
	 * to have a 1 second ON 1 second OFF of LED6
	 */
	STK::configurePeriodicInterrupt(2); /* Hz */
}

void initializeUSART2()
{
	USART2::enableClock();
//	USART2::unmaskInterrupts();
	USART2::configure(usart::cr1::rwu::RECEIVER_IN_ACTIVE_MODE,
    usart::cr1::re::RECEIVER_ENABLED,
    usart::cr1::te::TRANSMITTER_ENABLED,
    usart::cr1::idleie::IDLE_INTERRUPT_DISABLED,
    usart::cr1::rxneie::RXNE_ORE_INTERRUPT_DISABLED,
    usart::cr1::tcie::TC_INTERRUPT_DISABLED, // rs485 only
    usart::cr1::txeie::TXEIE_INTERRUPT_DISABLED,
    usart::cr1::peie::PEIE_INTERRUPT_DISABLED,
    usart::cr1::ps::EVEN_PARITY,
    usart::cr1::pce::PARITY_CONTROL_DISABLED,
    usart::cr1::wake::WAKE_ON_IDLE_LINE,
    usart::cr1::m::START_8_DATA_N_STOP,
    usart::cr1::ue::USART_ENABLED,
    usart::cr1::over8::OVERSAMPLING_BY_16,
    usart::cr2::stop::_1_STOP_BIT,
    usart::cr3::eie::ERROR_INTERRUPT_DISABLED,
    usart::cr3::hdsel::FULL_DUPLEX,
    usart::cr3::dmar::RECEIVER_DMA_DISABLED,
    usart::cr3::dmat::TRANSMITTER_DMA_DISABLED,
    usart::cr3::rtse::RTS_HARDWARE_FLOW_DISABLED,
    usart::cr3::ctse::CTS_HARDWARE_FLOW_DISABLED,
    usart::cr3::ctsie::CTS_INTERRUPT_DISABLED,
    usart::cr3::onebit::ONE_SAMPLE_BIT_METHOD);

	USART2::setBaudRate<
	  	  115200 /* bps */
	  >();
}

void initializePeripheral()
{
	  initializeUSART2();

	  RTC::initialize();
}

void initializeGpio()
{
  GPIOD::enableClock();

  LD3::setMode(gpio::moder::OUTPUT);
  LD4::setMode(gpio::moder::OUTPUT);
  LD6::setMode(gpio::moder::OUTPUT);
  LD5::setMode(gpio::moder::OUTPUT);

  GPIOA::enableClock();

  U2TXPIN::setAlternateFunction(gpio::afr::USART1_3);
  U2TXPIN::setMode(gpio::moder::ALTERNATE);
  U2RXPIN::setAlternateFunction(gpio::afr::USART1_3);
  U2RXPIN::setMode(gpio::moder::ALTERNATE);
}

void delay(u32 time)
{
	while (tick < time);
}

void loop()
{
  LD3::setLow();
  LD4::setHigh();
  LD5::setLow();

  delay(2);
  tick = 0;

  LD3::setHigh();
  LD4::setLow();
  LD5::setHigh();

  delay(2);
  tick = 0;
}



int main()
{
	RTCtime_t time;
	RTCdate_t date;

  clk::initialize();

  initializeGpio();

  initializePeripheral();

  initializeSystemTimer();

  time.hour = 18;
  time.min = 0;
  time.sec = 0;
  RTC::setTime(time);

  date.year = 13;
  date.month = rtcmonth::OCTOBER;
  date.mday = rtcday::MONDAY;
  RTC::setDate(date);

  while (true) {

    RTC::getTime(&time);
    RTC::getDate(&date);

		printf("Time is: %2d:%2d:%2d\n\r",time.hour, time.min, time.sec);
		printf("Date is: 20%d:%2d:%2d\n\r",date.year, date.month, date.mday);

    loop();
  }
}

/* SysTick exception is automatically cleared */
void exception::SysTick()
{
	tick++;

	if( LD6::isHigh() )
		LD6::setLow();
	else
		LD6::setHigh();
}
