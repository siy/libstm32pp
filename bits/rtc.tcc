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

#pragma once

#include "../include/bitband.hpp"
#include "../include/peripheral/pwr.hpp"
#include "../include/peripheral/rtc.hpp"
#include "../include/peripheral/exti.hpp"
#include "../include/core/nvic.hpp"

namespace rtc {
	/**
	 * @brief Disable write to RTC.
	 */

	void Functions::wplock()
	{
		RTC_REGS->WPR = 0xFF; // Any value would write protect the RTC
	}

	/**
	 * @brief Enables write to RTC
	 */
	void Functions::wpunlock()
	{
		RTC_REGS->WPR = 0xCA;
		RTC_REGS->WPR = 0x53;
	}

	/**
	 * @brief enable rtc interrupts
	 */
	template<u8 OFFSET, u8 BIT>
	void Functions::enableInterrupt()
	{
		wpunlock();

		*(bool volatile*) (bitband::peripheral<
				ADDRESS + OFFSET,
				BIT
		>()) = 1;

		wplock();
	}
	/**
	 * @brief disable rtc interrupts
	 */
	template<u8 OFFSET, u8 BIT>
	void Functions::disableInterrupt()
	{
		wpunlock();

		*(bool volatile*) (bitband::peripheral<
				ADDRESS + OFFSET,
				BIT
		>()) = 0;

		wplock();
	}

	/**
	 * @brief Calendar value are taken directly from counter
	 */
	void Functions::takeValueFromCounter()
	{
		wpunlock();

		*(bool volatile*) (bitband::peripheral<
				ADDRESS + cr::OFFSET,
				cr::bypshad::POSITION
		>()) = 1;

		wplock();
	}
	/**
	 * @brief Calendar value are taken from shadow register
	 */
	void Functions::takeValueFromShadow()
	{
		wpunlock();

		*(bool volatile*) (bitband::peripheral<
				ADDRESS + cr::OFFSET,
				cr::bypshad::POSITION
		>()) = 0;

		wplock();
	}
	/**
	 * @brief Get Bypass Shadow register
	 * @returns true if calendar values are directly taken from counter,
	 * 		    false if values are taken from the shadow register.
	 */

	bool Functions::isValueFromCounter()
	{
		return *(bool volatile*) (bitband::peripheral<
				ADDRESS + cr::OFFSET,
				cr::bypshad::POSITION
		>());
	}

	/**
	 * @brief Get time format.
	 * @returns true if 12Hours, false if 24Hours format.
	 */
	bool Functions::getTimeFormat()
	{
		return *(bool volatile*) (bitband::peripheral<
				ADDRESS + cr::OFFSET,
				cr::fmt::POSITION
				>());
	}
	/**
	 * @brief Set time format to 12/24 Hours
	 */
	void Functions::setTimeFormat(cr::fmt::format FORMAT)
	{
		wpunlock();

		*(u32 volatile*) (bitband::peripheral<
				ADDRESS + cr::OFFSET,
				cr::fmt::POSITION
				>()) = FORMAT;

		wplock();
	}

	/**
	 *@brief Set the daylight saving time.
	 */
	void Functions::setDST(cr::dst::States DST)
	{
		RTC_REGS->CR &= ~cr::dst::MASK;
		RTC_REGS->CR |= DST;
	}

	/**
	 * @brief Clear Initialization and Status flags
	 */
	template<isr::Flags F>
	void Functions::clearStatusFlag()
	{
	    *(bool volatile*) (bitband::peripheral<
	        ADDRESS + isr::OFFSET,
	        isr::POSITION + F
	    >()) = 0;
	}
	/**
	 * @brief Check Initialization and Status flags
	 */
	template<isr::Flags F>
	bool Functions::getStatus()
	{
	    return *(bool volatile*) (bitband::peripheral<
	        ADDRESS + isr::OFFSET,
	        isr::POSITION + F
	    >());
	}

	/**
	 * @brief Enter initialization mode
	 */
	void Functions::enterinit()
	{
		/* Check if the Initialization mode is already set */
		if (getStatus<isr::Flags::INITIALIZATION_MODE>() == false) {

			/* Set the Initialization mode */
			*(bool volatile*) (bitband::peripheral<
					ADDRESS + isr::OFFSET,
					isr::INITIALIZATION_MODE
			>()) = 1;

			/* Wait until initialization flag is set */
			for (u32 timeout = 0; timeout < INIT_TIMEOUT; timeout++)
			{
				if (getStatus<isr::Flags::INITIALIZATION_FLAG>() == true)
						break;

			}
		}
	}

	/**
	 * @brief Exit initialization mode
	 */
	void Functions::exitinit()
	{
		*(bool volatile*) (bitband::peripheral<
				ADDRESS + isr::OFFSET,
				isr::INITIALIZATION_MODE
		>()) = 0;
	}

	/**
	 * @brief Wait for Time and Date registers to sync.
	 */
	void Functions::syncwait()
	{
//		wpunlock();

	    /* Clear the synchronization flag */
		clearStatusFlag<isr::Flags::REGISTERS_SYNC_FLAG>();

	    /* Wait for sync */
		for (u32 timeout = 0; timeout < SYNC_TIMEOUT; timeout++)
		{
			if (getStatus<isr::Flags::REGISTERS_SYNC_FLAG>() == true)
					break;	/* Synchronized */
		}

//	    wplock();

	}

	/**
	 * @brief
	 */
	void Functions::resume()
	{

//		wpunlock(); // ISR Bits 8:13 does not need unlocking

		/* Clear RTC interrupt flags */
		clearStatusFlag<isr::Flags::ALARM_A_FLAG>();
		clearStatusFlag<isr::Flags::ALARM_B_FLAG>();
		syncwait();

//		wplock();

		/* External interrupt 17 is connected internally with RTC alarm */
		EXTI17::clearPendingFlag();
	}
	/**
	 * @brief Configure RTC
	 * @note Values configured here will be the default
	 * 		 with subsequent reset/power up until access
	 * 		 to Backup Domain Control is enabled again.
	 * 		 And a call to this function starts the vicious cycle :)
	 */
	void Functions::setup()
	{
		RTCtime_t tm;
		RTCdate_t dt;
		/* clock.tcc took care of the source and enabling of RTC */

		wpunlock();

		enterinit();

		/* Configure prescaler for external 32.768Khz clock */
		/* 2 separate write is required for PRER register */
		RTC_REGS->PRER = ((u32)0xFF << prer::predivs::POSITION);
		RTC_REGS->PRER |= ((u32)0x7F << prer::prediva::POSITION);

		/* Set initial time/date value */
		tm.hour = 00;
		tm.min = 36;
		tm.sec = 30;
		dt.year = 13;
		dt.month = rtcmonth::SEPTEMBER;
		dt.mday = 22;
		dt.wday = rtcday::SUNDAY;
		setTime(tm);
		setDate(dt);
		setTimeFormat(cr::fmt::format::_24_HOURS);

		exitinit();
		resume();

		wplock();



	}

	/**
	 * @brief Configure RTC.
	 */
	void Functions::initialize()
	{
		u32 regval;

		/* Access to RTC backup domain is taken care of by clock initialization clock.tcc */

		/* Check if RTC has been initialized */
		regval = RTC_REGS->BKPR[0];

		if (regval != rtc::RTC_MAGIC) {
			/* if not then go thru initialization */
			setup();

			/* RTC should be  intialized */
			RTC_REGS->BKPR[0] = rtc::RTC_MAGIC;
		} else {
			resume();
		}
	}

	/**
	 * @brief Convert binary to bcd format
	 */
	u8 Functions::bin2bcd(u8 value)
	{
		u8 bcd=0;

		while (value >= 10) {
			bcd++;
			value -= 10;
		}
		return ((u8)(bcd << 4) | value);
	}

	/**
	 * @brief Convert bcd to binary format
	 */
	u8 Functions::bcd2bin(u8 value) {
		u8 tmp = ((u8)(value & 0xF0) >> 4) * 10;
		return (u8)(tmp + (value & 0x0f));
	}

	/**
	 * @brief Configure the time
	 */
	void Functions::setTime(RTCtime_t time)
	{
		u32 tr;

		tr = (bin2bcd(time.sec) << tr::su::POSITION) |
			 (bin2bcd(time.min) << tr::mnu::POSITION) |
			 (bin2bcd(time.hour) <<	tr::hu::POSITION);
		tr &= tr::TR_MASK;

		wpunlock();

		enterinit();
		RTC_REGS->TR = tr;
		exitinit();

		/* If shadow is the source for calendar values */
		if (isValueFromCounter() == false) {
			/* wait to synchronize */
			syncwait();
		}

		wplock();
	}
	/**
	 * @brief Configure the date
	 */
	void Functions::setDate(RTCdate_t date)
	{
		u32 dr;

		dr = (bin2bcd(date.wday) << dr::wdu::POSITION)|
			 (bin2bcd(date.mday) << dr::du::POSITION) |
			 (bin2bcd(date.month) << dr::mu::POSITION)|
			 (bin2bcd(date.year) << dr::yu::POSITION);
		dr &= dr::DR_MASK;

		wpunlock();

		enterinit();
		RTC_REGS->DR = dr;
		exitinit();

		/* If shadow is the source for calendar values */
		if (isValueFromCounter() == false) {
			/* wait to synchronize */
			syncwait();
		}

		wplock();
	}
	/**
	 * @brief Get the time and convert from bcd to byte
	 */
	void Functions::getTime(RTCtime_t *time)
	{
		u32 tr;
		u32 tmp;

		do {
			tmp = RTC_REGS->TR;
			tr = RTC_REGS->TR;
		}
		while (tr != tmp);

		tr &= tr::TR_MASK;

		tmp = (u8)((tr & (tr::su::MASK|tr::st::MASK)) >> tr::su::POSITION);
		time->sec = bcd2bin(tmp);

		tmp = (u8)((tr & (tr::mnu::MASK|tr::mnt::MASK)) >> tr::mnu::POSITION);
		time->min = bcd2bin(tmp);

		tmp = (u8)((tr & (tr::hu::MASK|tr::ht::MASK)) >> tr::hu::POSITION);
		time->hour = bcd2bin(tmp);

		resume();
	}
	/**
	 * @brief Get the date and convert from bcd to byte
	 */
	void Functions::getDate(RTCdate_t *date)
	{
		u32 dr;
		u32 tmp;

		do {
			tmp = RTC_REGS->DR;
			dr = RTC_REGS->DR;
		}
		while (dr != tmp);

		dr &= dr::DR_MASK;

		tmp = (u8)((dr & (dr::du::MASK|dr::dt::MASK)) >> dr::du::POSITION);
		date->mday = bcd2bin(tmp);

		tmp = (u8)((dr & (dr::mu::MASK|dr::mt::MASK)) >> dr::mu::POSITION);
		date->month = bcd2bin(tmp);

		tmp = (u8)((dr & (dr::yu::MASK|dr::yt::MASK)) >> dr::yu::POSITION);
		date->year = bcd2bin(tmp);

		tmp = (u8)((dr & dr::wdu::MASK) >> dr::wdu::POSITION);
		date->wday = bcd2bin(tmp);

		resume();
	}
}// namespace rtc
