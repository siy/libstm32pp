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

namespace stk {
// STK functions implementation
	/**
	 * @brief Enable the counter.
	 * @note
	 */
	void Functions::enableCounter()
	{
		_STK->CTRL |= stkctrl::cntenable::States::COUNTER_ENABLED;
	}
	/**
	 * @brief Disable the counter.
	 * @note
	 */
	void Functions::disableCounter()
	{
		_STK->CTRL &= ~stkctrl::cntenable::MASK;
	}
	/**
	 * @brief Enable interrupt request.
	 * @note
	 */
	void Functions::enableInterrupt()
	{
		_STK->CTRL |= stkctrl::tickint::ENABLE_IRQ;
	}
	/**
	 * @brief Disable  interrupt request.
	 * @note
	 */
	void Functions::disableInterrupt()
	{
		_STK->CTRL &= ~stkctrl::tickint::MASK;
	}
	/**
	 * @brief Check the clock source.
	 * @note
	 */
	bool Functions::getClockSource()
	{
	  return _STK->CTRL & stkctrl::clksource::MASK;
	}
	/**
	 * @brief Select internal clock as source.
	 * @note
	 */
	void Functions::selectClockAHB()
	{
	  _STK->CTRL |= stkctrl::clksource::AHB_CLOCK;
	}
	  /**
	   * @brief Select External clock as source.
	   * @note
	   */
	void Functions::selectClockDiv8()
	{
		_STK->CTRL &= ~stkctrl::clksource::MASK;
	}
	/**
	 * @brief Returns TRUE if timer counted to zero since last read.
	 * @note
	 */
	bool Functions::getCountFlag()
	{
	  return _STK->CTRL & stkctrl::countflag::MASK;
	}

	/**
	 * @brief Sets the reload value of the counter.
	 * @note  Range: 1-0xFFFFFF. A value of 0 has no effects.
	 */
	void Functions::setTicks(u32 const ticks)
	{
		if(ticks > stkload::reload::MASK) return;//TODO
		_STK->LOAD = (ticks & stkload::reload::MASK) - 1;
	}
	/**
	 * @brief Set system ticker to tick at 1 millisecond.
	 * @note
	 */
//	void Functions::set1MsTicker()
//	{
//		if(getClockSource() == 1)
//		_STK->LOAD = CORECLOCK / 1000UL;
//		else
//		_STK->LOAD = TICKERCLOCK / 1000UL;
//	}
	/**
	 * @brief Clears the current value of the counter.
	 */
	void Functions::clearCurrentValue()
	{
		_STK->VAL = 0;
	}
	/**
	 * @brief Returns the current value of the counter.
	 */
	u32 Functions::getCurrentValue()
	{
		return _STK->VAL;
	}

	/**
	 * @brief Sets the 10ms reload value of the 10ms timing.
	 * @note  Range: 1-0xFFFFFF. Depends on SKEW.
	 */
	void Functions::set10msReload(u32 const val)
	{
		_STK->CALIB = val & stkcalib::tenms::MASK;
	}

	/**
	 * @brief Checks if the TENMS value is exact.
	 * @note
	 */
	bool Functions::isTenmsExact()
	{
		return _STK->CALIB & stkcalib::skew::MASK;
	}

	/**
	 * @brief Check if reference clock is provided for calibration.
	 * @note
	 */
	bool Functions::isRefClockProvided()
	{
	  return _STK->CALIB & stkcalib::noref::MASK;
	}
	/**
	 * @brief Get the current auto-reload value of the counter.
	 * @note  A value of 0 blocks the counter.
	 */

	u32 Functions::getAutoReloadValue()
	{
		return _STK->LOAD + 1;
	}

	/**
	 * @brief Sets the auto-reload value of the counter.
	 * @note  A value of 0 blocks the counter.
	 */

	void Functions::reloadSysTick(u32 const rld)
	{
		//TODO (rld > 0xFFFFFF, "Value exceeds the range of the RELOAD register");
		_STK->LOAD = rld;
	}

	/**
	 * @brief Configure System Timer for interrupt at frequency (Hz)
	 * @note TICKERCLOCK = Systick clock = (AHB clock/8)
	 */

	void Functions::configurePeriodicInterrupt(u32 Frequency)
	{
		reloadSysTick(
				(TICKERCLOCK / Frequency < 16777215 ?
						TICKERCLOCK / Frequency :
						(TICKERCLOCK / (10 * Frequency) < 0xFFFFFF ?
								TICKERCLOCK / (10 * Frequency) :
								(TICKERCLOCK / (100 * Frequency) < 0xFFFFFF ?
										TICKERCLOCK / (100 * Frequency) :
										(TICKERCLOCK / (1000 * Frequency) < 0xFFFFFF ?
												TICKERCLOCK / (1000 * Frequency) :
												(TICKERCLOCK / (10000 * Frequency) < 0xFFFFFF ?
														TICKERCLOCK / (10000 * Frequency) :
														0))))) - 1);
		clearCurrentValue();
		selectClockDiv8();
		enableInterrupt();
		enableCounter();
	}

}// namespace stk
