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

/*******************************************************************************
 *
 *                                System Timer
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"
#include "../defs.hpp"

#include "../../memorymap/stk.hpp"

// Low-level access to the registers
#define _STK reinterpret_cast<stk::Registers*>(stk::ADDRESS)

// High-level functions
namespace stk {
  class Functions {
    public:
	  enum {
		  TICKERCLOCK = clk::SYSTICK,
		  CORECLOCK = clk::AHB
	  };
      // T ODO STK functions declaration
	  static inline void enableCounter();
	  static inline void disableCounter();
	  static inline void enableInterrupt();
	  static inline void disableInterrupt();
	  static inline void selectClockAHB();
	  static inline void selectClockDiv8();
	  static inline bool getClockSource();
	  static inline bool getCountFlag();
	  static inline void setTicks(u32 const);
	  static inline u32 getCurrentValue();
	  static inline void clearCurrentValue();
	  static inline void set10msReload(u32 const);
	  static inline bool isTenmsExact();
	  static inline bool isRefClockProvided();
//	  static inline void set1MsTicker();

	  static inline void configurePeriodicInterrupt(u32 const);

	  static inline u32 getAutoReloadValue();
	  static inline void reloadSysTick(u32 const);
    private:
      Functions();
  };
}  // namespace stk

// High-level access to the peripheral
// T ODO STK high-level access
typedef stk::Functions STK;

#include "../../bits/stk.tcc"
