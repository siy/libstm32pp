/*******************************************************************************
 *
 * Copyright (C) 2012 Jorge Aparicio <jorge.aparicio.r@gmail.com>
 * Register bits addition (C)2013 Rommel Marcelo <jaqueza@gmail.com>
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

#include "common.hpp"

namespace stk {
  enum {
    ADDRESS = alias::PPB + 0x10
  };

  struct Registers {
      __RW
      u32 CTRL;  // 0x00: Control and status
      __RW
      u32 LOAD;  // 0x04: Reload value
      __RW
      u32 VAL;  // 0x08: Current value
      __RW
      u32 CALIB;  // 0x0C: Calibration value
  };

// T ODO STK register bits
  /* Systick Control and Status Register */
  namespace stkctrl {
  	  enum {
  		  OFFSET = 0x00
  	  };
  	  /* Enables Counter */
  	  namespace cntenable {
  	  	  enum {
  	  		  POSITION = 0,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  	  enum States {
  	  		  COUNTER_DISABLED = 0 << POSITION,
  	  		  COUNTER_ENABLED = 1 << POSITION
  	  	  };
  	  }
  	  /* Enables SysTick IRQ */
  	  namespace tickint {
  	  	  enum {
  	  		  POSITION = 1,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  	  enum States {
  	  		  DISABLE_IRQ = 0 << POSITION,
  	  		  ENABLE_IRQ = 1 << POSITION
  	  	  };
  	  }
  	  /* Clock Source */
  	  namespace clksource {
  	  	  enum {
  	  		  POSITION = 2,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  	  enum Source {
  	  		  AHB_CLOCK_DIV8 = 0 << POSITION,
  	  		  AHB_CLOCK = 1 << POSITION
  	  	  };
  	  }
  	  /* Count access flag */
  	  namespace countflag {
  	  	  enum {
  	  		  POSITION = 16,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  	  enum States {
  	  		  LASTREAD_NOT_DOWN_TO_ZERO = 0 << POSITION,
  	  		  LASTREAD_DOWN_TO_ZERO = 1 << POSITION
  	  	  };
  	  }
  } // namespace csr

  /* Systick Reload Value Register */
  namespace stkload {
  	  enum {
  		  OFFSET = 0x04
  	  };
  	  /* Reload Value */
  	  namespace reload {
  	  	  enum {
  	  		POSITION = 0,
  	  		MASK = 0xFFFFFF
  	  	  };
  	  }
  } //namespace load

  /* Systick Current Value Register */
  namespace stkval {
  	  enum {
  		  OFFSET = 0x08
  	  };
  	  /* Current Systick Value */
  	  namespace current {
  	  	  enum {
  	  		POSITION = 0,
  	  		MASK = 0xFFFFFF << POSITION
  	  	  };
  	  }
  } // namespace val

  /* Systick Calibration Value Register */
  namespace stkcalib {
  	  enum {
  		  OFFSET = 0x0C
  	  };
  	  /* 10ms Reload Value */
  	  namespace tenms {
  	  	  enum {
  	  		  POSITION = 0,
  	  		  MASK = 0xFFFFFF << POSITION
  	  	  };
  	  }
  	  /* Indicates TENMS is exact */
  	  namespace skew {
  	  	  enum {
  	  		  POSITION = 30,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  	  enum States {
  	  		  TENMS_VALUE_EXACT = 0 << POSITION,
  	  		  TENMS_VALUE_NOT_EXACT = 1 << POSITION
  	  	  };
  	  }
  	  /* Indicates device provides processor clock reference */
  	  namespace noref{
  	  	  enum {
  	  		  POSITION = 31,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  	  enum States {
  	  		  REFERENCE_CLOCK_PROVIDED = 0 << POSITION,
  	  		  NO_REFERENCE_CLOCK = 1 << POSITION
  	  	  };
  	  }
  }
}// namespace stk

