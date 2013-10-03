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

#include "common.hpp"

namespace rtc {

		enum {
			ADDRESS = alias::APB1 + 0x2800
		};

	struct Registers {

#ifdef STM32F1XX
      __RW
      u32 CR[2];   // 0x00, 0x04: Control
      __RW
      u32 PRL[2];// 0x08, 0x1C: Prescaler load
      __RW
      u32 DIV[2];// 0x10, 0x14: Prescaler divider
      __RW
      u32 CNT[2];// 0x18, 0x1C: Counter
      __RW
      u32 ALR[2];// 0x20, 0x24: Alarm
#elif defined STM32F2XX
      __RW
      u32 TR;  // 0x00: Time
      __RW
      u32 DR;// 0x04: Date
      __RW
      u32 CR;// 0x08: Control
      __RW
      u32 ISR;// 0x0C: Initialization and status
      __RW
      u32 PRER;// 0x10: Prescaler
      __RW
      u32 WUTR;// 0x14: Wakeup timer
      __RW
      u32 CALIBR;// 0x18: Calibration
      __RW
      u32 ALRMR[2];// 0x1C, 0x20: Alarm
      __RW
      u32 WPR;// 0x24: Write protection
      u32 _RESERVED0[2];
      __RW
      u32 TSTR;// 0x30: Time stamp time
      __RW
      u32 TSDR;// 0x34: Time stamp date
      u32 _RESERVED1[2];
      __RW
      u32 TAFCR;// 0x40: Tamper and alternate function configuration
      u32 _RESERVED2[3];
      __RW
      u32 BKPR[20];// 0x50-0x9C: Backup
#else
      __RW
      u32 TR;  // 0x00: Time
      __RW
      u32 DR;  // 0x04: Date
      __RW
      u32 CR;  // 0x08: Control
      __RW
      u32 ISR;  // 0x0C: Initialization and status
      __RW
      u32 PRER;  // 0x10: Prescaler
      __RW
      u32 WUTR;  // 0x14: Wakeup timer
      __RW
      u32 CALIBR;  // 0x18: Calibration
      __RW
      u32 ALRMR[2];  // 0x1C, 0x20: Alarm
      __RW
      u32 WPR;  // 0x24: Write protection
      __RW
      u32 SSR;  // 0x28: Sub second
      __RW
      u32 SHIFTR;  // 0x2C: Shift control
      __RW
      u32 TSTR;  // 0x30: Time stamp time
      __RW
      u32 TSDR;  // 0x34: Time stamp date
      __RW
      u32 TSSSR;  // 0x38: Time stamp sub second
      __RW
      u32 CALR;  // 0x3C: Calibration
      __RW
      u32 TAFCR;  // 0x40: Tamper and alternate function configuration
      __RW
      u32 ALRMSSR[2];  // 0x44, 0x48: Alarm sub second
      u32 RESERVED0;
      __RW
      u32 BKPR[20];  // 0x50-0x9C: Backup
#endif
  };

// TODO RTC register bits

  /* RTC Time Register */

  namespace tr {
    enum {
      OFFSET = 0x00,
//      TR_MASK = 0xFF808080
      TR_MASK = 0x007F7F7F
    };

    /* Seconds Units BCD */
    namespace su {
      enum {
        POSITION = 0,
        MASK = 15 << POSITION
      };
    }

    /* Seconds Tens BCD */
    namespace st {
      enum {
        POSITION = 4,
        MASK = 7 << POSITION
      };
    }

    /* Minute Units BCD */
    namespace mnu {
      enum {
        POSITION = 8,
        MASK = 15 << POSITION
      };
    }

    /* Minute Tens BCD */
    namespace mnt {
      enum {
        POSITION = 12,
        MASK = 7 << POSITION
      };
    }

    /* Hour Units BCD */
    namespace hu {
      enum {
        POSITION = 16,
        MASK = 15 << POSITION
      };
    }

    /* Hour Tens BCD */
    namespace ht {
      enum {
        POSITION = 20,
        MASK = 3 << POSITION
      };
    }

    /* AM/PM Notation */
    namespace pm {
      enum {
        POSITION = 22,
        MASK = 1 << POSITION
      };
      enum TimePeriod {
        AM = 0 << POSITION, // or 24 hours format
        PM = 1 << POSITION
      };
    }

  }  // namespace tr

  /* RTC Date Registers */

  namespace dr {
    enum {
      OFFSET = 0x04,
//      DR_MASK = 0xFF0000C0
      DR_MASK = 0x00FFFF3F
    };

    /* Date Units BCD */
    namespace du {
		enum {
			POSITION = 0,
			MASK = 15 << POSITION
		};
    }

    /* Date Tens BCD */
    namespace dt {
		enum {
			POSITION = 4,
			MASK = 3 << POSITION
		};
    }

    /* Months Units BCD */
    namespace mu {
		enum {
			POSITION = 8,
			MASK = 15 << POSITION
		};
    }

    /* Months Tens BCD */
    namespace mt {
		enum {
			POSITION = 12,
			MASK = 1 << POSITION
		};
    }

    /* Weekday Units */
    namespace wdu {
    	enum {
    		POSITION = 13,
    		MASK = 7 << POSITION
    	};

    	enum DayOfWeek {
    		// 0 = reserved
    		MONDAY 		= 1 << POSITION,
    		TUESDAY 	= 2 << POSITION,
    		WEDNESDAY 	= 3 << POSITION,
    		THURSDAY 	= 4 << POSITION,
    		FRIDAY 		= 5 << POSITION,
    		SATURDAY 	= 6 << POSITION,
    		SUNDAY 		= 7 << POSITION
    	};

    }

    /* Year Units BCD */
    namespace yu {
		enum {
			POSITION = 16,
			MASK = 15 << POSITION
		};
    }

    /* Year Tens BCD */
    namespace yt {
		enum {
			POSITION = 20,
			MASK = 15 << POSITION
		};
    }

  } // namespade dr

  /* RTC Control Register */
  namespace cr {
	  enum {
		OFFSET = 0x08
	  };

	  /* Wakeup Clock Selection */
	  namespace wucksel {
	  	  enum {
	  		  POSITION = 0,
	  		  MASK = 7 << POSITION
	  	  };
	  	  enum Selections {
	  		  RTC_DIV16 = 0 << POSITION,
	  		  RTC_DIV8	= 1 << POSITION,
	  		  RTC_DIV4	= 2 << POSITION,
	  		  RTC_DIV2	= 3 << POSITION,
	  		  RTC_DIV1	= 4 << POSITION,
	  		  RTC_DIV1_WUT	= 6 << POSITION
	  	  };
	  }

	  /* Timestamp Event Active Edge */
	  namespace tsedge {
	  	  enum {
	  		  POSITION = 3,
	  		  MASK = 1 << POSITION
	  	  };
	  	  enum States {
	  		  RISING_EDGE = 0 << POSITION,
	  		  FALLING_EDGE = 1 << POSITION
	  	  };
	  }

	  /* Reference Clock Detection Enable */
	  namespace refckon {
	  	  enum {
	  		  POSITION = 4,
	  		  MASK = 1 << POSITION
	  	  };
	  	  enum States {
	  		  DISABLE_DECTECTION = 0 << POSITION,
	  		  ENABLE_DECTECTION = 1 << POSITION
	  	  };
	  }

	  /* Bypass Shadow Register */
	  namespace bypshad {
	  	  enum {
	  		  POSITION = 5,
	  		  MASK = 1 << POSITION
	  	  };
	  	  enum States {
	  		  VALUES_FROM_SHADOW_REG = 0,
	  		  VALUES_FROM_COUNTERS	= 1
	  	  };
	  }

	  /* Hours Format */
	  namespace fmt {
	  	  enum {
	  		  POSITION = 6,
	  		  MASK = 1 << POSITION
	  	  };
	  	  enum format {
	  		  _24_HOURS = 0,
	  		  _12_HOURS = 1
	  	  };
	  }

	  /* Coarse Digital Calibration Enable */
	  namespace dce {
	  	  enum {
	  		  POSITION = 7,
	  		  MASK = 1 << POSITION
	  	  };
	  	  enum States {
	  		  DISABLE_CALIBRATION = 0 << POSITION,
	  		  ENABLE_CALIBRATION = 1 << POSITION
	  	  };
	  }

	  /* Alarm A Enable */
	  namespace alrae {
	  	  enum {
	  		  POSITION = 8,
	  		  MASK = 1 << POSITION
	  	  };
	  	  enum States {
	  		  ALARM_A_DISABLED = 0 << POSITION,
	  		  ALARM_A_ENABLED = 1 << POSITION
	  	  };
	  }

	  /* Alarm B Enable */
	  namespace alrbe {
	  	  enum {
	  		  POSITION = 9,
	  		  MASK = 1 << POSITION
	  	  };
	  	  enum States {
	  		  ALARM_B_DISABLED = 0 << POSITION,
	  		  ALARM_B_ENABLED = 1 << POSITION
	  	  };
	  }

	  /* WakeUp Timer Enable */
	  namespace wute {
	  	  enum {
	  		  POSITION = 10,
	  		  MASK = 1 << POSITION
	  	  };
	  	  enum States {
	  		  WAKEUP_TIMER_DISABLED = 0 << POSITION,
	  		  WAKEUP_TIMER_ENABLED = 1 << POSITION
	  	  };
	  }

	  /* Timestamp Enable */
	  namespace tse {
	  	  enum {
	  		  POSITION = 11,
	  		  MASK = 1 << POSITION
	  	  };
	  	  enum States {
	  		  TIMESTAMP_DISABLED = 0 << POSITION,
	  		  TIMESTAMP_ENABLED = 1 << POSITION
	  	  };
	  }

	  /* Interrupt enable/disable bits */
	  namespace rtcirq {
	  	  enum States {
	  		  INTERRUPT_DISABLED = 0,
	  		  INTERRUPT_ENABLED = 1
	  	  };

	  	 enum interrupt {
	  		 ALARM_A = 12,
	  		 ALARM_B = 13,
	  		 WAKEUP_TIMER = 14,
	  		 TIMESTAMP = 15,
	  		 ALARM_A_MASK = 1 << ALARM_A,
	  		 ALARM_B_MASK = 1 << ALARM_B,
	  		 WAKEUP_TIMER_MASK = 1 << WAKEUP_TIMER,
	  		 TIMESTAMP_MASK = 1 << TIMESTAMP
	  	 };

//	  	 	  /* Alarm A Interrupt Enable */
//	  	  namespace alraie {
//	  	  	  enum {
//	  	  		  POSITION = 12,
//	  	  		  MASK = 1 << POSITION
//	  	  	  };
//	  	  }
//	  	  /* Alarm B Interrupt Enable */
//	  	  namespace alrbie {
//	  	  	  enum {
//	  	  		  POSITION = 13,
//	  	  		  MASK = 1 << POSITION
//	  	  	  };
//	  	  }
//	  	  /* WakeUp Timer Interrupt Enable */
//	  	  namespace wutie {
//	  	  	  enum {
//	  	  		  POSITION = 14,
//	  	  		  MASK = 1 << POSITION
//	  	  	  };
//	  	  }
//	  	  /* TimeStamp Interrupt Enable */
//	  	  namespace tsie {
//	  	  	  enum {
//	  	  		  POSITION = 15,
//	  	  		  MASK = 1 << POSITION
//	  	  	  };
//	  	  }
	  } //namespace rtcirq

	  /* Daylight Saving Time */
	  namespace dst {
		  enum {
			  POSITION = 16,
			  MASK = 3 << POSITION
		  };
		  enum States {
			  SUMMER_DST = 1 << POSITION,
			  WINTER_DST = 2 << POSITION
		  };
	  }
//	  namespace add1h {
//	  	  enum {
//	  		  POSITION = 16,
//	  		  MASK = 1 << POSITION
//	  	  };
//	  	  enum States {
//	  		  ADD_1_HOUR = 1 << POSITION
//	  	  };
//	  }
//
//	  /* Standard Time */
//	  namespace sub1h {
//	  	  enum {
//	  		  POSITION = 17,
//	  		  MASK = 1 << POSITION
//	  	  };
//	  	  enum States {
//	  		  SUBTRACT_1_HOUR = 1 << POSITION
//	  	  };
//	  }

	  /* Back Up */
	  namespace bkp {
	  	  enum {
	  		  POSITION = 18,
	  		  MASK = 1 << POSITION
	  	  };
	  	  enum States {
	  		  DST_UNCHANGED = 0 << POSITION,
	  		  DST_CHANGED = 1 << POSITION
	  	  };
	  }

	  /* Calibration Output Selection */
	  namespace cosel {
	  	  enum {
	  		  POSITION = 19,
	  		  MASK = 1 << POSITION
	  	  };
	  	  enum States {
	  		  CALIBRATION_512_HZ = 0 << POSITION,
	  		  CALIBRATION_1_HZ = 1 << POSITION
	  	  };
	  }

	  /* Output Polarity */
	  namespace pol {
	  	  enum {
	  		  POSITION = 20,
	  		  MASK = 1 << POSITION
	  	  };
	  	  enum States {
	  		  PIN_IS_HIGH = 0 << POSITION,
	  		  PIN_IS_LOW = 1 << POSITION
	  	  };
	  }

	  /* Output Selection */
	  namespace osel {
	  	  enum {
	  		  POSITION = 21,
	  		  MASK = 3 << POSITION
	  	  };
	  	  enum States {
	  		  OUTPUT_DISABLED = 0 << POSITION,
	  		  ALARM_A_ENABLED = 1 << POSITION,
	  		  ALARM_B_ENABLED = 2 << POSITION,
	  		  WAKEUP_ENABLED = 3 << POSITION,
	  	  };
	  }

	  /* Calibration Output Enable */
	  namespace coe {
	  	  enum {
	  		  POSITION = 23,
	  		  MASK = 1 << POSITION
	  	  };
	  	  enum States {
	  		  CALIBRATION_OUTPUT_DISABLED = 0 << POSITION,
	  		  CALIBRATION_OUTPUT_ENABLED = 1 << POSITION
	  	  };
	  }

	  /* Bits 31:24 Reserved */
  } // namespace cr

  /* RTC Initialization and Status Register */
  namespace isr {
	  enum {
		OFFSET = 0x0C,
	  };
	  /* Status Flags */
	  enum {
		  POSITION = 0,
		  MASK = 0x00017FFF
	  };

	  enum Flags{
		  ALARM_A_WRITE_FLAG 		= 0 << POSITION,
		  ALARM_B_WRITE_FLAG 		= 1 << POSITION,
		  WAKEUPTIMER_WRITE_FLAG 	= 2 << POSITION,
		  SHIFT_OPERATION_FLAG 		= 3 << POSITION,
		  INIT_STATUS_FLAG			= 4 << POSITION,
		  REGISTERS_SYNC_FLAG		= 5 << POSITION,
		  INITIALIZATION_FLAG		= 6 << POSITION,
		  INITIALIZATION_MODE		= 7 << POSITION,
		  ALARM_A_FLAG				= 8 << POSITION,
		  ALARM_B_FLAG				= 9 << POSITION,
		  WAKEUP_TIMER_FLAG			= 10 << POSITION,
		  TIMESTAMP_FLAG			= 11 << POSITION,
		  TIMESTAMP_OVERFLOW_FLAG	= 12 << POSITION,
		  TAMPER1_DETECTION_FLAG	= 13 << POSITION,
		  TAMPER2_DETECTION_FLAG	= 14 << POSITION,
		  // RESERVED
		  RECALIBRATION_PENDING_FLAG= 16 << POSITION
		  // RESERVED
	  };
  } // namespace isr

  /* RTC Prescaler Register */
  namespace prer {
	  enum {
		OFFSET = 0x10
	  };
	  namespace predivs {
	  	  enum {
	  		  POSITION = 0,
	  		  MASK = 0x7FFF << POSITION
	  	  };
	  }
	  namespace prediva {
	  	  enum {
	  		  POSITION = 16,
	  		  MASK = 0x7F << POSITION
	  	  };
	  }
  } // namespace prer

  /* RTC Wakeup Timer Register */
  namespace wutr {
	  enum {
		OFFSET = 0x14
	  };
	  /* WakeUp Auto Reload */
	  namespace wut {
	  	  enum {
	  		  POSITION = 0,
	  		  MASK = 0x0000FFFF
	  	  };
	  }
  } // namespace wutr

  /* RTC Calibration Register */
  namespace calibr {
	  enum {
		OFFSET = 0x18
	  };
	  /* Digital Calibration */
	  namespace dc {
	  	  enum {
	  		  POSITION = 0,
	  		  MASK = 0x1F
	  	  };
	  	  enum Calibration {
	  		  //TODO
	  	  };
	  }
  } // namespace calibr

  /* RTC Alarm A Register */
  namespace alrmar {
  	  enum {
  		OFFSET = 0x1C
  	  };
  	  /* Second Units BCD */
  	  namespace su {
  	  	  enum {
  	  		  POSITION = 0,
  	  		  MASK = 0x0F
  	  	  };
  	  }
  	  /* Second Tens BCD */
  	  namespace st {
  	  	  enum {
  	  		  POSITION = 4,
  	  		  MASK = 0x07 << POSITION
  	  	  };
  	  }
  	  /* Alarm A Seconds Mask */
  	  namespace msk1 {
  	  	  enum {
  	  		  POSITION = 7,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  }
  	  /* Minute Units BCD */
  	  namespace mnu {
  	  	  enum {
  	  		  POSITION = 8,
  	  		  MASK = 0x0F << POSITION
  	  	  };
  	  }
  	  /* Minute Tens BCD */
  	  namespace mnt {
  	  	  enum {
  	  		  POSITION = 12,
  	  		  MASK = 0x07 << POSITION
  	  	  };
  	  }
  	  /* Alarm A Minutes Mask */
  	  namespace msk2 {
  	  	  enum {
  	  		  POSITION = 15,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  }
  	  /* Hours Units BCD */
  	  namespace hu {
  	  	  enum {
  	  		  POSITION = 16,
  	  		  MASK = 0x0F << POSITION
  	  	  };
  	  }
  	  /* Hours Tens BCD */
  	  namespace ht {
  	  	  enum {
  	  		  POSITION = 20,
  	  		  MASK = 0x03 << POSITION
  	  	  };
  	  }
  	  /* AM/PM Notation */
  	  namespace pm {
		  enum {
			POSITION = 22,
			MASK = 1 << POSITION
		  };
		  enum TimePeriod {
			AM = 0 << POSITION, // or 24 hours format
			PM = 1 << POSITION
		  };
  	  }
  	  /* Alarm A Hours Mask */
  	  namespace msk3 {
  	  	  enum {
  	  		  POSITION = 23,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  }
  	  /* Date Units or Day in BCD */
  	  namespace du {
  	  	  enum {
  	  		  POSITION = 24,
  	  		  MASK = 0x0F << POSITION
  	  	  };
  	  	  // If WDSEL = 1
  	  	  enum DayOfWeek {
      		// 0 = reserved
      		MONDAY 		= 1 << POSITION,
      		TUESDAY 	= 2 << POSITION,
      		WEDNESDAY 	= 3 << POSITION,
      		THURSDAY 	= 4 << POSITION,
      		FRIDAY 		= 5 << POSITION,
      		SATURDAY 	= 6 << POSITION,
      		SUNDAY 		= 7 << POSITION
  	  	  };
  	  }
  	  /* Date Tens BCD */
  	  namespace dt {
  	  	  enum {
  	  		  POSITION = 28,
  	  		  MASK = 0x03 << POSITION
  	  	  };
  	  }
  	  /* Week Day Selection */
  	  namespace wdsel {
  	  	  enum {
  	  		  POSITION = 30,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  }
  	  /* Alarm A Date Mask */
  	  namespace msk4 {
  	  	  enum {
  	  		  POSITION = 31,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  }
  } // namespace alrmar

  /* RTC Alarm B Register */
  namespace alrmbr {
  	  enum {
  		OFFSET = 0x20
  	  };
  	  /* Second Units BCD */
  	  namespace su {
  	  	  enum {
  	  		  POSITION = 0,
  	  		  MASK = 0x0F
  	  	  };
  	  }
  	  /* Second Tens BCD */
  	  namespace st {
  	  	  enum {
  	  		  POSITION = 4,
  	  		  MASK = 0x07 << POSITION
  	  	  };
  	  }
  	  /* Alarm B Seconds Mask */
  	  namespace msk1 {
  	  	  enum {
  	  		  POSITION = 7,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  }
  	  /* Minute Units BCD */
  	  namespace mnu {
  	  	  enum {
  	  		  POSITION = 8,
  	  		  MASK = 0x0F << POSITION
  	  	  };
  	  }
  	  /* Minute Tens BCD */
  	  namespace mnt {
  	  	  enum {
  	  		  POSITION = 12,
  	  		  MASK = 0x07 << POSITION
  	  	  };
  	  }
  	  /* Alarm B Minutes Mask */
  	  namespace msk2 {
  	  	  enum {
  	  		  POSITION = 15,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  }
  	  /* Hours Units BCD */
  	  namespace hu {
  	  	  enum {
  	  		  POSITION = 16,
  	  		  MASK = 0x0F << POSITION
  	  	  };
  	  }
  	  /* Hours Tens BCD */
  	  namespace ht {
  	  	  enum {
  	  		  POSITION = 20,
  	  		  MASK = 0x03 << POSITION
  	  	  };
  	  }
  	  /* AM/PM Notation */
  	  namespace pm {
		  enum {
			POSITION = 22,
			MASK = 1 << POSITION
		  };
		  enum TimePeriod {
			AM = 0 << POSITION, // or 24 hours format
			PM = 1 << POSITION
		  };
  	  }
  	  /* Alarm B Hours Mask */
  	  namespace msk3 {
  	  	  enum {
  	  		  POSITION = 23,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  }
  	  /* Date Units or Day in BCD */
  	  namespace du {
  	  	  enum {
  	  		  POSITION = 24,
  	  		  MASK = 0x0F << POSITION
  	  	  };
  	  	  // If WDSEL = 1
  	  	  enum Weekday {
      		// 0 = reserved
      		MONDAY 		= 1 << POSITION,
      		TUESDAY 	= 2 << POSITION,
      		WEDNESDAY 	= 3 << POSITION,
      		THURSDAY 	= 4 << POSITION,
      		FRIDAY 		= 5 << POSITION,
      		SATURDAY 	= 6 << POSITION,
      		SUNDAY 		= 7 << POSITION
  	  	  };
  	  }
  	  /* Date Tens BCD */
  	  namespace dt {
  	  	  enum {
  	  		  POSITION = 28,
  	  		  MASK = 0x03 << POSITION
  	  	  };
  	  }
  	  /* Week Day Selection */
  	  namespace wdsel {
  	  	  enum {
  	  		  POSITION = 30,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  }
  	  /* Alarm B Date Mask */
  	  namespace msk4 {
  	  	  enum {
  	  		  POSITION = 31,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  }
  } // namespace alrmbr

  /* RTC Write Protection Register */
  namespace wpr {
  	  enum {
  		OFFSET = 0x24
  	  };
  	  /* Write Protection Key */
  	  namespace key {
  	  	  enum {
  	  		  POSITION = 0,
  	  		  MASK = 0xFF
  	  	  };
  	  }
  } // namespace wpr

  /* RTC Sub SecondRegister */
  namespace ssr {
  	  enum {
  		OFFSET = 0x28
  	  };
  	  /* Sub Second Value */
  	  namespace ss {
  	  	  enum {
  	  		  POSITION = 0,
  	  		  MASK = 0xFFFF
  	  	  };
  	  }
  } // namespace ssr

  /* RTC Shift Control Register */
  namespace shiftr {
  	  enum {
  		OFFSET = 0x2C
  	  };
  	  /* Subtract a Fraction of a Second */
  	  namespace subfs {
  	  	  enum {
  	  		  POSITION = 0,
  	  		  MASK = 0x7FFF
  	  	  };
  	  }
  	  namespace add1s {
  	  	  enum {
  	  		  POSITION = 31,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  }
  } // namespace shiftr

  /* RTC Time Stamp Time Register */
  namespace tstr {
  	  enum {
  		OFFSET = 0x30
  	  };
  	  /* Seconds Units BCD */
  	  namespace su {
  	  	  enum {
  	  		  POSITION = 0,
  	  		  MASK = 0x0F
  	  	  };
  	  }
  	  /* Seconds Tens BCD */
  	  namespace st {
  	  	  enum {
  	  		  POSITION = 4,
  	  		  MASK = 0x07 << POSITION
  	  	  };
  	  }
  	  /* Minutes Units BCD */
  	  namespace mnu {
  	  	  enum {
  	  		  POSITION = 8,
  	  		  MASK = 0x0F << POSITION
  	  	  };
  	  }
  	  /* Minutes Tens BCD */
  	  namespace mnt {
  	  	  enum {
  	  		  POSITION = 12,
  	  		  MASK = 0x07 << POSITION
  	  	  };
  	  }
  	  /* Hours Units BCD */
  	  namespace hu {
  	  	  enum {
  	  		  POSITION = 16,
  	  		  MASK = 0x0F << POSITION
  	  	  };
  	  }
  	  /* Hours Tens BCD */
  	  namespace ht {
  	  	  enum {
  	  		  POSITION = 20,
  	  		  MASK = 0x03 << POSITION
  	  	  };
  	  }
  	  /* Time Period AM/PM */
  	  namespace pm {
  	  	  enum {
  	  		  POSITION = 22,
  	  		  MASK = 1 << POSITION
  	  	  };
  	      enum TimePeriod {
  	        AM = 0 << POSITION, // or 24 hours format
  	        PM = 1 << POSITION
  	      };
  	  }
  } // namespace tstr

  /* RTC Time Stamp Date Register */
  namespace tsdr {
  	  enum {
  		OFFSET = 0x34
  	  };
  	  /* Date Units BCD */
  	  namespace du {
  	  	  enum {
  	  		  POSITION = 0,
  	  		  MASK = 0x0F
  	  	  };
  	  }
  	  /* Date Tens BCD */
  	  namespace dt {
  	  	  enum {
  	  		  POSITION = 4,
  	  		  MASK = 0x03 << POSITION
  	  	  };
  	  }
  	  /* Month Units BCD */
  	  namespace mu {
  	  	  enum {
  	  		  POSITION = 8,
  	  		  MASK = 0x0F << POSITION
  	  	  };
  	  }
  	  /* Month Tens BCD */
  	  namespace mt {
  	  	  enum {
  	  		  POSITION = 12,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  }
      /* Weekday Units */
      namespace wdu {
      	enum {
      		POSITION = 13,
      		MASK = 7 << POSITION
      	};

      	enum Weekday {
      		MONDAY 		= 1 << POSITION,
      		TUESDAY 	= 2 << POSITION,
      		WEDNESDAY 	= 3 << POSITION,
      		THURSDAY 	= 4 << POSITION,
      		FRIDAY 		= 5 << POSITION,
      		SATURDAY 	= 6 << POSITION,
      		SUNDAY 		= 7 << POSITION
      	};
      }
  } // namespace tsdr

  /* RTC Time Stamp SSR Register */
  namespace tsssr {
  	  enum {
  		OFFSET = 0x38
  	  };
  	  /* Sub Second Value */
  	  namespace ss {
  	  	  enum {
  	  		  POSITION = 0,
  	  		  MASK = 0xFFFFFFFF
  	  	  };
  	  }
  } // namespace tsssr

  /* RTC Calibration Register */
  namespace calr {
  	  enum {
  		OFFSET = 0x3C
  	  };
  	  /* Calibration Minus */
  	  namespace calm {
  	  	  enum {
  	  		  POSITION = 0,
  	  		  MASK = 0x1FF
  	  	  };
  	  }
  	  /* 16 Seconds Calibration Cycle */
  	  namespace calw16 {
  	  	  enum {
  	  		  POSITION = 13,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  }
  	  /* 8 Seconds Calibration Cycle */
  	  namespace calw8 {
  	  	  enum {
  	  		  POSITION = 14,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  }
  	  /* Increase Frequency by 488.5 ppm */
  	  namespace calp {
  	  	  enum {
  	  		  POSITION = 15,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  	  enum Pulses {
  	  		  NO_PULSE_ADDED = 0,
  	  		  ONE_PULSE_ADDED = 1
  	  	  };
  	  }

  } // namespace calr

  /* RTC Tamper/Alternate Function Register */
  namespace tafcr {
  	  enum {
  		OFFSET = 0x40
  	  };
  	  /* Tamper 1 Detection Enable */
  	  namespace tamp1e {
  	  	  enum {
  	  		  POSITION = 0,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  	  enum States{
  	  		  TAMPER1_DETECTION_DISABLED = 0,
  	  		  TAMPER1_DETECTION_ENABLED = 1
  	  	  };
  	  }
  	  /* Tamper 1 Active Level */
  	  namespace tamp1trg {
  	  	  enum {
  	  		  POSITION = 1,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  }
  	  /* Tamper IRQ Enable */
  	  namespace tampie {
  	  	  enum {
  	  		  POSITION = 2,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  	  enum States {
  	  		  INTERRUPT_DISABLED = 0,
  	  		  INTERRUPT_ENABLED = 1
  	  	  };
  	  }
  	  /* Tamper 2 Detection Enable */
  	  namespace tamp2e {
  	  	  enum {
  	  		  POSITION = 3,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  	  enum States{
  	  		  TAMPER2_DETECTION_DISABLED = 0,
  	  		  TAMPER2_DETECTION_ENABLED = 1
  	  	  };
  	  }
  	  /* Tamper 2 Active Level */
  	  namespace tamp2trg {
  	  	  enum {
  	  		  POSITION = 4,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  }
  	  /* Activate TimeStamp on Tamper Detection Evnet */
  	  namespace tampts {
  	  	  enum {
  	  		  POSITION = 7,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  	  enum States {
  	  		  DO_NOT_SAVE_TIMESTAMP = 0,
  	  		  SAVE_TIMESTAMP = 1
  	  	  };
  	  }
  	  /* Tamper Sampling Frequency */
  	  namespace tampfreq {
  	  	  enum {
  	  		  POSITION = 8,
  	  		  MASK = 0x07 << POSITION
  	  	  };
  	  	  enum SamplingFreq {
  	  		  RTCCLK_DIV_32768 = 0 << POSITION,
  	  		  RTCCLK_DIV_16384 = 1 << POSITION,
  	  		  RTCCLK_DIV_8192  = 2 << POSITION,
  	  		  RTCCLK_DIV_4096  = 3 << POSITION,
  	  		  RTCCLK_DIV_2048  = 4 << POSITION,
  	  		  RTCCLK_DIV_1024  = 5 << POSITION,
  	  		  RTCCLK_DIV_512  = 6 << POSITION,
  	  		  RTCCLK_DIV_256  = 7 << POSITION,
  	  	  };
  	  }
  	  /* Tampre Filter Count */
  	  namespace tampflt {
  	  	  enum {
  	  		  POSITION = 11,
  	  		  MASK = 3 << POSITION
  	  	  };
  	  	  enum SamplingCounts {
  	  		  ACTIVE_ON_EDGE = 0,
  	  		  ACTIVE_AT_2_SAMPLE = 1,
  	  		  ACTIVE_AT_4_SAMPLE = 2,
  	  		  ACTIVE_AT_8_SAMPLE = 3,
  	  	  };
  	  }
  	  /* Tamper precharge duration */
  	  namespace tampprch {
  	  	  enum {
  	  		  POSITION = 13,
  	  		  MASK = 3 << POSITION
  	  	  };
  	  	  enum Cycles {
  	  		  ONE_CYCLE = 0,
  	  		  TWO_CYCLES = 1,
  	  		  FOUR_CYCLES = 2,
  	  		  EIGHT_CYCLES = 3,
  	  	  };
  	  }
  	  /* Tamper pull-up disable */
  	  namespace tamppudis {
  	  	  enum {
  	  		  POSITION = 15,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  }
  	  /* Tamper1 Mapping */
  	  namespace tamp1insel  {
  	  	  enum {
  	  		  POSITION = 16,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  	  enum InputSelect {
  	  		  RTC_AF1_AS_TAMPER = 0,
  	  		  RTC_AF2_AS_TAMPER = 1
  	  	  };
  	  }
  	  /* Timestamp Mapping */
  	  namespace tsinsel  {
  	  	  enum {
  	  		  POSITION = 17,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  	  enum InputSelect {
  	  		  RTC_AF1_AS_TIMESTAMP = 0,
  	  		  RTC_AF2_AS_TIMESTAMP  = 1
  	  	  };
  	  }
  	  /* RTC Alarm Output Type */
  	  namespace alarmouttype {
  	  	  enum {
  	  		  POSITION = 18,
  	  		  MASK = 1 << POSITION
  	  	  };
  	  	  enum Modes {
  	  		  OPEN_DRAIN = 0,
  	  		  PUSH_PULL = 1
  	  	  };
  	  }

  } // namespace tafcr

  /* RTC Alarm A SSR Register */
  namespace alrmassr {
  	  enum {
  		OFFSET = 0x44
  	  };
  	  /* Sub Second value */
  	  namespace ss {
  	  	  enum {
  	  		  POSITION = 0,
  	  		  MASK = 0x7FFF
  	  	  };
   	  }
  	  /* Mask MSB */
  	  namespace maskss {
  		  enum {
  			  POSITION = 24,
  			  MASK = 0x0F << POSITION
  		  };
  		  //TODO
  		  enum ssmask {

  		  };
  	  }
  } // namespace alrmassr

  /* RTC Alarm B SSR Register */
  namespace alrmbssr {
  	  enum {
  		OFFSET = 0x48
  	  };
  	  /* Sub Second value */
  	  namespace ss {
  	  	  enum {
  	  		  POSITION = 0,
  	  		  MASK = 0x7FFF
  	  	  };
   	  }
  	  /* Mask MSB */
  	  namespace maskss {
  		  enum {
  			  POSITION = 24,
  			  MASK = 0x0F << POSITION
  		  };
  		  //TODO
  		  enum ssmask {

  		  };
  	  }
  } // namespace alrmbssr


  enum {
	  RTC_MAGIC = 0x0BADFEED
  };

}// namespace rtc
