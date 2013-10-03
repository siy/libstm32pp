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
 *                             Real-Time Clock
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"

#include "../defs.hpp"
#include "../clock.hpp"
#include "../../memorymap/rtc.hpp"

#define SYNC_TIMEOUT (0x00020000)
#define INIT_TIMEOUT (0x00010000)

typedef struct
{
	u8 sec;    /* second (0-61, allows for leap seconds) */
	u8 min;    /* minute (0-59) */
	u8 hour;   /* hour (0-23) */
}RTCtime_t;

typedef struct
{
	u8 wday;	/* day of the week */
	u8 mday;	/* day of the month (1-31) */
	u8 month;  	/* month (0-11) */
	u8 year;   	/* years since 1900 */
}RTCdate_t;

namespace rtcday
{
	enum
	{
		MONDAY = 1,
		TUESDAY,
		WEDNESDAY,
		THURSDAY,
		FRIDAY,
		SATURDAY,
		SUNDAY
	};
}

namespace rtcmonth
{
	enum
	{
		JANUARY = 1,
		FEBRUARY,
		MARCH,
		APRIL,
		MAY,
		JUNE,
		JULY,
		AUGUST,
		SEPTEMBER,
		OCTOBER,
		NOVEMBER,
		DECEMBER
	};
}
// Low-level access to the registers
#define RTC_REGS reinterpret_cast<rtc::Registers *>(rtc::ADDRESS)

// High-level functions
namespace rtc {

  class Functions {
    public:

	  static inline void initialize();

	  static inline void setTime(RTCtime_t);
	  static inline void setDate(RTCdate_t);
	  static inline void getTime(RTCtime_t*);
	  static inline void getDate(RTCdate_t*);
	  static inline void setDST(cr::dst::States);

//	  static inline void setAlarm(); //TODO
    private:
      Functions();
	  template<isr::Flags F>
	  static inline bool getStatus();

	  template<isr::Flags F>
	  static inline void clearStatusFlag();

	  template<u8 OFFSET, u8 BIT>
	  static inline void enableInterrupt();

	  template<u8 OFFSET, u8 BIT>
	  static inline void disableInterrupt();

	  static inline void setTimeFormat(cr::fmt::format);
	  static inline bool getTimeFormat();
	  static inline u8 bcd2bin(u8);
	  static inline u8 bin2bcd(u8);
	  static inline void wplock();
	  static inline void wpunlock();
	  static inline void syncwait();
	  static inline void resume();
	  static inline void enterinit();
	  static inline void exitinit();
	  static inline void setup();

//	  static inline bool isCalendarFromCounter();
	  static inline bool isValueFromCounter();
	  static inline void takeValueFromShadow();
	  static inline void takeValueFromCounter();
  };
}

// High-level access to the peripheral
typedef rtc::Functions RTC;

#include "../../bits/rtc.tcc"


