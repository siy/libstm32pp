/*******************************************************************************
 *
 * Copyright (C) 2012 Jorge Aparicio <jorge.aparicio.r@gmail.com>
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

namespace adc {
  enum Address {
#ifdef STM32F1XX
    ADC1 = alias::APB2 + 0x2400,
    ADC2 = alias::APB2 + 0x2800,
    ADC3 = alias::APB2 + 0x3C00,
#else // STM32F1XX
    ADC1 = alias::APB2 + 0x2000,
    ADC2 = alias::APB2 + 0x2100,
    ADC3 = alias::APB2 + 0x2200,
    ADC = alias::APB2 + 0x2300,
#endif // STM32F1XX
  };

  struct Registers {
      __RW
      u32 SR;       // 0x00: Status
      __RW
      u32 CR1;      // 0x04: Control 1
      __RW
      u32 CR2;      // 0x08: Control 2
      __RW
      u32 SMPR[2];  // 0x0C: Sample time
      __RW
      u32 JOFR[4];  // 0x14: Injected channel data offset
      __RW
      u32 HTR;      // 0x24: Watchdog higher threshold
      __RW
      u32 LTR;      // 0x28: Watchdog lower threshold
      __RW
      u32 SQR[3];   // 0x2C: Regular sequence
      __RW
      u32 JSQR;     // 0x38: Injected sequence
      __RW
      u32 JDR[4];   // 0x3C: Injected data 1
      __RW
      u32 DR;       // 0x4C: Data
  };

#if defined STM32F2XX || \
    defined STM32F4XX
  struct CommonRegisters {
      __RW
      u32 CSR;  //0x00: Common status
      __RW
      u32 CCR;  //0x04: Common control
      __RW
      u32 CDR;  //0x08: Common regular data
  };
#endif

  /**
   * ADC status register (ADC_SR)
   * Address offset: 0x00
   * Reset value: 0x0000 0000
   **/
  namespace sr {
    enum {
      OFFSET = 0x00
    };

    /**
     * Bit 0 AWD: Analog watchdog flag
     * This bit is set by hardware when the converted voltage crosses the values programmed in
     * the ADC_LTR and ADC_HTR registers. It is cleared by software.
     * 0: No analog watchdog event occurred
     * 1: Analog watchdog event occurred
     **/
    namespace awd {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        NO_ANALOG_WATCHDOG_EVENT_OCURRED = 0 << POSITION,
        ANALOG_WATCHDOG_EVENT_OCURRED = 1 << POSITION,
      };
    }  // namespace awd

    /**
     * Bit 1 EOC: Regular channel end of conversion
     * This bit is set by hardware at the end of the conversion of a regular group of channels. It is
     * cleared by software or by reading the ADC_DR register.
     * 0: Conversion not complete (EOCS=0), or sequence of conversions not complete (EOCS=1)
     * 1: Conversion complete (EOCS=0), or sequence of conversions complete (EOCS=1)
     **/
    namespace eoc {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        CONVERSION_IS_NOT_COMPLETE = 0 << POSITION,
        COMVERSION_COMPLETED = 1 << POSITION,
      };
    }  // namespace eoc

    /**
     * Bit 2 JEOC: Injected channel end of conversion
     * This bit is set by hardware at the end of the conversion of all injected channels in the group.
     * It is cleared by software.
     * 0: Conversion is not complete
     * 1: Conversion complete
     **/
    namespace jeoc {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        ALL_INJECTED_CONVERSIONS_ARE_NOT_COMPLETE = 0 << POSITION,
        ALL_INJECTED_CONVERSIONS_COMPLETED = 1 << POSITION,
      };
    }  // namespace jeoc

    /**
     * Bit 3 JSTRT: Injected channel start flag
     * This bit is set by hardware when injected group conversion starts. It is cleared by software.
     * 0: No injected group conversion started
     * 1: Injected group conversion has started
     **/
    namespace jstrt {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        NO_INJECTED_GROUP_CONVERSION_STARTED = 0 << POSITION,
        INJECTED_GROUP_CONVERSION_HAS_STARTED = 1 << POSITION,
      };
    }  // namespace jsrt

    /**
     * Bit 4 STRT: Regular channel start flag
     * This bit is set by hardware when regular channel conversion starts. It is cleared by software.
     * 0: No regular channel conversion started
     * 1: Regular channel conversion has started
     **/
    namespace strt {
      enum {
        POSITION = 4,
        MASK = 1 << POSITION
      };
      enum States {
        NO_REGULAR_CHANNEL_CONVERSION_STARTED = 0 << POSITION,
        REGULAR_CHANNEL_CONVERSION_HAS_STARTED = 1 << POSITION,
      };
    }  // namespace strt

#ifndef STM32F1XX
    /**
     * Bit 5 OVR: Overrun
     * This bit is set by hardware when data are lost (either in single mode or in dual/triple mode). It
     * is cleared by software. Overrun detection is enabled only when DMA = 1 or EOCS = 1.
     * 0: No overrun occurred
     * 1: Overrun has occurred
     **/
    namespace ovr {
      enum {
        POSITION = 5,
        MASK = 1 << POSITION
      };
      enum States {
        NO_OVERRUN_OCURRED = 0 << POSITION,
        OVERRUN_OCURRED = 1 << POSITION,
      };
    }  // namespace ovr
#endif // STM32F1XX

    /**
     * Bits 31:6 Reserved, must be kept at reset value.
     **/

  }  // namespace sr


  /**
   * ADC control register 1 (ADC_CR1)
   * Address offset: 0x04
   * Reset value: 0x0000 0000
   **/
  namespace cr1 {
    enum {
      OFFSET = 0x04
    };

    /**
     * Bits 4:0 AWDCH[4:0]: Analog watchdog channel select bits
     * These bits are set and cleared by software. They select the input channel to be guarded by
     * the analog watchdog.
     * Note: 00000: ADC analog input Channel0
     * 00001: ADC analog input Channel1
     * ...
     * 01111: ADC analog input Channel15
     * 10000: ADC analog input Channel16
     * 10001: ADC analog input Channel17
     * 10010: ADC analog input Channel18
     * Other values reserved
     **/
    namespace awdch {
      enum {
        POSITION = 0,
        MASK = 0b11111 << POSITION
      };
      enum States {
        SET_ANALOG_WATCHDOG_ON_CHANNEL0 = 0 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL1 = 1 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL2 = 2 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL3 = 3 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL4 = 4 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL5 = 5 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL6 = 6 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL7 = 7 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL8 = 8 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL9 = 9 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL10 = 10 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL11 = 11 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL12 = 12 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL13 = 13 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL14 = 14 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL15 = 15 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL16 = 16 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL17 = 17 << POSITION,
        SET_ANALOG_WATCHDOG_ON_CHANNEL18 = 18 << POSITION,
      };
    }  // namespace awdch

    /**
     * Bit 5 EOCIE: Interrupt enable for EOC
     * This bit is set and cleared by software to enable/disable the end of conversion interrupt.
     * 0: EOC interrupt disabled
     * 1: EOC interrupt enabled. An interrupt is generated when the EOC bit is set.
     **/
    namespace eocie {
      enum {
        POSITION = 5,
        MASK = 1 << POSITION
      };
      enum States {
        END_OF_CONVERSION_INTERRUPT_DISABLED = 0 << POSITION,
        END_OF_CONVERSION_INTERRUPT_ENABLED = 1 << POSITION,
      };
    }  // namespace eocie

    /**
     * Bit 6 AWDIE: Analog watchdog interrupt enable
     * This bit is set and cleared by software to enable/disable the analog watchdog interrupt.
     * 0: Analog watchdog interrupt disabled
     * 1: Analog watchdog interrupt enabled
     **/
    namespace awdie {
      enum {
        POSITION = 6,
        MASK = 1 << POSITION
      };
      enum States {
        ANALOG_WATCHDOG_INTERRUPT_DISABLED = 0 << POSITION,
        ANALOG_WATCHDOG_INTERRUPT_ENABLED = 1 << POSITION,
      };
    }  // namespace awdie

    /**
     * Bit 7 JEOCIE: Interrupt enable for injected channels
     * This bit is set and cleared by software to enable/disable the end of conversion interrupt for
     * injected channels.
     * 0: JEOC interrupt disabled
     * 1: JEOC interrupt enabled. An interrupt is generated when the JEOC bit is set.
     **/
    namespace jeocie {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        END_OF_ALL_INJECTED_CONVERSIONS_INTERRUPT_DISABLED = 0 << POSITION,
        END_OF_ALL_INJECTED_CONVERSIONS_INTERRUPT_ENABLED = 1 << POSITION,
      };
    }  // namespace jeocie

    /**
     * Bit 7 JEOCIE: Interrupt enable for injected channels
     * This bit is set and cleared by software to enable/disable the end of conversion interrupt for
     * injected channels.
     * 0: JEOC interrupt disabled
     * 1: JEOC interrupt enabled. An interrupt is generated when the JEOC bit is set.
     **/
    namespace scan {
      enum {
        POSITION = 8,
        MASK = 1 << POSITION
      };
      enum States {
        SCAN_MODE_DISABLED = 0 << POSITION,
        SCAN_MODE_ENABLED = 1 << POSITION,
      };
    }  // namespace scan

    /**
     * Bit 9 AWDSGL: Enable the watchdog on a single channel in scan mode
     * This bit is set and cleared by software to enable/disable the analog watchdog on the channel
     * identified by the AWDCH[4:0] bits.
     * 0: Analog watchdog enabled on all channels
     * 1: Analog watchdog enabled on a single channel
     **/
    namespace awdsgl {
      enum {
        POSITION = 9,
        MASK = 1 << POSITION
      };
      enum States {
        ANALOG_WATCHDOG_ENABLED_ON_ALL_CHANNELS = 0 << POSITION,
        ANALOG_WATCHDOG_ENABLED_ON_A_SINGLE_CHANNEL = 1 << POSITION,
      };
    }  // namespace awdsgl

    /**
     * Bit 10 JAUTO: Automatic injected group conversion
     * This bit is set and cleared by software to enable/disable automatic injected group conversion
     * after regular group conversion.
     * 0: Automatic injected group conversion disabled
     * 1: Automatic injected group conversion enabled
     **/
    namespace jauto {
      enum {
        POSITION = 10,
        MASK = 1 << POSITION
      };
      enum States {
        AUTOMATIC_INJECTED_CONVERSION_DISABLED = 0 << POSITION,
        AUTOMATIC_INJECTED_CONVERSION_ENABLED = 1 << POSITION,
      };
    }  // namespace jauto

    /**
     * Bit 11 DISCEN: Discontinuous mode on regular channels
     * This bit is set and cleared by software to enable/disable Discontinuous mode on regular
     * channels.
     * 0: Discontinuous mode on regular channels disabled
     * 1: Discontinuous mode on regular channels enabled
     **/
    namespace discen {
      enum {
        POSITION = 11,
        MASK = 1 << POSITION
      };
      enum States {
        DISCONTINUOUS_MODE_ON_REGULAR_CHANNELS_DISABLED = 0 << POSITION,
        DISCONTINUOUS_MODE_ON_REGULAR_CHANNELS_ENABLED = 1 << POSITION,
      };
    }  // namespace discen

    /**
     * Bit 12 JDISCEN: Discontinuous mode on injected channels
     * This bit is set and cleared by software to enable/disable discontinuous mode on the injected
     * channels of a group.
     * 0: Discontinuous mode on injected channels disabled
     * 1: Discontinuous mode on injected channels enabled
     **/
    namespace jdiscen {
      enum {
        POSITION = 12,
        MASK = 1 << POSITION
      };
      enum States {
        DISCONTINUOUS_MODE_ON_INJECTED_CHANNELS_DISABLED = 0 << POSITION,
        DISCONTINUOUS_MODE_ON_INJECTED_CHANNELS_ENABLED = 1 << POSITION,
      };
    }  // namespace discen

    /**
     * Bits 15:13 DISCNUM[2:0]: Discontinuous mode channel count
     * These bits are written by software to define the number of regular channels to be converted
     * in discontinuous mode, after receiving an external trigger.
     * 000: 1 channel
     * 001: 2 channels
     * ...
     * 111: 8 channels
     **/
    namespace discnum {
      enum {
        POSITION = 13,
        MASK = 0b111 << POSITION
      };
      enum States {
        _1_CHANNEL_FOR_DISCONTINUOUS_MODE = 0 << POSITION,
        _2_CHANNEL_FOR_DISCONTINUOUS_MODE = 1 << POSITION,
        _3_CHANNEL_FOR_DISCONTINUOUS_MODE = 2 << POSITION,
        _4_CHANNEL_FOR_DISCONTINUOUS_MODE = 3 << POSITION,
        _5_CHANNEL_FOR_DISCONTINUOUS_MODE = 4 << POSITION,
        _6_CHANNEL_FOR_DISCONTINUOUS_MODE = 5 << POSITION,
        _7_CHANNEL_FOR_DISCONTINUOUS_MODE = 6 << POSITION,
        _8_CHANNEL_FOR_DISCONTINUOUS_MODE = 7 << POSITION,
      };
    }  // namespace discnum

    /**
     * Bits 21:16 Reserved, must be kept at reset value.
     **/

    /**
     * Bit 22 JAWDEN: Analog watchdog enable on injected channels
     * This bit is set and cleared by software.
     * 0: Analog watchdog disabled on injected channels
     * 1: Analog watchdog enabled on injected channels
     **/
    namespace jawden {
      enum {
        POSITION = 22,
        MASK = 1 << POSITION
      };
      enum States {
        ANALOG_WATCHDOG_DISABLED_ON_INJECTED_CHANNELS = 0 << POSITION,
        ANALOG_WATCHDOG_ENABLED_ON_INJECTED_CHANNELS = 1 << POSITION,
      };
    }  // namespace jawden

    /**
     * Bit 23 AWDEN: Analog watchdog enable on regular channels
     * This bit is set and cleared by software.
     * 0: Analog watchdog disabled on regular channels
     * 1: Analog watchdog enabled on regular channels
     **/
    namespace awden {
      enum {
        POSITION = 23,
        MASK = 1 << POSITION
      };
      enum States {
        ANALOG_WATCHDOG_DISABLED_ON_REGULAR_CHANNELS = 0 << POSITION,
        ANALOG_WATCHDOG_ENABLED_ON_REGULAR_CHANNELS = 1 << POSITION,
      };
    }  // namespace awden

    /**
     * Bits 25:24 RES[1:0]: Resolution
     * These bits are written by software to select the resolution of the conversion.
     * 00: 12-bit (15 ADCCLK cycles)
     * 01: 10-bit (13 ADCCLK cycles)
     * 10: 8-bit (11 ADCCLK cycles)
     * 11: 6-bit (9 ADCCLK cycles)
     **/
    namespace res {
      enum {
        POSITION = 24,
        MASK = 0b11 << POSITION
      };
      enum States {
        _12_BITS_RESOLUTION = 0 << POSITION,
        _10_BITS_RESOLUTION = 1 << POSITION,
        _8_BITS_RESOLUTION = 2 << POSITION,
        _6_BITS_RESOLUTION = 3 << POSITION,
      };
    }  // namespace res

    /**
     * Bit 26 OVRIE: Overrun interrupt enable
     * This bit is set and cleared by software to enable/disable the Overrun interrupt.
     * 0: Overrun interrupt disabled
     * 1: Overrun interrupt enabled. An interrupt is generated when the OVR bit is set.
     **/
    namespace ovrie {
      enum {
        POSITION = 26,
        MASK = 1 << POSITION
      };
      enum States {
        OVERRUN_INTERRUPT_DISABLED = 0 << POSITION,
        ENABLE_OVERRUN_INTERRUPT_ENABLED = 1 << POSITION,
      };
    }  // namespace ovrie

    /**
     * Bits 31:27 Reserved, must be kept at reset value.
     **/

  }  // namespace cr1

  /**
   * ADC control register 2 (ADC_CR2)
   * Address offset: 0x08
   * Reset value: 0x0000 0000
   **/
  namespace cr2 {
    enum {
      OFFSET = 0x08
    };

    /**
     * Bit 0 ADON: A/D Converter ON / OFF
     * This bit is set and cleared by software.
     * Note: 0: Disable ADC conversion and go to power down mode
     * 		 1: Enable ADC
     **/
    namespace adon {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        ADC_POWERED_DOWN = 0 << POSITION,
        ADC_ENABLED = 1 << POSITION,
      };
    }  // namespace adon

    /**
     * Bit 1 CONT: Continuous conversion
     * This bit is set and cleared by software. If it is set, conversion takes place continuously until it
     * is cleared.
     * 0: Single conversion mode
     * 1: Continuous conversion mode
     **/
    namespace cont {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        SINGLE_CONVERSION_MODE = 0 << POSITION,
        CONTINUOUS_CONVERSION_MODE = 1 << POSITION,
      };
    }  // namespace cont

    /**
     * Bits 7:2 Reserved, must be kept at reset value.
     **/

    /**
     * Bit 8 DMA: Direct memory access mode (for single ADC mode)
     * This bit is set and cleared by software. Refer to the DMA controller chapter for more details.
     * 0: DMA mode disabled
     * 1: DMA mode enabled
     **/
    namespace dma {
      enum {
        POSITION = 8,
        MASK = 1 << POSITION
      };
      enum States {
        DMA_MODE_DISABLED = 0 << POSITION,
        DMA_MODE_ENABLED = 1 << POSITION,
      };
    }  // namespace dma

    /**
     * Bit 9 DDS: DMA disable selection (for single ADC mode)
     * This bit is set and cleared by software.
     * 0: No new DMA request is issued after the last transfer (as configured in the DMA controller)
     * 1: DMA requests are issued as long as data are converted and DMA=1
     **/
    namespace dds {
      enum {
        POSITION = 9,
        MASK = 1 << POSITION
      };
      enum States {
        NO_NEW_DMA_REQUEST_IS_ISSUED_AFTER_THE_LAST_TRANSFER = 0 << POSITION,
        DMA_REQUEST_ARE_ISSUED_AS_LONG_AS_DATA_IS_CONVERTED = 1 << POSITION,
      };
    }  // namespace dds

    /**
     * Bit 10 EOCS: End of conversion selection
     * This bit is set and cleared by software.
     * 0: The EOC bit is set at the end of each sequence of regular conversions. Overrun detection
     * is enabled only if DMA=1.
     * 1: The EOC bit is set at the end of each regular conversion. Overrun detection is enabled.
     **/
    namespace eocs {
      enum {
        POSITION = 10,
        MASK = 1 << POSITION
      };
      enum States {
        EOC_BIT_IS_SET_AFTER_A_SEQUENCE_OF_REGULAR_CONVERSIONS = 0 << POSITION,
        EOC_BIT_IS_SET_AFTER_EACH_REGULAR_CONVERSION = 1 << POSITION,
      };
    }  // namespace eocs

    /**
     * Bit 11 ALIGN: Data alignment
     * This bit is set and cleared by software.
     *
     * Right alignment of 12-bit data
     * Injected group
     * ---------------------------------------------------------------------------
     * |SEXT |SEXT |SEXT |SEXT |D11 |D10 |D9 |D8 |D7 |D6 |D5 |D4 |D3 |D2 |D1 |D0 |
     * ---------------------------------------------------------------------------
     *
     * Regular group
     * -----------------------------------------------------------------
     * |0 |0 |0 |0 |D11 | D10 |D9 |D8 | D7 |D6 |D5 |D4 |D3 |D2 |D1 |D0 |
     * -----------------------------------------------------------------
     *
     * Left alignment of 12-bit data
     * Injected group
     * ------------------------------------------------------------------
     * |SEXT |D11 |D10 |D9 |D8 |D7 |D6 |D5 |D4 |D3 |D2 |D1 |D0 |0 |0 |0 |
     * ------------------------------------------------------------------
     *
     * Regular group
     * -----------------------------------------------------------------
     * |D11 | D10 |D9 |D8 | D7 |D6 |D5 |D4 |D3 |D2 |D1 |D0 |0 |0 |0 |0 |
     * -----------------------------------------------------------------
     *
     * 0: Right alignment
     * 1: Left alignment
     **/
    namespace align {
      enum {
        POSITION = 11,
        MASK = 1 << POSITION
      };
      enum States {
        RIGTH_ALIGNED_DATA = 0 << POSITION,
        LEFT_ALIGNED_DATA = 1 << POSITION,
      };
    }  // namespace align

    /**
     * Bits 15:12 Reserved, must be kept at reset value.
     **/

    /**
     * Bits 19:16 JEXTSEL[3:0]: External event select for injected group
     * These bits select the external event used to trigger the start of conversion of an injected
     * group.
     * 0000: Timer 1 CC4 event
     * 0001: Timer 1 TRGO event
     * 0010: Timer 2 CC1 event
     * 0011: Timer 2 TRGO event
     * 0100: Timer 3 CC2 event
     * 0101: Timer 3 CC4 event
     * 0110: Timer 4 CC1 event
     * 0111: Timer 4 CC2 event
     * 1000: Timer 4 CC3 event
     * 1001: Timer 4 TRGO event
     * 1010: Timer 5 CC4 event
     * 1011: Timer 5 TRGO event
     * 1100: Timer 8 CC2 event
     * 1101: Timer 8 CC3 event
     * 1110: Timer 8 CC4 event
     * 1111: EXTI line15
     **/
    namespace jextsel {
      enum {
        POSITION = 16,
        MASK = 0b1111 << POSITION
      };
      enum States {
        INJECTED_GROUP_TRIGGERED_BY_TIMER1_CC4 = 0 << POSITION,
        INJECTED_GROUP_TRIGGERED_BY_TIMER1_TRGO = 1 << POSITION,
        INJECTED_GROUP_TRIGGERED_BY_TIMER2_CC1 = 2 << POSITION,
        INJECTED_GROUP_TRIGGERED_BY_TIMER2_TRGO = 3 << POSITION,
        INJECTED_GROUP_TRIGGERED_BY_TIMER3_CC2 = 4 << POSITION,
        INJECTED_GROUP_TRIGGERED_BY_TIMER3_CC4 = 5 << POSITION,
        INJECTED_GROUP_TRIGGERED_BY_TIMER4_CC1 = 6 << POSITION,
        INJECTED_GROUP_TRIGGERED_BY_TIMER4_CC2 = 7 << POSITION,
        INJECTED_GROUP_TRIGGERED_BY_TIMER4_CC3 = 8 << POSITION,
        INJECTED_GROUP_TRIGGERED_BY_TIMER4_TRGO = 9 << POSITION,
        INJECTED_GROUP_TRIGGERED_BY_TIMER5_CC4 = 10 << POSITION,
        INJECTED_GROUP_TRIGGERED_BY_TIMER5_TRGO = 11 << POSITION,
        INJECTED_GROUP_TRIGGERED_BY_TIMER8_CC2 = 12 << POSITION,
        INJECTED_GROUP_TRIGGERED_BY_TIMER8_CC3 = 13 << POSITION,
        INJECTED_GROUP_TRIGGERED_BY_TIMER8_CC4 = 14 << POSITION,
        INJECTED_GROUP_TRIGGERED_BY_EXTI15 = 15 << POSITION,
      };
    }  // namespace ajextel

    /**
     * Bits 21:20 JEXTEN: External trigger enable for injected channels
     * These bits are set and cleared by software to select the external trigger polarity and enable
     * the trigger of an injected group.
     * 00: Trigger detection disabled
     * 01: Trigger detection on the rising edge
     * 10: Trigger detection on the falling edge
     * 11: Trigger detection on both the rising and falling edges
     **/
    namespace jexten {
      enum {
        POSITION = 20,
        MASK = 0b11 << POSITION
      };
      enum States {
        INJECTED_TRIGGER_DISABLED = 0 << POSITION,
        INJECTED_TRIGGERED_ON_RISING_EDGE = 1 << POSITION,
        INJECTED_TRIGGERED_DETECTION_ON_FALLING_EDGE = 2 << POSITION,
        INJECTED_TRIGGERED_ON_RISING_AND_FALLING_EDGES = 3 << POSITION,
      };
    }  // namespace jexten

    /**
     * Bit 22 JSWSTART: Start conversion of injected channels
     * This bit is set by software and cleared by hardware as soon as the conversion starts.
     * 0: Reset state
     * 1: Starts conversion of injected channels
     * Note: This bit can be set only when ADON = 1 otherwise no conversion is launched.
     **/
    namespace jswstart {
      enum {
        POSITION = 22,
        MASK = 1 << POSITION
      };
      enum States {
        INJECTED_CHANNELS_ON_RESET_STATE = 0 << POSITION,
        START_CONVERSION_ON_INJECTED_CHANNELS = 1 << POSITION,
      };
    }  // namespace jswstart

    /**
     * Bit 23 Reserved, must be kept at reset value.
     **/

    /**
     * Bits 27:24 EXTSEL[3:0]: External event select for regular group
     * These bits select the external event used to trigger the start of conversion of a regular group:
     * 0000: Timer 1 CC1 event
     * 0001: Timer 1 CC2 event
     * 0010: Timer 1 CC3 event
     * 0011: Timer 2 CC2 event
     * 0100: Timer 2 CC3 event
     * 0101: Timer 2 CC4 event
     * 0110: Timer 2 TRGO event
     * 0111: Timer 3 CC1 event
     * 1000: Timer 3 TRGO event
     * 1001: Timer 4 CC4 event
     * 1010: Timer 5 CC1 event
     * 1011: Timer 5 CC2 event
     * 1100: Timer 5 CC3 event
     * 1101: Timer 8 CC1 event
     * 1110: Timer 8 TRGO event
     * 1111: EXTI line11
     **/
    namespace extsel {
      enum {
        POSITION = 24,
        MASK = 0b1111 << POSITION
      };
      enum States {
        REGULAR_GROUP_TRIGGERED_BY_TIMER1_CC1 = 0 << POSITION,
        REGULAR_GROUP_TRIGGERED_BY_TIMER1_CC2 = 1 << POSITION,
        REGULAR_GROUP_TRIGGERED_BY_TIMER1_CC3 = 2 << POSITION,
        REGULAR_GROUP_TRIGGERED_BY_TIMER2_CC2 = 3 << POSITION,
        REGULAR_GROUP_TRIGGERED_BY_TIMER2_CC3 = 4 << POSITION,
        REGULAR_GROUP_TRIGGERED_BY_TIMER2_CC4 = 5 << POSITION,
        REGULAR_GROUP_TRIGGERED_BY_TIMER2_TRGO = 6 << POSITION,
        REGULAR_GROUP_TRIGGERED_BY_TIMER3_CC1 = 7 << POSITION,
        REGULAR_GROUP_TRIGGERED_BY_TIMER3_TRGO = 8 << POSITION,
        REGULAR_GROUP_TRIGGERED_BY_TIMER4_CC4 = 9 << POSITION,
        REGULAR_GROUP_TRIGGERED_BY_TIMER5_CC1 = 10 << POSITION,
        REGULAR_GROUP_TRIGGERED_BY_TIMER5_CC2 = 11 << POSITION,
        REGULAR_GROUP_TRIGGERED_BY_TIMER5_CC3 = 12 << POSITION,
        REGULAR_GROUP_TRIGGERED_BY_TIMER8_CC1 = 13 << POSITION,
        REGULAR_GROUP_TRIGGERED_BY_TIMER8_TRGO = 14 << POSITION,
        REGULAR_GROUP_TRIGGERED_BY_EXTI11 = 15 << POSITION,
      };
    }  // namespace extsel

    /**
     * Bits 29:28 EXTEN: External trigger enable for regular channels
     * These bits are set and cleared by software to select the external trigger polarity and enable
     * the trigger of a regular group.00: Trigger detection disabled
     * 01: Trigger detection on the rising edge
     * 10: Trigger detection on the falling edge
     * 11: Trigger detection on both the rising and falling edges
     **/
    namespace exten {
      enum {
        POSITION = 28,
        MASK = 0b11 << POSITION
      };
      enum States {
        REGULAR_TRIGGER_DISABLED = 0 << POSITION,
        REGULAR_TRIGGER_ON_THE_RISING_EDGE = 1 << POSITION,
        REGULAR_TRIGGER_ON_THE_FALLING_EDGE = 2 << POSITION,
        REGULAR_TRIGGER_ON_RISING_AND_FALLING_EDGES = 3 << POSITION,
      };
    }  // namespace exten

    /**
     * Bit 30 SWSTART: Start conversion of regular channels
     * This bit is set by software to start conversion and cleared by hardware as soon as the
     * conversion starts.
     * 0: Reset state
     * 1: Starts conversion of regular channels
     * Note: This bit can be set only when ADON = 1 otherwise no conversion is launched.
     **/
    namespace swstart {
      enum {
        POSITION = 30,
        MASK = 1 << POSITION
      };
      enum States {
        REGULAR_CHANNELS_ON_RESET_STATE = 0 << POSITION,
        START_CONVERSION_ON_REGULAR_CHANNELS = 1 << POSITION,
      };
    }  // namespace swstart

    /**
     * Bit 31 Reserved, must be kept at reset value.
     **/
  }  // namespace cr2

  /**
   * ADC sample time register 1 (ADC_SMPR1)
   * Address offset: 0x0C
   * Reset value: 0x0000 0000
   **/
  namespace smpr1 {
    enum {
      OFFSET = 0x0C
    };

    /**
     * SMPR1 and SMPR2 have a common sampling time which is defined in namespace smp
     **/
  }  // namespace smpr1

  /**
   * ADC sample time register 2 (ADC_SMPR2)
   * Address offset: 0x10
   * Reset value: 0x0000 0000
   **/
  namespace smpr2 {
    enum {
      OFFSET = 0x10
    };

    /**
     * SMPR1 and SMPR2 have a common sampling time which is defined in namespace smp
     **/
  }  // namespace smpr2

  /**
   * Bits 31: 27 Reserved, must be kept at reset value.
   * Bits 26:0 SMPx[2:0]: Channel x sampling time selection
   * These bits are written by software to select the sampling time individually for each channel.
   * During sampling cycles, the channel selection bits must remain unchanged.
   * Note: 000: 3 cycles
   * 001: 15 cycles
   * 010: 28 cycles
   * 011: 56 cycles
   * 100: 84 cycles
   * 101: 112 cycles
   * 110: 144 cycles
   * 111: 480 cycles
   **/
  namespace smp {
    enum {
      MASK = 0b111,
      POSITION = 3
    };
    enum States {
      SAMPLING_TIME_3_CYCLES = 0,
      SAMPLING_TIME_15_CYCLES = 1,
      SAMPLING_TIME_28_CYCLES = 2,
      SAMPLING_TIME_56_CYCLES = 3,
      SAMPLING_TIME_84_CYCLES = 4,
      SAMPLING_TIME_112_CYCLES = 5,
      SAMPLING_TIME_144_CYCLES = 6,
      SAMPLING_TIME_480_CYCLES = 7,
    };
  }  // namespace smp

  /**
   * ADC injected channel data offset register 1 (ADC_JOFR1)
   * Address offset: 0x14
   * Reset value: 0x0000 0000
   **/
  namespace jofr1 {
    enum {
      OFFSET = 0x14
    };

    /**
     * Bits 31:12 Reserved, must be kept at reset value.
     * Bits 11:0 JOFFSETx[11:0]: Data offset for injected channel x
     * These bits are written by software to define the offset to be subtracted from the raw
     * converted data when converting injected channels. The conversion result can be read from
     * in the ADC_JDRx registers.
     **/
  }  // namespace jofr1

  /**
   * ADC injected channel data offset register 2 (ADC_JOFR2)
   * Address offset: 0x18
   * Reset value: 0x0000 0000
   **/
  namespace jofr2 {
    enum {
      OFFSET = 0x18
    };

    /**
     * Bits 31:12 Reserved, must be kept at reset value.
     * Bits 11:0 JOFFSETx[11:0]: Data offset for injected channel x
     * These bits are written by software to define the offset to be subtracted from the raw
     * converted data when converting injected channels. The conversion result can be read from
     * in the ADC_JDRx registers.
     **/

  }  // namespace jofr2

  /**
   * ADC injected channel data offset register 3 (ADC_JOFR3)
   * Address offset: 0x1C
   * Reset value: 0x0000 0000
   **/
  namespace jofr3 {
    enum {
      OFFSET = 0x1C
    };

    /**
     * Bits 31:12 Reserved, must be kept at reset value.
     * Bits 11:0 JOFFSETx[11:0]: Data offset for injected channel x
     * These bits are written by software to define the offset to be subtracted from the raw
     * converted data when converting injected channels. The conversion result can be read from
     * in the ADC_JDRx registers.
     **/

  }  // namespace jofr3

  /**
   * ADC injected channel data offset register 4 (ADC_JOFR4)
   * Address offset: 0x20
   * Reset value: 0x0000 0000
   **/
  namespace jofr4 {
    enum {
      OFFSET = 0x20
    };

    /**
     * Bits 31:12 Reserved, must be kept at reset value.
     * Bits 11:0 JOFFSETx[11:0]: Data offset for injected channel x
     * These bits are written by software to define the offset to be subtracted from the raw
     * converted data when converting injected channels. The conversion result can be read from
     * in the ADC_JDRx registers.
     **/

  }  // namespace jofr4

  /**
   * ADC watchdog higher threshold register (ADC_HTR)
   * Address offset: 0x24
   * Reset value: 0x0000 0FFF
   **/
  namespace htr {
    enum {
      OFFSET = 0x24
    };

    /**
     * Bits 31:12 Reserved, must be kept at reset value.
     * Bits 11:0 HT[11:0]: Analog watchdog higher threshold
     * These bits are written by software to define the higher threshold for the analog watchdog.
     **/
  }  // namespace htr

  /**
   * ADC watchdog lower threshold register (ADC_LTR)
   * Address offset: 0x28
   * Reset value: 0x0000 0000
   **/
  namespace ltr {
    enum {
      OFFSET = 0x28
    };

    /**
     * Bits 31:12 Reserved, must be kept at reset value.
     * Bits 11:0 LT[11:0]: Analog watchdog lower threshold
     * These bits are written by software to define the lower threshold for the analog watchdog.
     **/

  }  // namespace ltr


  /**
   * ADC regular sequence register 1 (ADC_SQR1)
   * Address offset: 0x2C
   * Reset value: 0x0000 0000
   **/
  namespace sqr1 {
    enum {
      OFFSET = 0x2C
    };

    /**
     * Bits 31:24 Reserved, must be kept at reset value.
     * Bits 23:20 L[3:0]: Regular channel sequence length
     * These bits are written by software to define the total number of conversions in the regular
     * channel conversion sequence.
     * 0000: 1 conversion
     * 0001: 2 conversions
     * ...
     * 1111: 16 conversions
     * ///TODO Figure out how to manipulate bits [19:0]
     * Bits 19:15 SQ16[4:0]: 16th conversion in regular sequence
     * These bits are written by software with the channel number (0..18) assigned as the 16th in
     * the conversion sequence.
     * Bits 14:10 SQ15[4:0]: 15th conversion in regular sequence
     * Bits 9:5 SQ14[4:0]: 14th conversion in regular sequence
     * Bits 4:0 SQ13[4:0]: 13th conversion in regular sequence
     **/
    namespace l {
      enum {
        POSITION = 20,
        MASK = 0b1111 << POSITION
      };
      enum States {
        SEQUENCE_OF_1_REGULAR_CONVERSION = 0 << POSITION,
        SEQUENCE_OF_2_REGULAR_CONVERSIONS = 1 << POSITION,
        SEQUENCE_OF_3_REGULAR_CONVERSIONS = 2 << POSITION,
        SEQUENCE_OF_4_REGULAR_CONVERSIONS = 3 << POSITION,
        SEQUENCE_OF_5_REGULAR_CONVERSIONS = 4 << POSITION,
        SEQUENCE_OF_6_REGULAR_CONVERSIONS = 5 << POSITION,
        SEQUENCE_OF_7_REGULAR_CONVERSIONS = 6 << POSITION,
        SEQUENCE_OF_8_REGULAR_CONVERSIONS = 7 << POSITION,
        SEQUENCE_OF_9_REGULAR_CONVERSIONS = 8 << POSITION,
        SEQUENCE_OF_10_REGULAR_CONVERSIONS = 9 << POSITION,
        SEQUENCE_OF_11_REGULAR_CONVERSIONS = 10 << POSITION,
        SEQUENCE_OF_12_REGULAR_CONVERSIONS = 11 << POSITION,
        SEQUENCE_OF_13_REGULAR_CONVERSIONS = 12 << POSITION,
        SEQUENCE_OF_14_REGULAR_CONVERSIONS = 13 << POSITION,
        SEQUENCE_OF_15_REGULAR_CONVERSIONS = 14 << POSITION,
        SEQUENCE_OF_16_REGULAR_CONVERSIONS = 15 << POSITION,
      };
    }  // namespace l
  }  // namespace sqr1

  /**
   * ADC regular sequence register 2 (ADC_SQR2)
   * Address offset: 0x30
   * Reset value: 0x0000 0000
   **/
  namespace sqr2 {
    enum {
      OFFSET = 0x30
    };

    /**
     * Bits 31:30 Reserved, must be kept at reset value.
     * Bits 29:26 SQ12[4:0]: 12th conversion in regular sequence
     * These bits are written by software with the channel number (0..18) assigned as the 12th in
     * the sequence to be converted.
     * Bits 24:20 SQ11[4:0]: 11th conversion in regular sequence
     * Bits 19:15 SQ10[4:0]: 10th conversion in regular sequence
     * Bits 14:10 SQ9[4:0]: 9th conversion in regular sequence
     * Bits 9:5 SQ8[4:0]: 8th conversion in regular sequence
     * Bits 4:0 SQ7[4:0]: 7th conversion in regular sequence
     **/

  }  // namespace sqr2

  /**
   * ADC regular sequence register 3 (ADC_SQR3)
   * Address offset: 0x34
   * Reset value: 0x0000 0000
   **/
  namespace sqr3 {
    enum {
      OFFSET = 0x34
    };

    /**
     * Bits 31:30 Reserved, must be kept at reset value.
     * Bits 29:25 SQ6[4:0]: 6th conversion in regular sequence
     * These bits are written by software with the channel number (0..18) assigned as the 6th in the
     * sequence to be converted.
     * Bits 24:20 SQ5[4:0]: 5th conversion in regular sequence
     * Bits 19:15 SQ4[4:0]: 4th conversion in regular sequence
     * Bits 14:10 SQ3[4:0]: 3rd conversion in regular sequence
     * Bits 9:5 SQ2[4:0]: 2nd conversion in regular sequence
     * Bits 4:0 SQ1[4:0]: 1st conversion in regular sequence
     **/
  }  // namespace sqr3

  /**
   * Mask Pattern for ADC_SQRx where x=1,2,3
   **/
  namespace sqr {
    enum {
      MASK = 0b11111,
      POSITION = 5
    };
  }  // namespace sqr

  /**
   * ADC injected sequence register (ADC_JSQR)
   * Address offset: 0x38
   * Reset value: 0x0000 0000
   **/
  namespace jsqr {
    enum {
      OFFSET = 0x38
    };

    /**
     * Bits 31:22 Reserved, must be kept at reset value.
     **/

    /**
     * Bits 21:20 JL[1:0]: Injected sequence length
     * These bits are written by software to define the total number of conversions in the injected
     * channel conversion sequence.
     * 00: 1 conversion
     * 01: 2 conversions
     * 10: 3 conversions
     * 11: 4 conversions
     **/
    namespace jl {
      enum {
        POSITION = 20,
        MASK = 0b11 << POSITION
      };
      enum States {
        SEQUENCE_OF_1_INJECTED_CONVERSION = 0 << POSITION,
        SEQUENCE_OF_2_INJECTED_CONVERSIONS = 1 << POSITION,
        SEQUENCE_OF_3_INJECTED_CONVERSIONS = 2 << POSITION,
        SEQUENCE_OF_4_INJECTED_CONVERSIONS = 3 << POSITION,
      };
    }  // namespace jl

    /**
     * Bits 19:15 JSQ4[4:0]: 4th conversion in injected sequence (when JL[1:0]=3, see note below)
     * These bits are written by software with the channel number (0..18) assigned as the 4th in the
     * sequence to be converted.
     * Bits 14:10 JSQ3[4:0]: 3rd conversion in injected sequence (when JL[1:0]=3, see note below)
     * Bits 9:5 JSQ2[4:0]: 2nd conversion in injected sequence (when JL[1:0]=3, see note below)
     * Bits 4:0 JSQ1[4:0]: 1st conversion in injected sequence (when JL[1:0]=3, see note below)
     *
     * Note: When JL[1:0]=3 (4 injected conversions in the sequencer), the ADC converts the channels
     * in the following order: JSQ1[4:0], JSQ2[4:0], JSQ3[4:0], and JSQ4[4:0].
     * When JL=2 (3 injected conversions in the sequencer), the ADC converts the channels in the
     * following order: JSQ2[4:0], JSQ3[4:0], and JSQ4[4:0].
     * When JL=1 (2 injected conversions in the sequencer), the ADC converts the channels in
     * starting from JSQ3[4:0], and then JSQ4[4:0].
     * When JL=0 (1 injected conversion in the sequencer), the ADC converts only JSQ4[4:0]
     * channel.
     **/
  }  // namespace jsqr

 /**
  * Mask Pattern for JSQx in ADC_JSQR where x=1..4
  **/
 namespace jsq {
    enum {
      MASK = 0b11111,
      POSITION = 5
    };
  }  // namespace jsq

 /**
  * ADC injected data register 1 (ADC_JDR1)
  * Address offset: 0x3C
  * Reset value: 0x0000 0000
  **/
  namespace jdr1 {
    enum {
      OFFSET = 0x3C
    };

    /**
     * Bits 31:16 Reserved, must be kept at reset value.
     * Bits 15:0 JDATA[15:0]: Injected data
     * These bits are read-only. They contain the conversion result from injected channel x. The
     * data are left -or right-aligned
     **/
  }  // namespace jdr1

  /**
   * ADC injected data register 2 (ADC_JDR2)
   * Address offset: 0x40
   * Reset value: 0x0000 0000
   **/
  namespace jdr2 {
    enum {
      OFFSET = 0x40
    };

    /**
     * Bits 31:16 Reserved, must be kept at reset value.
     * Bits 15:0 JDATA[15:0]: Injected data
     * These bits are read-only. They contain the conversion result from injected channel x. The
     * data are left -or right-aligned
     **/
  }  // namespace jdr2

  /**
   * ADC injected data register 3 (ADC_JDR3)
   * Address offset: 0x44
   * Reset value: 0x0000 0000
   **/
  namespace jdr3 {
    enum {
      OFFSET = 0x44
    };

    /**
     * Bits 31:16 Reserved, must be kept at reset value.
     * Bits 15:0 JDATA[15:0]: Injected data
     * These bits are read-only. They contain the conversion result from injected channel x. The
     * data are left -or right-aligned
     **/
  }  // namespace jdr3

  /**
   * ADC injected data register 4 (ADC_JDR4)
   * Address offset: 0x48
   * Reset value: 0x0000 0000
   **/
  namespace jdr4 {
    enum {
      OFFSET = 0x48
    };

    /**
     * Bits 31:16 Reserved, must be kept at reset value.
     * Bits 15:0 JDATA[15:0]: Injected data
     * These bits are read-only. They contain the conversion result from injected channel x. The
     * data are left -or right-aligned
     **/
  }  // namespace jdr4

  /**
   * ADC regular data register (ADC_DR)
   * Address offset: 0x4C
   * Reset value: 0x0000 0000
   **/
  namespace dr {
    enum {
      OFFSET = 0x4C
    };

    /**
     * Bits 31:16 Reserved, must be kept at reset value.
     * Bits 15:0 DATA[15:0]: Regular data
     * These bits are read-only. They contain the conversion result from the regular
     * channels. The data are left- or right-aligned
     **/
  }  // namespace dr

  /**
   * ADC Common status register (ADC_CSR)
   * Address offset: 0x00 (this offset address is relative to ADC1 base address + 0x300)
   * Reset value: 0x0000 0000
   * This register provides an image of the status bits of the different ADCs. Nevertheless it is
   * read-only and does not allow to clear the different status bits. Instead each status bit must
   * be cleared by writing it to 0 in the corresponding ADC_SR register.
   **/
  namespace csr {
    enum {
      OFFSET = 0x00
    };

    /**
     * Bits 31:22 Reserved, must be kept at reset value.
     * Bit 21 OVR3: Overrun flag of ADC3						-	This bit is a copy of the OVR bit in the 	ADC3_SR register.
     * Bit 20 STRT3: Regular channel Start flag of ADC3			-	This bit is a copy of the STRT bit in the 	ADC3_SR register.
     * Bit 19 JSTRT3: Injected channel Start flag of ADC3		-	This bit is a copy of the JSTRT bit in the 	ADC3_SR register.
     * Bit 18 JEOC3: Injected channel end of conversion of ADC3	-	This bit is a copy of the JEOC bit in the 	ADC3_SR register.
     * Bit 17 EOC3: End of conversion of ADC3					-	This bit is a copy of the EOC bit in the 	ADC3_SR register.
     * Bit 16 AWD3: Analog watchdog flag of ADC3				-	This bit is a copy of the AWD bit in the 	ADC3_SR register.
     * Bits 15:14 Reserved, must be kept at reset value.
     * Bit 13 OVR2: Overrun flag of ADC2						-	This bit is a copy of the OVR bit in the 	ADC2_SR register.
     * Bit 12 STRT2: Regular channel Start flag of ADC2			-	This bit is a copy of the STRT bit in the 	ADC2_SR register.
     * Bit 11 JSTRT2: Injected channel Start flag of ADC2		-	This bit is a copy of the JSTRT bit in the 	ADC2_SR register.
     * Bit 10 JEOC2: Injected channel end of conversion of ADC2	-	This bit is a copy of the JEOC bit in the 	ADC2_SR register.
     * Bit 9 EOC2: End of conversion of ADC2					-	This bit is a copy of the EOC bit in the 	ADC2_SR register.
     * Bit 8 AWD2: Analog watchdog flag of ADC2					-	This bit is a copy of the AWD bit in the 	ADC2_SR register.
     * Bits 7:6 Reserved, must be kept at reset value.
     * Bit 5 OVR1: Overrun flag of ADC1							-	This bit is a copy of the OVR bit in the 	ADC1_SR register.
     * Bit 4 STRT1: Regular channel Start flag of ADC1			-	This bit is a copy of the STRT bit in the 	ADC1_SR register.
     * Bit 3 JSTRT1: Injected channel Start flag of ADC1		-	This bit is a copy of the JSTRT bit in the 	ADC1_SR register.
     * Bit 2 JEOC1: Injected channel end of conversion of ADC1	-	This bit is a copy of the JEOC bit in the 	ADC1_SR register.
     * Bit 1 EOC1: End of conversion of ADC1					-	This bit is a copy of the EOC bit in the 	ADC1_SR register.
     * Bit 0 AWD1: Analog watchdog flag of ADC1					-	This bit is a copy of the AWD bit in the 	ADC1_SR register.
     **/
  }  // namespace csr

  /**
   * ADC common control register (ADC_CCR)
   * Address offset: 0x04 (this offset address is relative to ADC1 base address + 0x300)
   * Reset value: 0x0000 0000
   **/
  namespace ccr {
    enum {
      OFFSET = 0x04
    };

    /**
     * Bits 31:24 Reserved, must be kept at reset value.
     **/

    /**
     *Bit 23 TSVREFE: Temperature sensor and VREFINT enable
     *Bit This bit is set and cleared by software to enable/disable the temperature sensor and the
     *Bit VREFINT channel.
     *Bit 0: Temperature sensor and VREFINT channel disabled
     *Bit 1: Temperature sensor and VREFINT channel enabled
     **/
    namespace tsvrefe {
      enum {
        POSITION = 23,
        MASK = 0b1 << POSITION
      };
      enum States {
        TEMPERATURE_SENSOR_AND_VREFINT_CHANNEL_DISABLED = 0 << POSITION,
        TEMPERATURE_SENSOR_AND_VREFINT_CHANNEL_ENABLED = 1 << POSITION,
      };
    }  // namespace tsvrefe

    /**
     * Bit 22 VBATE: VBAT enable
     * This bit is set and cleared by software to enable/disable the VBAT channel.
     * 0: VBAT channel disabled
     * 1: VBAT channel enabled
     **/
    namespace vbate {
      enum {
        POSITION = 22,
        MASK = 0b1 << POSITION
      };
      enum States {
        BATTERY_VOLTAGE_CHANNEL_DISABLED = 0 << POSITION,
        BATTERY_VOLTAGE_CHANNEL_ENABLED = 1 << POSITION,
      };
    }  // namespace vbate

    /**
     * Bits 21:18 Reserved, must be kept at reset value.
     **/

    /**
     * Bits 17:16 ADCPRE: ADC prescaler
     * Set and cleared by software to select the frequency of the clock to the ADC. The clock is
     * common for all the ADCs.
     * Note: 00: PCLK2 divided by 2
     * 01: PCLK2 divided by 4
     * 10: PCLK2 divided by 6
     * 11: PCLK2 divided by 8
     **/
    namespace adcpre {
      enum {
        POSITION = 16,
        MASK = 0b11 << POSITION
      };
      enum States {
        APB2_CLOCK_DIVIDED_BY_2 = 0 << POSITION,
        APB2_CLOCK_DIVIDED_BY_4 = 1 << POSITION,
        APB2_CLOCK_DIVIDED_BY_6 = 2 << POSITION,
        APB2_CLOCK_DIVIDED_BY_8 = 3 << POSITION,
      };
    }  // namespace adcpre

    /**
     * Bits 15:14 DMA: Direct memory access mode for multi ADC mode
     * This bit-field is set and cleared by software. Refer to the DMA controller section for more
     * details.
     * 00: DMA mode disabled
     * 01: DMA mode 1 enabled (2 / 3 half-words one by one - 1 then 2 then 3)
     * 10: DMA mode 2 enabled (2 / 3 half-words by pairs - 2&1 then 1&3 then 3&2)
     * 11: DMA mode 3 enabled (2 / 3 bytes by pairs - 2&1 then 1&3 then 3&2)
     **/
    namespace dma {
      enum{
    	POSITION = 14,
    	MASK = 0b11 << POSITION
      };
      enum States {
    	DMA_MODE_DISABLED = 0 << POSITION,
    	DMA_MODE1_ENABLED = 1 << POSITION,
    	DMA_MODE2_ENABLED = 2 << POSITION,
    	DMA_MODE3_ENABLED = 3 << POSITION,
      };
    } // namespace dma

    /**
     * Bit 13 DDS: DMA disable selection (for multi-ADC mode)
     * This bit is set and cleared by software.
     * 0: No new DMA request is issued after the last transfer (as configured in the DMA controller).
     *    DMA bits are not cleared by hardware, however they must have been cleared and set to the
     *    wanted mode by software before new DMA requests can be generated.
     * 1: DMA requests are issued as long as data are converted and DMA = 01, 10 or 11.
     **/
    namespace dds{
      enum{
    	  POSITION = 13,
    	  MASK = 0b1 << POSITION
      };
      enum States {
    	  NO_NEW_DMA_REQUEST_IS_ISSUED_AFTER_THE_LAST_TRANSFER = 0 << POSITION,
    	  DMA_REQUEST_ARE_ISSUED_AS_LONG_AS_DATA_IS_CONVERTED = 1 << POSITION,
      };

    } // namespace dds

    /**
     * Bit 12 Reserved, must be kept at reset value.
     **/

    /**
     * Bit 11:8 DELAY: Delay between 2 sampling phases
     * Set and cleared by software. These bits are used in dual or triple interleaved modes.
     * 0000: 5 * TADCCLK
     * 0001: 6 * TADCCLK
     * 0010: 7 * TADCCLK
     * ...
     * 1111: 20 * TADCCLK
     **/
    namespace delay{
      enum{
    	  POSITION = 8,
    	  MASK = 0b1111 << POSITION
      };
      enum States {
    	  SAMPLING_DELAY_5_ADC_CLOCK_CYCLES = 0 << POSITION,
    	  SAMPLING_DELAY_6_ADC_CLOCK_CYCLES = 1 << POSITION,
    	  SAMPLING_DELAY_7_ADC_CLOCK_CYCLES = 2 << POSITION,
    	  SAMPLING_DELAY_8_ADC_CLOCK_CYCLES = 3 << POSITION,
    	  SAMPLING_DELAY_9_ADC_CLOCK_CYCLES = 4 << POSITION,
    	  SAMPLING_DELAY_10_ADC_CLOCK_CYCLES = 5 << POSITION,
    	  SAMPLING_DELAY_11_ADC_CLOCK_CYCLES = 6 << POSITION,
    	  SAMPLING_DELAY_12_ADC_CLOCK_CYCLES = 7 << POSITION,
    	  SAMPLING_DELAY_13_ADC_CLOCK_CYCLES = 8 << POSITION,
    	  SAMPLING_DELAY_14_ADC_CLOCK_CYCLES = 9 << POSITION,
    	  SAMPLING_DELAY_15_ADC_CLOCK_CYCLES = 10 << POSITION,
    	  SAMPLING_DELAY_16_ADC_CLOCK_CYCLES = 11 << POSITION,
    	  SAMPLING_DELAY_17_ADC_CLOCK_CYCLES = 12 << POSITION,
    	  SAMPLING_DELAY_18_ADC_CLOCK_CYCLES = 13 << POSITION,
    	  SAMPLING_DELAY_19_ADC_CLOCK_CYCLES = 14 << POSITION,
    	  SAMPLING_DELAY_20_ADC_CLOCK_CYCLES = 15 << POSITION,
      };
    } // namespace delay

    /**
     * Bits 7:5 Reserved, must be kept at reset value.
     **/

    /**
     * Bits 4:0 MULTI[4:0]: Multi ADC mode selection
     * These bits are written by software to select the operating mode.
     * – All the ADCs independent:
     * 00000: Independent mode
     * – 00001 to 01001: Dual mode, ADC1 and ADC2 working together, ADC3 is independent
     * 00001: Combined regular simultaneous + injected simultaneous mode
     * 00010: Combined regular simultaneous + alternate trigger mode
     * 00011: Reserved
     * 00101: Injected simultaneous mode only
     * 00110: Regular simultaneous mode only
     * 00111: interleaved mode only
     * 01001: Alternate trigger mode only
     * – 10001 to 11001: Triple mode: ADC1, 2 and 3 working together
     * 10001: Combined regular simultaneous + injected simultaneous mode
     * 10010: Combined regular simultaneous + alternate trigger mode
     * 10011: Reserved
     * 10101: Injected simultaneous mode only
     * 10110: Regular simultaneous mode only
     * 10111: interleaved mode only
     * 11001: Alternate trigger mode only
     * All other combinations are reserved and must not be programmed
     * Note: In multi mode, a change of channel configuration generates an abort that can cause a
     * loss of synchronization. It is recommended to disable the multi ADC mode before any
     * configuration change.
     **/
    namespace multi {
      enum {
    	  POSITION = 0,
    	  MASK = 0b11111 << POSITION
      };
      enum States {
    	  INDEPENDENT_MODE                	  = 0 << POSITION,

    	  DUAL_MODE_REGULAR_AND_INJECTED  	  = 1 << POSITION,
    	  DUAL_MODE_REGULAR_AND_ALTERNATE 	  = 2 << POSITION,
    	  DUAL_MODE_INJECTED_ONLY		  	  = 5 << POSITION,
    	  DUAL_MODE_REGULAR_ONLY		  	  = 6 << POSITION,
    	  DUAL_MODE_INTERLEAVED_ONLY	  	  = 7 << POSITION,
    	  DUAL_MODE_ALTERNATE_TRIGGER_ONLY 	  = 9 << POSITION,

    	  TRIPPLE_MODE_REGULAR_AND_INJECTED   = 17 << POSITION,
    	  TRIPPLE_MODE_REGULAR_AND_ALTERNATE  = 18 << POSITION,
    	  TRIPPLE_MODE_INJECTED_ONLY		  = 21 << POSITION,
    	  TRIPPLE_MODE_REGULAR_ONLY		  	  = 22 << POSITION,
    	  TRIPPLE_MODE_INTERLEAVED_ONLY	  	  = 23 << POSITION,
    	  TRIPPLE_MODE_ALTERNATE_TRIGGER_ONLY = 25 << POSITION,
      };
    }  // namespace multi
  }  // namespace ccr

  /**
   * ADC common regular data register for dual and triple modes (ADC_CDR)
   * Address offset: 0x08 (this offset address is relative to ADC1 base address + 0x300)
   * Reset value: 0x0000 0000
   **/
  namespace cdr {
    enum {
      OFFSET = 0x08
    };

    /**
     * Bits 31:16 DATA2[15:0]: 2nd data item of a pair of regular conversions
     * - In dual mode, these bits contain the regular data of ADC2. Refer to Dual ADC mode.
     * – In triple mode, these bits contain alternatively the regular data of ADC2, ADC1 and ADC3.
     * Refer to Triple ADC mode.
     * Bits 15:0 DATA1[15:0]: 1st data item of a pair of regular conversions
     * – In dual mode, these bits contain the regular data of ADC1. Refer to Dual ADC mode
     * – In triple mode, these bits contain alternatively the regular data of ADC1, ADC3 and ADC2.
     * Refer to Triple ADC mode.
     **/
  }  // namespace cdr
}  // namespace adc

/**
 * ADC register map
 * The following table summarizes the ADC registers.
 * Offset - Register
 * 0x000  - 0x04C ADC1
 * 0x050  - 0x0FC Reserved
 * 0x100  - 0x14C ADC2
 * 0x118  - 0x1FC Reserved
 * 0x200  - 0x24C ADC3
 * 0x250  - 0x2FC Reserved
 * 0x300  - 0x308 Common registers
 **/
