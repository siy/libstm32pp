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

namespace dma {
  namespace common {
    enum Address {
#ifdef STM32F1XX
      DMA1 = alias::AHB + 0x0000,
      DMA2 = alias::AHB + 0x0400,
#else // STM32F1XX
      DMA1 = alias::AHB1 + 0x6000,
      DMA2 = alias::AHB1 + 0x6400,
#endif // STM32F1XX
    };

    struct Registers
    {
#ifdef STM32F1XX
        __RW
        u32 ISR;   // 0x00: Interrupt status
        __RW
        u32 IFCR;// 0x04: Interrupt flag clear
#else // STM32F1XX
        __RW
        u32 ISR[2];  // 0x00, 0x04: Interrupt status
        __RW
        u32 IFCR[2];  // 0x08, 0x0C: Interrupt flag clear
#endif // STM32F1XX
    };

#ifdef STM32F1XX
    namespace isr {
      enum {
        OFFSET = 0x00
      };
    }  // namespace isr

    namespace ifcr {
      enum {
        OFFSET = 0x00
      };
    }  // namespace ifcr
#else // STM32F1XX


    /**
     * DMA low interrupt status register (DMA_LISR)
     * Address offset: 0x00
     * Reset value: 0x0000 0000
     **/
    namespace lisr {
      enum {
        OFFSET = 0x00
      };

      /**
       * Bits 31:28, 15:12 Reserved, must be kept at reset value.
       **/

      /**
       * Bits 27, 21, 11, 5 TCIFx: Stream x transfer complete interrupt flag (x = 3..0)
       * This bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the
       * DMA_LIFCR register.
       * 0: No transfer complete event on stream x
       * 1: A transfer complete event occurred on stream x
       **/

      /**
       * Bits 26, 20, 10, 4 HTIFx: Stream x half transfer interrupt flag (x=3..0)
       * This bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the
       * DMA_LIFCR register.
       * 0: No half transfer event on stream x
       * 1: A half transfer event occurred on stream x
       **/

      /**
       * Bits 25, 19, 9, 3 TEIFx: Stream x transfer error interrupt flag (x=3..0)
       * This bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the
       * DMA_LIFCR register.
       * 0: No transfer error on stream x
       * 1: A transfer error occurred on stream x
       **/

      /**
       * Bits 24, 18, 8, 2 DMEIFx: Stream x direct mode error interrupt flag (x=3..0)
       * This bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the
       * DMA_LIFCR register.
       * 0: No Direct Mode Error on stream x
       * 1: A Direct Mode Error occurred on stream x
       **/

      /**
       * Bits 23, 17, 7, 1 Reserved, must be kept at reset value.
       **/

      /**
       * Bits 22, 16, 6, 0 FEIFx: Stream x FIFO error interrupt flag (x=3..0)
       * This bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the
       * DMA_LIFCR register.
       * 0: No FIFO Error event on stream x
       * 1: A FIFO Error event occurred on stream x
       **/

    }  // namespace lisr

    /**
     * DMA high interrupt status register (DMA_HISR)
     * Address offset: 0x04
     * Reset value: 0x0000 0000
     **/
    namespace hisr {
      enum {
        OFFSET = 0x04
      };

      /**
       * Bits 31:28, 15:12 Reserved, must be kept at reset value.
       **/

      /**
       * Bits 27, 21, 11, 5 TCIFx: Stream x transfer complete interrupt flag (x=7..4)
       * This bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the
       * DMA_HIFCR register.
       * 0: No transfer complete event on stream x
       * 1: A transfer complete event occurred on stream x
       **/

      /**
       * Bits 26, 20, 10, 4 HTIFx: Stream x half transfer interrupt flag (x=7..4)
       * This bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the
       * DMA_HIFCR register.
       * 0: No half transfer event on stream x
       * 1: A half transfer event occurred on stream x
       **/

      /**
       * Bits 25, 19, 9, 3 TEIFx: Stream x transfer error interrupt flag (x=7..4)
       * This bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the
       * DMA_HIFCR register.
       * 0: No transfer error on stream x
       * 1: A transfer error occurred on stream x
       **/

      /**
       * Bits 24, 18, 8, 2 DMEIFx: Stream x direct mode error interrupt flag (x=7..4)
       * This bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the
       * DMA_HIFCR register.
       * 0: No Direct mode error on stream x
       * 1: A Direct mode error occurred on stream x
       **/

      /**
       * Bits 23, 17, 7, 1 Reserved, must be kept at reset value.
       **/

      /**
       * Bits 22, 16, 6, 0 FEIFx: Stream x FIFO error interrupt flag (x=7..4)
       * This bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the
       * DMA_HIFCR register.
       * 0: No FIFO error event on stream x
       * 1: A FIFO error event occurred on stream x
       **/

    }  // namespace hisr


    /**
     * DMA low interrupt flag clear register (DMA_LIFCR)
     * Address offset: 0x08
     * Reset value: 0x0000 0000
     **/
    namespace lifcr {
      enum {
        OFFSET = 0x08
      };

      /**
       * Bits 31:28, 15:12 Reserved, must be kept at reset value.
       **/

      /**
       * Bits 27, 21, 11, 5 CTCIFx: Stream x clear transfer complete interrupt flag (x = 3..0)
       * Writing 1 to this bit clears the corresponding TCIFx flag in the DMA_LISR register
       **/

      /**
       * Bits 26, 20, 10, 4 CHTIFx: Stream x clear half transfer interrupt flag (x = 3..0)
       * Writing 1 to this bit clears the corresponding HTIFx flag in the DMA_LISR register
       **/

      /**
       * Bits 25, 19, 9, 3 CTEIFx: Stream x clear transfer error interrupt flag (x = 3..0)
       * Writing 1 to this bit clears the corresponding TEIFx flag in the DMA_LISR register
       **/

      /**
       * Bits 24, 18, 8, 2 CDMEIFx: Stream x clear direct mode error interrupt flag (x = 3..0)
       * Writing 1 to this bit clears the corresponding DMEIFx flag in the DMA_LISR register
       **/

      /**
       * Bits 23, 17, 7, 1 Reserved, must be kept at reset value.
       **/

      /**
       * Bits 22, 16, 6, 0 CFEIFx: Stream x clear FIFO error interrupt flag (x = 3..0)
       * Writing 1 to this bit clears the corresponding CFEIFx flag in the DMA_LISR register
       **/

    }  // namespace lifcr


    /**
     * DMA high interrupt flag clear register (DMA_HIFCR)
     * Address offset: 0x0C
     * Reset value: 0x0000 0000
     **/
    namespace hifcr {
      enum {
        OFFSET = 0x0C
      };

      /**
       *Bits 31:28, 15:12 Reserved, must be kept at reset value.
       **/

      /**
       * Bits 27, 21, 11, 5 CTCIFx: Stream x clear transfer complete interrupt flag (x = 7..4)
       * Writing 1 to this bit clears the corresponding TCIFx flag in the DMA_HISR register
       **/

      /**
       * Bits 26, 20, 10, 4 CHTIFx: Stream x clear half transfer interrupt flag (x = 7..4)
       * Writing 1 to this bit clears the corresponding HTIFx flag in the DMA_HISR register
       **/

      /**
       * Bits 25, 19, 9, 3 CTEIFx: Stream x clear transfer error interrupt flag (x = 7..4)
       * Writing 1 to this bit clears the corresponding TEIFx flag in the DMA_HISR register
       **/

      /**
       * Bits 24, 18, 8, 2 CDMEIFx: Stream x clear direct mode error interrupt flag (x = 7..4)
       * Writing 1 to this bit clears the corresponding DMEIFx flag in the DMA_HISR register
       **/

      /**
       * Bits 23, 17, 7, 1 Reserved, must be kept at reset value.
       **/

      /**
       * Bits 22, 16, 6, 0 CFEIFx: Stream x clear FIFO error interrupt flag (x = 7..4)
       * Writing 1 to this bit clears the corresponding CFEIFx flag in the DMA_HISR register
       **/


    }  // namespace hifcr

  #endif // STM32F1XX
  }  // namespace common

#ifdef STM32F1XX
  namespace channel {
    enum Address {
      CHANNEL_1 = 0x08,
      CHANNEL_2 = 0x1C,
      CHANNEL_3 = 0x30,
      CHANNEL_4 = 0x44,
      CHANNEL_5 = 0x58,
      CHANNEL_6 = 0x6C,
      CHANNEL_7 = 0x80,
    };

    struct Registers {
      __RW
      u32 CCR;  // 0x00: Configuration
      __RW
      u32 CNDTR;// 0x04: Number of data
      __RW
      u32 CPAR;// 0x08: Peripheral address
      __RW
      u32 CMAR;// 0x0C: Memory address
    };

    namespace cr {
      enum {
        OFFSET = 0x00
      };
      namespace en {
        enum {
          POSITION = 0,
          MASK = 1 << POSITION
        };
        enum States {
          CHANNEL_DISABLED = 0 << POSITION,
          CHANNEL_ENABLED = 1 << POSITION,
        };
      }  // namespace en

      namespace tcie {
        enum {
          POSITION = 1,
          MASK = 1 << POSITION
        };
        enum States {
          TRANSFER_COMPLETE_INTERRUPT_DISABLED = 0 << POSITION,
          TRANSFER_COMPLETE_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace tcie

      namespace htie {
        enum {
          POSITION = 2,
          MASK = 1 << POSITION
        };
        enum States {
          HALF_TRANSFER_INTERRUPT_DISABLED = 0 << POSITION,
          HALF_TRANSFER_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace htie

      namespace teie {
        enum {
          POSITION = 3,
          MASK = 1 << POSITION
        };
        enum States {
          TRANSFER_ERROR_INTERRUPT_DISABLED = 0 << POSITION,
          TRANSFER_ERROR_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace teie

      namespace dir {
        enum {
          POSITION = 4,
          MASK = 1 << POSITION
        };
        enum States {
          READ_FROM_PERIPHERAL = 0 << POSITION,
          READ_FROM_MEMORY = 1 << POSITION,
        };
      }  // namespace dir

      namespace circ {
        enum {
          POSITION = 5,
          MASK = 1 << POSITION
        };
        enum States {
          CIRCULAR_MODE_DISABLED = 0 << POSITION,
          CIRCULAR_MODE_ENABLED = 1 << POSITION,
        };
      }  // namespace circ

      namespace pinc {
        enum {
          POSITION = 6,
          MASK = 1 << POSITION
        };
        enum States {
          PERIPHERAL_INCREMENT_MODE_DISABLED = 0 << POSITION,
          PERIPHERAL_INCREMENT_MODE_ENABLED = 1 << POSITION,
        };
      }  // namespace pinc

      namespace minc {
        enum {
          POSITION = 7,
          MASK = 1 << POSITION
        };
        enum States {
          MEMORY_INCREMENT_MODE_DISABLED = 0 << POSITION,
          MEMORY_INCREMENT_MODE_ENABLED = 1 << POSITION,
        };
      }  // namespace minc

      namespace psize {
        enum {
          POSITION = 8,
          MASK = 0b11 << POSITION
        };
        enum States {
          PERIPHERAL_SIZE_8BITS = 0 << POSITION,
          PERIPHERAL_SIZE_16BITS = 1 << POSITION,
          PERIPHERAL_SIZE_32BITS = 2 << POSITION,
        };
      }  // namespace psize

      namespace msize {
        enum {
          POSITION = 10,
          MASK = 0b11 << POSITION
        };
        enum States {
          MEMORY_SIZE_8BITS = 0 << POSITION,
          MEMORY_SIZE_16BITS = 1 << POSITION,
          MEMORY_SIZE_32BITS = 2 << POSITION,
        };
      }  // namespace msize

      namespace pl {
        enum {
          POSITION = 12,
          MASK = 0b11 << POSITION
        };
        enum States {
          CHANNEL_PRIORITY_LEVEL_LOW = 0 << POSITION,
          CHANNEL_PRIORITY_LEVEL_MEDIUM = 1 << POSITION,
          CHANNEL_PRIORITY_LEVEL_HIGH = 2 << POSITION,
          CHANNEL_PRIORITY_LEVEL_VERY_HIGH = 3 << POSITION,
        };
      }  // namespace pl

      namespace mem2mem {
        enum {
          POSITION = 14,
          MASK = 1 << POSITION
        };
        enum States {
          MEMORY_TO_MEMORY_MODE_DISABLED = 0 << POSITION,
          MEMORY_TO_MEMORY_MODE_ENABLED = 1 << POSITION,
        };
      }  // namespace mem2mem
    }  // namespace cr

    namespace ndtr {
      enum {
        OFFSET = 0x04
      };
    }  // namespace ndtr

    namespace par {
      enum {
        OFFSET = 0x08
      };
    }  // namespace par

    namespace mar {
      enum {
        OFFSET = 0x0C
      };
    }  // namespace mar
  }  // namespace channel

#else // STM32F1XX



  namespace stream {
    enum Address {
      STREAM_0 = 0x10,
      STREAM_1 = 0x28,
      STREAM_2 = 0x40,
      STREAM_3 = 0x58,
      STREAM_4 = 0x70,
      STREAM_5 = 0x88,
      STREAM_6 = 0xA0,
      STREAM_7 = 0xB8,
    };

    struct Registers {
        __RW
        u32 CR;  // 0x00: Configuration
        __RW
        u32 NDTR;  // 0x04: Number of data
        __RW
        u32 PAR;  // 0x08: Peripheral address
        __RW
        u32 M0AR;  // 0x0C: Memory 0 address
        __RW
        u32 M1AR;  // 0x10: Memory 1 address
        __RW
        u32 FCR;  // 0x14: FIFO control
    };

    /**
     * DMA stream x configuration register (DMA_SxCR) (x = 0..7)
     * This register is used to configure the concerned stream.
     * Address offset: 0x10 + 0x18 × stream number
     * Reset value: 0x0000 0000
     **/
    namespace cr {
      enum {
        OFFSET = 0x00  // stream::address already has the 0x10 offset
      };

      /**
       * Bits 0 EN: Stream enable / flag stream ready when read low
       * This bit is set and cleared by software.
       * 0: Stream disabled
       * 1: Stream enabled
       * This bit may be cleared by hardware:
       * – on a DMA end of transfer (stream ready to be configured)
       * – if a transfer error occurs on the AHB master buses
       * – when the FIFO threshold on memory AHB port is not compatible with the size of the
       * burst
       * When this bit is read as 0, the software is allowed to program the Configuration and FIFO
       * bits registers. It is forbidden to write these registers when the EN bit is read as 1.
       * Note: Before setting EN bit to '1' to start a new transfer, the event flags corresponding to the
       * stream in DMA_LISR or DMA_HISR register must be cleared
       **/
      namespace en {
        enum {
          POSITION = 0,
          MASK = 1 << POSITION
        };
        enum States {
          STREAM_DISABLED = 0 << POSITION,
          STREAM_ENABLED = 1 << POSITION,
        };
      }  // namespace en

      /**
       * Bits 1 DMEIE: Direct mode error interrupt enable
       * This bit is set and cleared by software.
       * 0: DME interrupt disabled
       * 1: DME interrupt enabled
       **/
      namespace dmeie {
        enum {
          POSITION = 1,
          MASK = 1 << POSITION
        };
        enum States {
          DIRECT_MODE_ERROR_INTERRUPT_DISABLED = 0 << POSITION,
          DIRECT_MODE_ERROR_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace dmeie

      /**
       * Bits 2 TEIE: Transfer error interrupt enable
       * This bit is set and cleared by software.
       * 0: TE interrupt disabled
       * 1: TE interrupt enabled
       **/
      namespace teie {
        enum {
          POSITION = 2,
          MASK = 1 << POSITION
        };
        enum States {
          TRANSFER_ERROR_INTERRUPT_DISABLED = 0 << POSITION,
          TRANSFER_ERROR_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace teie

      /**
       * Bits 3 HTIE: Half transfer interrupt enable
       * This bit is set and cleared by software.
       * 0: HT interrupt disabled
       * 1: HT interrupt enabled
       **/
      namespace htie {
        enum {
          POSITION = 3,
          MASK = 1 << POSITION
        };
        enum States {
          HALF_TRANSFER_INTERRUPT_DISABLED = 0 << POSITION,
          HALF_TRANSFER_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace htie

      /**
       * Bits 4 TCIE: Transfer complete interrupt enable
       * This bit is set and cleared by software.
       * 0: TC interrupt disabled
       * 1: TC interrupt enabled
       **/
      namespace tcie {
        enum {
          POSITION = 4,
          MASK = 1 << POSITION
        };
        enum States {
          TRANSFER_COMPLETE_INTERRUPT_DISABLED = 0 << POSITION,
          TRANSFER_COMPLETE_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace tcie

      /**
       * Bits 5 PFCTRL: Peripheral flow controller
       * This bit is set and cleared by software.
       * 0: The DMA is the flow controller
       * 1: The peripheral is the flow controller
       * This bit is protected and can be written only if EN is ‘0’.
       * When the memory-to-memory mode is selected (bits DIR[1:0]=10), then this bit is
       * automatically forced to 0 by hardware.
       **/
      namespace pfctrl {
        enum {
          POSITION = 5,
          MASK = 1 << POSITION
        };
        enum States {
          DMA_FLOW_CONTROLLER = 0 << POSITION,
          PERIPHERAL_FLOW_CONTROLLER = 1 << POSITION,
        };
      }  // namespace pfctrl

      /**
       * Bits 7:6 DIR[1:0]: Data transfer direction
       * These bits are set and cleared by software.
       * 00: Peripheral-to-memory
       * 01: Memory-to-peripheral
       * 10: Memory-to-memory
       * 11: reserved
       * These bits are protected and can be written only if EN is ‘0’.
       **/
      namespace dir {
        enum {
          POSITION = 6,
          MASK = 0b11 << POSITION
        };
        enum States {
          PERIPHERAL_TO_MEMORY = 0 << POSITION,
          MEMORY_TO_PERIPHERAL = 1 << POSITION,
          MEMORY_TO_MEMORY = 2 << POSITION,
        };
      }  // namespace dir

      /**
       * Bits 8 CIRC: Circular mode
       * This bit is set and cleared by software and can be cleared by hardware.
       * 0: Circular mode disabled
       * 1: Circular mode enabled
       * When the peripheral is the flow controller (bit PFCTRL=1) and the stream is enabled (bit
       * EN=1), then this bit is automatically forced by hardware to 0.
       * It is automatically forced by hardware to 1 if the DBM bit is set, as soon as the stream is
       * enabled (bit EN ='1').
       **/
      namespace circ {
        enum {
          POSITION = 8,
          MASK = 1 << POSITION
        };
        enum States {
          CIRCULAR_MODE_DISABLED = 0 << POSITION,
          CIRCULAR_MODE_ENABLED = 1 << POSITION,
        };
      }  // namespace circ

      /**
       * Bits 9 PINC: Peripheral increment mode
       * This bit is set and cleared by software.
       * 0: Peripheral address pointer is fixed
       * 1: Peripheral address pointer is incremented after each data transfer (increment is done
       * according to PSIZE)
       * This bit is protected and can be written only if EN is ‘0’.
       **/
      namespace pinc {
        enum {
          POSITION = 9,
          MASK = 1 << POSITION
        };
        enum States {
          PERIPHERAL_INCREMENT_MODE_DISABLED = 0 << POSITION,
          PERIPHERAL_INCREMENT_MODE_ENABLED = 1 << POSITION,
        };
      }  // namespace pinc

      /**
       * Bits 10 MINC: Memory increment mode
       * This bit is set and cleared by software.
       * 0: Memory address pointer is fixed
       * 1: Memory address pointer is incremented after each data transfer (increment is done
       * according to MSIZE)
       * This bit is protected and can be written only if EN is ‘0’.
       **/
      namespace minc {
        enum {
          POSITION = 10,
          MASK = 1 << POSITION
        };
        enum States {
          MEMORY_INCREMENT_MODE_DISABLED = 0 << POSITION,
          MEMORY_INCREMENT_MODE_ENABLED = 1 << POSITION,
        };
      }  // namespace minc

      /**
       * Bits 12:11 PSIZE[1:0]: Peripheral data size
       * These bits are set and cleared by software.
       * 00: Byte (8-bit)
       * 01: Half-word (16-bit)
       * 10: Word (32-bit)
       * 11: reserved
       * These bits are protected and can be written only if EN is ‘0’
       **/
      namespace psize {
        enum {
          POSITION = 11,
          MASK = 0b11 << POSITION
        };
        enum States {
          PERIPHERAL_SIZE_8BITS = 0 << POSITION,
          PERIPHERAL_SIZE_16BITS = 1 << POSITION,
          PERIPHERAL_SIZE_32BITS = 2 << POSITION,
        };
      }  // namespace psize

      /**
       * Bits 14:13 MSIZE[1:0]: Memory data size
       * These bits are set and cleared by software.
       * 00: byte (8-bit)
       * 01: half-word (16-bit)
       * 10: word (32-bit)
       * 11: reserved
       * These bits are protected and can be written only if EN is ‘0’.
       * In direct mode, MSIZE is forced by hardware to the same value as PSIZE as soon as bit EN
       * = '1'.
       **/
      namespace msize {
        enum {
          POSITION = 13,
          MASK = 0b11 << POSITION
        };
        enum States {
          MEMORY_SIZE_8BITS = 0 << POSITION,
          MEMORY_SIZE_16BITS = 1 << POSITION,
          MEMORY_SIZE_32BITS = 2 << POSITION,
        };
      }  // namespace msize

      /**
       * Bits 15 PINCOS: Peripheral increment offset size
       * This bit is set and cleared by software
       * 0: The offset size for the peripheral address calculation is linked to the PSIZE
       * 1: The offset size for the peripheral address calculation is fixed to 4 (32-bit alignment).
       * This bit has no meaning if bit PINC = '0'.
       * This bit is protected and can be written only if EN = '0'.1
       * This bit is forced low by hardware when the stream is enabled (bit EN = '1') if the direct
       * mode is selected or if PBURST are different from “00”.
       **/
      namespace pincos {
        enum {
          POSITION = 15,
          MASK = 1 << POSITION
        };
        enum States {
          PERIPHERAL_INCREMENT_OFFSET_SIZE_PSIZE = 0 << POSITION,
          PERIPHERAL_INCREMENT_OFFSET_SIZE_32BITS = 1 << POSITION,
        };
      }  // namespace pincos

      /**
       * Bits 17:16 PL[1:0]: Priority level
       * These bits are set and cleared by software.
       * 00: Low
       * 01: Medium
       * 10: High
       * 11: Very high
       * These bits are protected and can be written only if EN is ‘0’.
       **/
      namespace pl {
        enum {
          POSITION = 16,
          MASK = 0b11 << POSITION
        };
        enum States {
          PRIORITY_LEVEL_LOW = 0 << POSITION,
          PRIORITY_LEVEL_MEDIUM = 1 << POSITION,
          PRIORITY_LEVEL_HIGH = 2 << POSITION,
          PRIORITY_LEVEL_VERY_HIGH = 3 << POSITION,
        };
      }  // namespace pl

      /**
       * Bits 18 DBM: Double buffer mode
       * This bits is set and cleared by software.
       * 0: No buffer switching at the end of transfer
       * 1: Memory target switched at the end of the DMA transfer
       * This bit is protected and can be written only if EN is ‘0’.
       **/
      namespace dbm {
        enum {
          POSITION = 18,
          MASK = 1 << POSITION
        };
        enum States {
          DOUBLE_BUFFER_MODE_DISABLED = 0 << POSITION,
          DOUBLE_BUFFER_MODE_ENABLED = 1 << POSITION,
        };
      }  // namespace dbm

      /**
       * Bits 19 CT: Current target (only in double buffer mode)
       * This bits is set and cleared by hardware. It can also be written by software.
       * 0: The current target memory is Memory 0 (addressed by the DMA_SxM0AR pointer)
       * 1: The current target memory is Memory 1 (addressed by the DMA_SxM1AR pointer)
       * This bit can be written only if EN is ‘0’ to indicate the target memory area of the first transfer.
       * Once the stream is enabled, this bit operates as a status flag indicating which memory area
       * is the current target.
       **/
      namespace ct {
        enum {
          POSITION = 19,
          MASK = 1 << POSITION
        };
        enum States {
          CURRENT_TARGET_MEMORY_0 = 0 << POSITION,
          CURRENT_TARGET_MEMORY_1 = 1 << POSITION,
        };
      }  // namespace ct

      /**
       * Bits 20 Reserved, must be kept at reset value.
       **/

      /**
       * Bits 22:21 PBURST[1:0]: Peripheral burst transfer configuration
       * These bits are set and cleared by software.
       * 00: single transfer
       * 01: INCR4 (incremental burst of 4 beats)
       * 10: INCR8 (incremental burst of 8 beats)
       * 11: INCR16 (incremental burst of 16 beats)
       * These bits are protected and can be written only if EN is ‘0’
       * In direct mode, these bits are forced to 0x0 by hardware.
       **/
      namespace pburst {
        enum {
          POSITION = 21,
          MASK = 0b11 << POSITION
        };
        enum States {
          PERIPHERAL_BURST_TRANSFER_SINGLE = 0 << POSITION,
          PERIPHERAL_BURST_TRANSFER_4BEATS = 1 << POSITION,
          PERIPHERAL_BURST_TRANSFER_8BEATS = 2 << POSITION,
          PERIPHERAL_BURST_TRANSFER_16BEATS = 3 << POSITION,
        };
      }  // namespace pburst

      /**
       * Bits 24:23 MBURST: Memory burst transfer configuration
       * These bits are set and cleared by software.
       * 00: single transfer
       * 01: INCR4 (incremental burst of 4 beats)
       * 10: INCR8 (incremental burst of 8 beats)
       * 11: INCR16 (incremental burst of 16 beats)
       * These bits are protected and can be written only if EN is ‘0’
       * In direct mode, these bits are forced to 0x0 by hardware as soon as bit EN= '1'.
       **/
      namespace mburst {
        enum {
          POSITION = 23,
          MASK = 0b11 << POSITION
        };
        enum States {
          MEMORY_BURST_TRANSFER_SINGLE = 0 << POSITION,
          MEMORY_BURST_TRANSFER_4BEATS = 1 << POSITION,
          MEMORY_BURST_TRANSFER_8BEATS = 2 << POSITION,
          MEMORY_BURST_TRANSFER_16BEATS = 3 << POSITION,
        };
      }  // namespace mburst

      /**
       * Bits 27:25 CHSEL[2:0]: Channel selection
       * These bits are set and cleared by software.
       * 000: channel 0 selected
       * 001: channel 1 selected
       * 010: channel 2 selected
       * 011: channel 3 selected
       * 100: channel 4 selected
       * 101: channel 5 selected
       * 110: channel 6 selected
       * 111: channel 7 selected
       * These bits are protected and can be written only if EN is ‘0’
       **/
      namespace chsel {
        enum {
          POSITION = 25,
          MASK = 0b111 << POSITION
        };
        enum States {
          CHANNEL_0 = 0 << POSITION,
          CHANNEL_1 = 1 << POSITION,
          CHANNEL_2 = 2 << POSITION,
          CHANNEL_3 = 3 << POSITION,
          CHANNEL_4 = 4 << POSITION,
          CHANNEL_5 = 5 << POSITION,
          CHANNEL_6 = 6 << POSITION,
          CHANNEL_7 = 7 << POSITION,
        };
      }  // namespace chsel

      /**
       * Bits 31:28 Reserved, must be kept at reset value.
       **/
    }  // namespace cr

    /**
     * DMA stream x number of data register (DMA_SxNDTR) (x = 0..7)
     * Address offset: 0x14 + 0x18 × stream number
     * Reset value: 0x0000 0000
     **/
    namespace ndtr {
      enum {
        OFFSET = 0x04 // stream::address already has the 0x10 offset
      };

      /**
       * Bits 31:16 Reserved, must be kept at reset value.
       **/

      /**
       * Bits 15:0 NDT[15:0]: Number of data items to transfer
       * Number of data items to be transferred (0 up to 65535). This register can be written only
       * when the stream is disabled. When the stream is enabled, this register is read-only,
       * indicating the remaining data items to be transmitted. This register decrements after each
       * DMA transfer.
       * Once the transfer has completed, this register can either stay at zero (when the stream is in
       * normal mode) or be reloaded automatically with the previously programmed value in the
       * following cases:
       * – when the stream is configured in Circular mode.
       * – when the stream is enabled again by setting EN bit to '1'
       * If the value of this register is zero, no transaction can be served even if the stream is
       * enabled.
       **/
    }  // namespace ndtr

    /**
     * DMA stream x peripheral address register (DMA_SxPAR) (x = 0..7)
     * Address offset: 0x18 + 0x18 × stream number
     * Reset value: 0x0000 0000
     **/
    namespace par {
      enum {
        OFFSET = 0x08 // stream::address already has the 0x10 offset
      };

      /**
       * Bits 31:0 PAR[31:0]: Peripheral address
       * Base address of the peripheral data register from/to which the data will be read/written.
       * These bits are write-protected and can be written only when bit EN = '0' in the DMA_SxCR register.
       **/
    }  // namespace par


    /**
     * DMA stream x memory 0 address register (DMA_SxM0AR) (x = 0..7)
     * Address offset: 0x1C + 0x18 × stream number
     * Reset value: 0x0000 0000
     **/
    namespace m0ar {
      enum {
        OFFSET = 0x0C // stream::address already has the 0x10 offset
      };

      /**
       * Bits 31:0 M0A[31:0]: Memory 0 address
       * Base address of Memory area 0 from/to which the data will be read/written.
       * These bits are write-protected. They can be written only if:
       * – the stream is disabled (bit EN= '0' in the DMA_SxCR register) or
       * – the stream is enabled (EN=’1’ in DMA_SxCR register) and bit CT = '1' in the
       * DMA_SxCR register (in Double buffer mode).
       **/
    }  // namespace m0ar

    /**
     * DMA stream x memory 1 address register (DMA_SxM1AR) (x = 0..7)
     * Address offset: 0x20 + 0x18 × stream number
     * Reset value: 0x0000 0000
     **/
    namespace m1ar {
      enum {
        OFFSET = 0x10 // stream::address already has the 0x10 offset
      };

      /**
       * Bits 31:0 M1A[31:0]: Memory 1 address (used in case of Double buffer mode)
       * Base address of Memory area 1 from/to which the data will be read/written.
       * This register is used only for the Double buffer mode.
       * These bits are write-protected. They can be written only if:
       * – the stream is disabled (bit EN= '0' in the DMA_SxCR register) or
       * – the stream is enabled (EN=’1’ in DMA_SxCR register) and bit CT = '0' in the
       * DMA_SxCR register.
       **/
    }  // namespace m1ar

    /**
     * DMA stream x FIFO control register (DMA_SxFCR) (x = 0..7)
     * Address offset: 0x24 + 0x24 × stream number
     * Reset value: 0x0000 0021
     **/
    namespace fcr {
      enum {
        OFFSET = 0x14 // stream::address already has the 0x10 offset
      };

      /**
       * Bits 1:0 FTH[1:0]: FIFO threshold selection
       * These bits are set and cleared by software.
       * 00: 1/4 full FIFO
       * 01: 1/2 full FIFO
       * 10: 3/4 full FIFO
       * 11: full FIFO
       * These bits are not used in the direct mode when the DMIS value is zero.
       * These bits are protected and can be written only if EN is ‘1’.
       **/
      namespace fth {
        enum {
          POSITION = 0,
          MASK = 0b11 << POSITION
        };
        enum States {
          FIFO_THRESHOLD_SELECTION_1_OVER_4 = 0 << POSITION,
          FIFO_THRESHOLD_SELECTION_2_OVER_4 = 1 << POSITION,
          FIFO_THRESHOLD_SELECTION_3_OVER_4 = 2 << POSITION,
          FIFO_THRESHOLD_SELECTION_FULL = 3 << POSITION,
        };
      }  // namespace fth

      /**
       * Bits 2 DMDIS: Direct mode disable
       * This bit is set and cleared by software. It can be set by hardware.
       * 0: Direct mode enabled
       * 1: Direct mode disabled
       * This bit is protected and can be written only if EN is ‘0’.
       * This bit is set by hardware if the memory-to-memory mode is selected (DIR bit in
       * DMA_SxCR are “10”) and the EN bit in the DMA_SxCR register is ‘1’ because the direct
       * mode is not allowed in the memory-to-memory configuration.
       **/
      namespace dmdis {
        enum {
          POSITION = 2,
          MASK = 1 << POSITION
        };
        enum States {
          DIRECT_MODE_ENABLED = 0 << POSITION,
          DIRECT_MODE_DISABLED = 1 << POSITION,
        };
      }  // namespace dmdis

      /**
       * Bits 5:3 FS[2:0]: FIFO status
       * These bits are read-only.
       * 000: 0 < fifo_level < 1/4
       * 001: 1/4 <= fifo_level < 1/2
       * 010: 1/2 <= fifo_level < 3/4
       * 011: 3/4 <= fifo_level < full
       * 100: FIFO is empty
       * 101: FIFO is full
       * others: no meaning
       * These bits are not relevant in the direct mode (DMDIS bit is zero).
       **/
      namespace fs {
        enum {
          POSITION = 3,
          MASK = 0b111 << POSITION
        };
        enum States{
        	FIFO_LEVEL_BETWEEN_EMPTY_AND_QUARTER = 0,
        	FIFO_LEVEL_BETWEEN_QUARTER_AND_HALF = 1,
        	FIFO_LEVEL_BETWEEN_HALF_AND_THREEQUARTERS = 2,
        	FIFO_LEVEL_BETWEEN_THREEQUARTERS_AND_FULL = 3,
        	FIFO_LEVEL_EMPTY = 4,
        	FIFO_LEVEL_FULL = 5,
        };
      }  // namespace fs

      /**
       * Bits 6 Reserved, must be kept at reset value.
       **/

      /**
       * Bits 7 FEIE: FIFO error interrupt enable
       * This bit is set and cleared by software.
       * 0: FE interrupt disabled
       * 1: FE interrupt enabled
       **/
      namespace feie {
        enum {
          POSITION = 7,
          MASK = 1 << POSITION
        };
        enum States {
          FIFO_ERROR_INTERRUPT_DISABLED = 0 << POSITION,
          FIFO_ERROR_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace feie

      /**
       * Bits 31:8 Reserved, must be kept at reset value.
       **/
    }  // namespace fcr
  }  // namespace stream
#endif // STM32F1XX
}  // namespace dma
