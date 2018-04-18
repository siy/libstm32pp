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

namespace fsmc {
enum {
    ADDRESS = alias::FSMC
};

struct Registers {
    struct { // SRAM/NOR-Flash chip-select register
        __RW
        u32 BCR; // control
        __RW
        u32 BTR; // timing
    } NorPsram[4];
    u32 _RESERVED0[20]; // 0x20 - 0x5C
    struct {
        __RW
        u32 PCR; // 0x60: PC Card/NAND Flash control register 2
        __RW
        u32 SR; // 0x64: FIFO status and interrupt register 2
        __RW
        u32 PMEM; // 0x68: Common memory space timing register 2
        __RW
        u32 PATT; // 0x6C: Attribute memory space timing register 2
        u32 _RESERVED1;
        __R
        u32 ECCR; // 0x74: ECC result register 2
        u32 _RESERVED2[2];
    } Nand[2];
    struct {
        __RW
        u32 PCR; // 0xA0: PC Card/NAND Flash control register 4
        __RW
        u32 SR; // 0xA4: FIFO status and interrupt register 4
        __RW
        u32 PMEM; // 0xA8: Common memory space timing register 4
        __RW
        u32 PATT; // 0xAC: Attribute memory space timing register 4
        __RW
        u32 PIO; // 0xB0: I/O space timing register 4
    } Pccard;
    u32 _RESERVED3[19];
    struct {
        __RW
        u32 BWTR; // 0x104: SRAM/NOR-Flash write timing register 1
        u32 _RESERVED4;
    } NorPsramTiming[4];
};

namespace norpsram {
enum Device {
    NORPSRAM1 = 0,
    NORPSRAM2 = 1,
    NORPSRAM3 = 2,
    NORPSRAM4 = 3
};

const u32 DataAddress[4] = {
    0x60000000,
    0x64000000,
    0x68000000,
    0x6C000000
};

} // namespace norpsram

namespace bcr {
enum {
    OFFSET = 0x00
};

namespace mbken {
enum {
    POSITION = 0,
    MASK = 1 << POSITION
};
enum States {
    MEMORY_BANK_DISABLED = 0 << POSITION,
    MEMORY_BANK_ENABLED = 1 << POSITION
};
}

namespace muxen {
enum {
    POSITION = 1,
    MASK = 1 << POSITION
};
enum States {
    DATABUS_DATA_ONLY = 0 << POSITION,
    DATABUS_BOTH_ADDRESS_AND_DATA = 1 << POSITION
};
}

namespace mtyp {
enum {
    POSITION = 2,
    MASK = 3 << POSITION
};
enum States {
    TYPE_IS_SRAM_ROM = 0 << POSITION,
    TYPE_IS_PSRAM = 1 << POSITION,
    TYPE_IS_NOR_NAND = 2 << POSITION
};
}

namespace mwid {
enum {
    POSITION = 4,
    MASK = 3 << POSITION
};
enum States {
    BUSWIDTH_8BIT = 0 << POSITION,
    BUSWIDTH_16BIT = 1 << POSITION
};
}

namespace faccen {
enum {
    POSITION = 6,
    MASK = 1 << POSITION
};
enum States {
    NOR_ACCESS_DISABLED = 0 << POSITION,
    NOR_ACCESS_ENABLED = 1 << POSITION
};
}
namespace bursten {
enum {
    POSITION = 8,
    MASK = 1 << POSITION
};
enum States {
    BURST_DISABLED = 0 << POSITION,
    BURST_ENABLED = 1 << POSITION
};
}
namespace waitpol {
enum {
    POSITION = 9,
    MASK = 1 << POSITION
};
enum States {
    NWAIT_ACTIVE_LOW = 0 << POSITION,
    NWAIT_ACTIVE_HIGH = 1 << POSITION
};
}
namespace wrapmod {
enum {
    POSITION = 10,
    MASK = 1 << POSITION
};
enum States {
    WRAPPED_BURST_DISABLED = 0 << POSITION,
    WRAPPED_BURST_ENABLED = 1 << POSITION
};
}
namespace waitcfg {
enum {
    POSITION = 11,
    MASK = 1 << POSITION
};
enum States {
    NWAIT_BEFORE_WAIT_STATE = 0 << POSITION,
    NWAIT_WHILE_WAIT_STATE = 1 << POSITION
};
}
namespace wren {
enum {
    POSITION = 12,
    MASK = 1 << POSITION
};
enum States {
    WRITE_DISABLED = 0 << POSITION,
    WRITE_ENABLED = 1 << POSITION
};
}
namespace waiten {
enum {
    POSITION = 13,
    MASK = 1 << POSITION
};
enum States {
    NWAIT_DISABLED = 0 << POSITION,
    NWAIT_ENABLED = 1 << POSITION
};
}
namespace extmod {
enum {
    POSITION = 14,
    MASK = 1 << POSITION
};
enum States {
    BWTR_REGISTER_IGNORED = 0 << POSITION,
    BWTR_REGISTER_USED = 1 << POSITION
};
}
namespace asyncwait {
enum {
    POSITION = 15,
    MASK = 1 << POSITION
};
enum States {
    NWAIT_FOR_ASYNC_DISABLED = 0 << POSITION,
    NWAIT_FOR_ASYNC_ENABLED = 1 << POSITION
};
}
namespace cburstrw {
enum {
    POSITION = 19,
    MASK = 1 << POSITION
};
enum States {
    WRITE_ALWAYS_ASYNC = 0 << POSITION,
    WRITE_ALWAYS_SYNC = 1 << POSITION
};
}
}

namespace btr {
namespace addset {
enum {
    POSITION = 0,
    MASK = 15 << POSITION
};
enum States {
    PHASE_DURATION_0xHCLK = 0 << POSITION,
    PHASE_DURATION_1xHCLK = 1 << POSITION,
    PHASE_DURATION_2xHCLK = 2 << POSITION,
    PHASE_DURATION_3xHCLK = 3 << POSITION,
    PHASE_DURATION_4xHCLK = 4 << POSITION,
    PHASE_DURATION_5xHCLK = 5 << POSITION,
    PHASE_DURATION_6xHCLK = 6 << POSITION,
    PHASE_DURATION_7xHCLK = 7 << POSITION,
    PHASE_DURATION_8xHCLK = 8 << POSITION,
    PHASE_DURATION_9xHCLK = 9 << POSITION,
    PHASE_DURATION_10xHCLK = 10 << POSITION,
    PHASE_DURATION_11xHCLK = 11 << POSITION,
    PHASE_DURATION_12xHCLK = 12 << POSITION,
    PHASE_DURATION_13xHCLK = 13 << POSITION,
    PHASE_DURATION_14xHCLK = 14 << POSITION,
    PHASE_DURATION_15xHCLK = 15 << POSITION
};
}
namespace addhld {
enum {
    POSITION = 4,
    MASK = 15 << POSITION
};
enum States {
    PHASE_DURATION_1xHCLK = 1 << POSITION,
    PHASE_DURATION_2xHCLK = 2 << POSITION,
    PHASE_DURATION_3xHCLK = 3 << POSITION,
    PHASE_DURATION_4xHCLK = 4 << POSITION,
    PHASE_DURATION_5xHCLK = 5 << POSITION,
    PHASE_DURATION_6xHCLK = 6 << POSITION,
    PHASE_DURATION_7xHCLK = 7 << POSITION,
    PHASE_DURATION_8xHCLK = 8 << POSITION,
    PHASE_DURATION_9xHCLK = 9 << POSITION,
    PHASE_DURATION_10xHCLK = 10 << POSITION,
    PHASE_DURATION_11xHCLK = 11 << POSITION,
    PHASE_DURATION_12xHCLK = 12 << POSITION,
    PHASE_DURATION_13xHCLK = 13 << POSITION,
    PHASE_DURATION_14xHCLK = 14 << POSITION,
    PHASE_DURATION_15xHCLK = 15 << POSITION
};
}
namespace datast {
enum {
    POSITION = 8,
    MASK = 255 << POSITION
};
enum States {
    PHASE_DURATION_1xHCLK = 0x1 << POSITION,
    PHASE_DURATION_2xHCLK = 0x2 << POSITION,
    PHASE_DURATION_3xHCLK = 0x3 << POSITION,
    PHASE_DURATION_4xHCLK = 0x4 << POSITION,
    PHASE_DURATION_5xHCLK = 0x5 << POSITION,
    PHASE_DURATION_6xHCLK = 0x6 << POSITION,
    PHASE_DURATION_7xHCLK = 0x7 << POSITION,
    PHASE_DURATION_8xHCLK = 0x8 << POSITION,
    PHASE_DURATION_9xHCLK = 0x9 << POSITION,
    PHASE_DURATION_10xHCLK = 0xA << POSITION
};
}
namespace busturn {
enum {
    POSITION = 16,
    MASK = 15 << POSITION
};
enum States {
    PHASE_DURATION_0xHCLK = 0 << POSITION,
    PHASE_DURATION_1xHCLK = 1 << POSITION,
    PHASE_DURATION_2xHCLK = 2 << POSITION,
    PHASE_DURATION_3xHCLK = 3 << POSITION,
    PHASE_DURATION_4xHCLK = 4 << POSITION,
    PHASE_DURATION_5xHCLK = 5 << POSITION,
    PHASE_DURATION_6xHCLK = 6 << POSITION,
    PHASE_DURATION_7xHCLK = 7 << POSITION,
    PHASE_DURATION_8xHCLK = 8 << POSITION,
    PHASE_DURATION_9xHCLK = 9 << POSITION,
    PHASE_DURATION_10xHCLK = 10 << POSITION,
    PHASE_DURATION_11xHCLK = 11 << POSITION,
    PHASE_DURATION_12xHCLK = 12 << POSITION,
    PHASE_DURATION_13xHCLK = 13 << POSITION,
    PHASE_DURATION_14xHCLK = 14 << POSITION,
    PHASE_DURATION_15xHCLK = 15 << POSITION
};
}
namespace clkdiv {
  enum {
      POSITION = 20,
      MASK = 15 << POSITION
  };
  enum States {
      CLK_PERIOD_2xHCLK = 1 << POSITION,
      CLK_PERIOD_3xHCLK = 2 << POSITION,
      CLK_PERIOD_4xHCLK = 3 << POSITION,
      CLK_PERIOD_5xHCLK = 4 << POSITION,
      CLK_PERIOD_6xHCLK = 5 << POSITION,
      CLK_PERIOD_7xHCLK = 6 << POSITION,
      CLK_PERIOD_8xHCLK = 7 << POSITION,
      CLK_PERIOD_9xHCLK = 8 << POSITION,
      CLK_PERIOD_10xHCLK = 9 << POSITION,
      CLK_PERIOD_11xHCLK = 10 << POSITION,
      CLK_PERIOD_12xHCLK = 11 << POSITION,
      CLK_PERIOD_13xHCLK = 12 << POSITION,
      CLK_PERIOD_14xHCLK = 13 << POSITION,
      CLK_PERIOD_15xHCLK = 14 << POSITION,
      CLK_PERIOD_16xHCLK = 15 << POSITION
  };
}
namespace datlat {
  enum {
      POSITION = 24,
      MASK = 15 << POSITION
  };
  enum States {
      FIRST_BURST_DATA_LATENCY_2xCLK = 0 << POSITION,
      FIRST_BURST_DATA_LATENCY_3xCLK = 1 << POSITION,
      FIRST_BURST_DATA_LATENCY_4xCLK = 2 << POSITION,
      FIRST_BURST_DATA_LATENCY_5xCLK = 3 << POSITION,
      FIRST_BURST_DATA_LATENCY_6xCLK = 4 << POSITION,
      FIRST_BURST_DATA_LATENCY_7xCLK = 5 << POSITION,
      FIRST_BURST_DATA_LATENCY_8xCLK = 6 << POSITION,
      FIRST_BURST_DATA_LATENCY_9xCLK = 7 << POSITION,
      FIRST_BURST_DATA_LATENCY_10xCLK = 8 << POSITION,
      FIRST_BURST_DATA_LATENCY_11xCLK = 9 << POSITION,
      FIRST_BURST_DATA_LATENCY_12xCLK = 10 << POSITION,
      FIRST_BURST_DATA_LATENCY_13xCLK = 11 << POSITION,
      FIRST_BURST_DATA_LATENCY_14xCLK = 12 << POSITION,
      FIRST_BURST_DATA_LATENCY_15xCLK = 13 << POSITION,
      FIRST_BURST_DATA_LATENCY_16xCLK = 14 << POSITION,
      FIRST_BURST_DATA_LATENCY_17xCLK = 15 << POSITION
  };
}
namespace accmod {
  enum {
      POSITION = 28,
      MASK = 3 << POSITION
  };
  enum States {
      ACCESS_MODE_A = 0 << POSITION,
      ACCESS_MODE_B = 1 << POSITION,
      ACCESS_MODE_C = 2 << POSITION,
      ACCESS_MODE_D = 3 << POSITION
  };
}
}

namespace pcr {
  namespace pwaiten {
    enum {
        POSITION = 1,
        MASK = 1 << POSITION
    };
    enum States {
        WAIT_FEATURE_DISABLED = 0 << POSITION,
        WAIT_FEATURE_ENABLED = 1 << POSITION
    };
  }
  namespace pbken {
    enum {
        POSITION = 2,
        MASK = 1 << POSITION
    };
    enum States {
        MEMORY_BANK_DISABLED = 0 << POSITION,
        MEMORY_BANK_ENABLED = 1 << POSITION
    };
  }
  namespace ptyp {
    enum {
        POSITION = 3,
        MASK = 1 << POSITION
    };
    enum States {
        MEMORY_TYPE_IS_PCCARD = 0 << POSITION,
        MEMORY_TYPE_IS_CF = 0 << POSITION,
        MEMORY_TYPE_IS_CFP = 0 << POSITION,
        MEMORY_TYPE_IS_PCMCIA = 0 << POSITION,
        MEMORY_TYPE_IS_NAND = 1 << POSITION
    };
  }
  namespace pwid {
    enum {
        POSITION = 4,
        MASK = 3 << POSITION
    };
    enum States {
        BUSWIDTH_8BIT = 0 << POSITION,
        BUSWIDTH_16BIT = 1 << POSITION
    };
  }
  namespace eccen {
    enum {
        POSITION = 6,
        MASK = 1 << POSITION
    };
    enum States {
        ECC_LOGIC_DISABLED = 0 << POSITION,
        ECC_LOGIC_ENABLED = 1 << POSITION
    };
  }
  namespace tclr {
    enum {
        POSITION = 9,
        MASK = 15 << POSITION
    };
    enum States {
        CLE_TO_RE_DELAY_1xHCLK = 0 << POSITION,
        CLE_TO_RE_DELAY_2xHCLK = 1 << POSITION,
        CLE_TO_RE_DELAY_3xHCLK = 2 << POSITION,
        CLE_TO_RE_DELAY_4xHCLK = 3 << POSITION,
        CLE_TO_RE_DELAY_5xHCLK = 4 << POSITION,
        CLE_TO_RE_DELAY_6xHCLK = 5 << POSITION,
        CLE_TO_RE_DELAY_7xHCLK = 6 << POSITION,
        CLE_TO_RE_DELAY_8xHCLK = 7 << POSITION,
        CLE_TO_RE_DELAY_9xHCLK = 8 << POSITION,
        CLE_TO_RE_DELAY_10xHCLK = 9 << POSITION,
        CLE_TO_RE_DELAY_11xHCLK = 10 << POSITION,
        CLE_TO_RE_DELAY_12xHCLK = 11 << POSITION,
        CLE_TO_RE_DELAY_13xHCLK = 12 << POSITION,
        CLE_TO_RE_DELAY_14xHCLK = 13 << POSITION,
        CLE_TO_RE_DELAY_15xHCLK = 14 << POSITION,
        CLE_TO_RE_DELAY_16xHCLK = 15 << POSITION
    };
  }
  namespace tar {
    enum {
        POSITION = 13,
        MASK = 15 << POSITION
    };
    enum States {
        ALE_TO_RE_DELAY_1xHCLK = 0 << POSITION,
        ALE_TO_RE_DELAY_2xHCLK = 1 << POSITION,
        ALE_TO_RE_DELAY_3xHCLK = 2 << POSITION,
        ALE_TO_RE_DELAY_4xHCLK = 3 << POSITION,
        ALE_TO_RE_DELAY_5xHCLK = 4 << POSITION,
        ALE_TO_RE_DELAY_6xHCLK = 5 << POSITION,
        ALE_TO_RE_DELAY_7xHCLK = 6 << POSITION,
        ALE_TO_RE_DELAY_8xHCLK = 7 << POSITION,
        ALE_TO_RE_DELAY_9xHCLK = 8 << POSITION,
        ALE_TO_RE_DELAY_10xHCLK = 9 << POSITION,
        ALE_TO_RE_DELAY_11xHCLK = 10 << POSITION,
        ALE_TO_RE_DELAY_12xHCLK = 11 << POSITION,
        ALE_TO_RE_DELAY_13xHCLK = 12 << POSITION,
        ALE_TO_RE_DELAY_14xHCLK = 13 << POSITION,
        ALE_TO_RE_DELAY_15xHCLK = 14 << POSITION,
        ALE_TO_RE_DELAY_16xHCLK = 15 << POSITION
    };
  }
  namespace eccps {
    enum {
        POSITION = 17,
        MASK = 7 << POSITION
    };
    enum States {
        EECC_PAGESIZE_256BYTES = 0 << POSITION,
        EECC_PAGESIZE_512BYTES = 1 << POSITION,
        EECC_PAGESIZE_1024BYTES = 2 << POSITION,
        EECC_PAGESIZE_2048BYTES = 3 << POSITION,
        EECC_PAGESIZE_4096BYTES = 4 << POSITION,
        EECC_PAGESIZE_8192BYTES = 5 << POSITION
    };
  }
}

namespace sr {
  namespace irs {
    enum {
        POSITION = 0,
        MASK = 1 << POSITION
    };
    enum States {
        NO_RISING_EDGE_INTERRUPT_OCCURRED = 0 << POSITION,
        RISING_EDGE_INTERRUPT_OCCURRED = 1 << POSITION
    };
  }
  namespace ils {
    enum {
        POSITION = 1,
        MASK = 1 << POSITION
    };
    enum States {
        NO_HIGHLEVEL_INTERRUPT_OCCURRED = 0 << POSITION,
        HIGHLEVEL_INTERRUPT_OCCURRED = 1 << POSITION
    };
  }
  namespace ifs {
    enum {
        POSITION = 2,
        MASK = 1 << POSITION
    };
    enum States {
        NO_FALLING_EDGE_INTERRUPT_OCCURRED = 0 << POSITION,
        FALLING_EDGE_INTERRUPT_OCCURRED = 1 << POSITION
    };
  }
  namespace iren {
    enum {
        POSITION = 3,
        MASK = 1 << POSITION
    };
    enum States {
        INTERRUPT_RISING_EDGE_DETECT_DISABLED = 0 << POSITION,
        INTERRUPT_RISING_EDGE_DETECT_ENABLED = 1 << POSITION
    };
  }
  namespace ilen {
    enum {
        POSITION = 4,
        MASK = 1 << POSITION
    };
    enum States {
        INTERRUPT_HIGH_LEVEL_DETECT_DISABLED = 0 << POSITION,
        INTERRUPT_HIGH_LEVEL_DETECT_ENABLED = 1 << POSITION
    };
  }
  namespace ifen {
    enum {
        POSITION = 5,
        MASK = 1 << POSITION
    };
    enum States {
        INTERRUPT_FALLING_EDGE_DETECT_DISABLED = 0 << POSITION,
        INTERRUPT_FALLING_EDGE_DETECT_ENABLED = 1 << POSITION
    };
  }
  namespace fempt {
  // this pin is readonly
    enum {
        POSITION = 6,
        MASK = 1 << POSITION
    };
    enum States {
        FIFO_NOT_EMPTY = 0 << POSITION,
        FIFO_IS_EMPTY = 1 << POSITION
    };
  }
}

namespace pmem {

}
namespace patt {

}
namespace pio {

}

namespace eccr {
  namespace eccr_256_byte {
  enum {
      POSITION = 0,
      MASK = 0x3FFFFF << POSITION
  };
  }
  namespace eccr_512_byte {
  enum {
      POSITION = 0,
      MASK = 0xFFFFFF << POSITION
  };
  }
  namespace eccr_1024_byte {
  enum {
      POSITION = 0,
      MASK = 0x3FFFFFF << POSITION
  };
  }
  namespace eccr_2048_byte {
  enum {
      POSITION = 0,
      MASK = 0xFFFFFFF << POSITION
  };
  }
  namespace eccr_4096_byte {
  enum {
      POSITION = 0,
      MASK = 0x3FFFFFFF << POSITION
  };
  }
  namespace eccr_8192_byte {
  enum {
      POSITION = 0,
      MASK = 0xFFFFFFFF << POSITION
  };
  }
}
namespace bwtr {
  namespace addset {

  }
}
// TODO FSMC register bits
}// namespace fsmc
