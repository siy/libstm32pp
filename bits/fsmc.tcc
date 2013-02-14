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

#include "../include/peripheral/rcc.hpp"

namespace fsmc {


  void Functions::enableClock()
  {
#ifndef STM32F1XX
    RCC::enableClocks<rcc::ahb3enr::FSMC>();
#else // STM32F1XX
#ifndef CONNECTIVITY_LINE
    RCC::enableClocks<rcc::ahbenr::FSMC>();
#endif // CONNECTIVITY_LINE
#endif // STM32F1XX
  }

  void Functions::disableClock()
  {
#ifndef STM32F1XX
    RCC::disableClocks<rcc::ahb3enr::FSMC>();
#else // STM32F1XX
#ifndef CONNECTIVITY_LINE
    RCC::disableClocks<rcc::ahbenr::FSMC>();
#endif // CONNECTIVITY_LINE
#endif // STM32F1XX
  }

namespace norpsram {
  template<Device N>
  void Functions<N>::configure(
          fsmc::bcr::mbken::States MBKEN,
          fsmc::bcr::muxen::States MUXEN,
          fsmc::bcr::mtyp::States MTYP,
          fsmc::bcr::mwid::States MWID,
          fsmc::bcr::faccen::States FACCEN,
          fsmc::bcr::bursten::States BURSTEN,
          fsmc::bcr::waitpol::States WAITPOL,
          fsmc::bcr::wrapmod::States WRAPMOD,
          fsmc::bcr::waitcfg::States WAITCFG,
          fsmc::bcr::wren::States WREN,
          fsmc::bcr::waiten::States WAITEN,
          fsmc::bcr::asyncwait::States ASYNCWAIT,
          fsmc::bcr::cburstrw::States CBURSTRW,

          fsmc::btr::addset::States ADDSET,
          fsmc::btr::addhld::States ADDHLD,
          fsmc::btr::datast::States DATAST,
          fsmc::btr::busturn::States BUSTURN,
          fsmc::btr::clkdiv::States CLKDIV,
          fsmc::btr::datlat::States DATLAT,
          fsmc::btr::accmod::States ACCMOD
          )
  {

      FSMC_REGS->NorPsram[N].BCR =
              MBKEN + MUXEN + MTYP + MWID + FACCEN + BURSTEN + WAITPOL
              + WRAPMOD + WAITCFG + WREN + WAITEN + ASYNCWAIT + CBURSTRW
              + fsmc::bcr::extmod::BWTR_REGISTER_IGNORED ;
      FSMC_REGS->NorPsram[N].BTR =
              ADDSET + ADDHLD + DATAST + BUSTURN + CLKDIV + DATLAT
              + ACCMOD ;
  }

  template<Device N>
  void Functions<N>::readWord(u32* value, u32 address) {
      if (address > 0x4000000) {
          *value = 0x0;
      }
      else {
          //value = *(DataAddress[N]+address);
          return;
      }

  }

  template<Device N>
  void Functions<N>::readHalfWord(u16* value, u32 address) {

  }

  template<Device N>
  void Functions<N>::readByte(u8* value, u32 address) {

  }

  template<Device N>
  const u32* Functions<N>::dataAddress() {
      return DataAddress[N];
  }

  } // namespace norpsram
} // namespace fsmc
