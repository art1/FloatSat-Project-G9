/*
 * The default linker script for standalone executables running on
 * Core A of ADSP-BF561 processor (dual core).
 *
 * Copyright (C) 2008 Analog Devices, Inc.
 *
 * The authors hereby grant permission to use, copy, modify, distribute,
 * and license this software and its documentation for any purpose, provided
 * that existing copyright notices are retained in all copies and that this
 * notice is included verbatim in any distributions. No written agreement,
 * license, or royalty fee is required for any of the authorized uses.
 * Modifications to this software may be copyrighted by their authors
 * and need not follow the licensing terms described here, provided that
 * the new terms are clearly indicated on the first page of each file where
 * they apply.
 */

MEMORY
{
  MEM_L1_CODE : ORIGIN = 0xFFA00000, LENGTH = 0x4000
  MEM_L1_CODE_CACHE : ORIGIN = 0xFFA10000, LENGTH = 0x4000
  MEM_L1_SCRATCH : ORIGIN = 0xFFB00000, LENGTH = 0x1000
  MEM_L1_DATA_B : ORIGIN = 0xFF900000, LENGTH = 0x4000
  MEM_L1_DATA_A : ORIGIN = 0xFF800000, LENGTH = 0x4000

  B_MEM_L1_CODE : ORIGIN = 0xFF600000, LENGTH = 0x0
  B_MEM_L1_CODE_CACHE : ORIGIN = 0xFF610000, LENGTH = 0x0
  B_MEM_L1_SCRATCH : ORIGIN = 0xFF700000, LENGTH = 0x0
  B_MEM_L1_DATA_B : ORIGIN = 0xFF500000, LENGTH = 0x0
  B_MEM_L1_DATA_A : ORIGIN = 0xFF400000, LENGTH = 0x0

  MEM_L2 : ORIGIN = 0xFEB08000, LENGTH = 0
  MEM_L2_SHARED : ORIGIN = 0xFEB10000, LENGTH = 0
  MEM_L2_CODE : ORIGIN = 0xFEB00000, LENGTH = 0x20000
  
  SDRAM : ORIGIN = 0x00000010 , LENGTH = 0x1ffffe0
  SDRAM_NO_CACHE : ORIGIN = 0x2000000, LENGTH = 0x2000000
}
