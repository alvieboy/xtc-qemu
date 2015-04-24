/*
 *  XThunderCore insn decoding macros.
 *
 *  Copyright (c) 2015 Alvaro Lopes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

/* Convenient binary macros.  */
#define HEX__(n) 0x##n##LU
#define B8__(x) ((x&0x0000000FLU)?1:0) \
                 + ((x&0x000000F0LU)?2:0) \
                 + ((x&0x00000F00LU)?4:0) \
                 + ((x&0x0000F000LU)?8:0) \
                 + ((x&0x000F0000LU)?16:0) \
                 + ((x&0x00F00000LU)?32:0) \
                 + ((x&0x0F000000LU)?64:0) \
                 + ((x&0xF0000000LU)?128:0)
#define B8(d) ((unsigned char)B8__(HEX__(d)))

#define OPCODE_MASK_H       0x7000  /* High 4 bits only.  */
#define OPCODE_MASK_EXT     0x7F00  /* High 7 bits only.  */
#define OPCODE_MASK_REGONLY 0x7FF0  /* High 11 bits only.  */

#define OPCODE_MASK_MEM     OPCODE_MASK_EXT
#define OPCODE_MASK_ARITH   OPCODE_MASK_EXT
#define OPCODE_MASK_BRI     0x7000
#define OPCODE_MASK_COP     0x7C00

#define OPCODE_MASK_IMM    0x60008000

#define IMMVAL_MASK_NON_SPECIAL 0x0000
#define IMMVAL_MASK_12 0x0FFF
#define IMMVAL_MASK_8 0x0FF0

#define OPCODE_FLAGS_NONE     0
#define OPCODE_FLAGS_NEEDEVAL 1

#define DEC_IMM      { 0x60008000, OPCODE_MASK_IMM,OPCODE_FLAGS_NONE}
#define DEC_ADD      { 0x00000000, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_ADDC     { 0x00000100, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_SUB      { 0x00000200, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_SUBB     { 0x00000300, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_AND      { 0x00000400, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_OR       { 0x00000500, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_XOR      { 0x00000600, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_CMP      { 0x00000700, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_SHL      { 0x00000800, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_SRL      { 0x00000900, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_SRA      { 0x00000A00, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_MUL      { 0x00000B00, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_ADDR     { 0x00000C00, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_NOT      { 0x00000D00, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_CADD     { 0x00000E00, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_CSUB     { 0x00000F00, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}

#define DEC_STW      { 0x00001000, OPCODE_MASK_MEM,OPCODE_FLAGS_NONE}
#define DEC_STS      { 0x00001100, OPCODE_MASK_MEM,OPCODE_FLAGS_NONE}
#define DEC_STB      { 0x00001200, OPCODE_MASK_MEM,OPCODE_FLAGS_NONE}
// Unused
#define DEC_STWP     { 0x00001400, OPCODE_MASK_MEM,OPCODE_FLAGS_NONE}
#define DEC_STSP     { 0x00001500, OPCODE_MASK_MEM,OPCODE_FLAGS_NONE}
#define DEC_STBP     { 0x00001600, OPCODE_MASK_MEM,OPCODE_FLAGS_NONE}
// Unused
#define DEC_LDW      { 0x00001800, OPCODE_MASK_MEM,OPCODE_FLAGS_NONE}
#define DEC_LDS      { 0x00001900, OPCODE_MASK_MEM,OPCODE_FLAGS_NONE}
#define DEC_LDB      { 0x00001A00, OPCODE_MASK_MEM,OPCODE_FLAGS_NONE}
// Unused
#define DEC_LDWP     { 0x00001C00, OPCODE_MASK_MEM,OPCODE_FLAGS_NONE}
#define DEC_LDSP     { 0x00001D00, OPCODE_MASK_MEM,OPCODE_FLAGS_NONE}
#define DEC_LDBP     { 0x00001E00, OPCODE_MASK_MEM,OPCODE_FLAGS_NONE}
// Unused
#define DEC_COPR     { 0x00002000, OPCODE_MASK_COP,OPCODE_FLAGS_NONE}
#define DEC_COPW     { 0x00002400, OPCODE_MASK_COP,OPCODE_FLAGS_NONE}

#define DEC_BR       { 0x00004000, OPCODE_MASK_H,OPCODE_FLAGS_NEEDEVAL}
#define DEC_ADDI     { 0x00005000, OPCODE_MASK_H,OPCODE_FLAGS_NONE}
#define DEC_CMPI     { 0x00006000, OPCODE_MASK_H,OPCODE_FLAGS_NONE}
#define DEC_LIMR     { 0x00007000, OPCODE_MASK_H,OPCODE_FLAGS_NONE}

#define DEC_JMP      { 0x00003000, OPCODE_MASK_ARITH,OPCODE_FLAGS_NEEDEVAL}
#define DEC_JMPE     { 0x00003400, OPCODE_MASK_ARITH,OPCODE_FLAGS_NEEDEVAL}
#define DEC_SEXTB    { 0x00003800, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_SEXTS    { 0x00003A00, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}

#define DEC_RSPR     { 0x00003C00, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_WSPR     { 0x00003E00, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_RUSR     { 0x00003D00, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}
#define DEC_WUSR     { 0x00003F00, OPCODE_MASK_ARITH,OPCODE_FLAGS_NONE}

// Add other.....

