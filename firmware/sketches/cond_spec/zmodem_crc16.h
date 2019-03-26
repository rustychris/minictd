#ifndef ZMODEM_CRC16_H
#define ZMODEM_CRC16_H

#include <avr/pgmspace.h>


/*
 *  Crc calculation stuff
 */
/* crctab calculated by Mark G. Mendel, Network Systems Corporation */
// Dylan (monte_carlo_ecm, bitflipper, etc.) - Moved to PROGMEM
extern const unsigned short crctab[256];

/*
 * updcrc macro derived from article Copyright (C) 1986 Stephen Satchell. 
 *  NOTE: First argument must be in range 0 to 255.
 *        Second argument is referenced twice.
 * 
 * Programmers may incorporate any or all code into their programs, 
 * giving proper credit within the source. Publication of the 
 * source routines is permitted so long as proper credit is given 
 * to Stephen Satchell, Satchell Evaluations and Chuck Forsberg, 
 * Omen Technology.
 */
// Pete (El Supremo) Can't parenthesize this in a way that gets rid of a compiler
// warning on line 283 of zmodem_zm.cpp
// Dylan (monte_carlo_ecm, bitflipper, etc.) - No warning in Arduino IDE 1.6.5

#ifdef ARDUINO
#define updcrc(cp, crc) ( ( (pgm_read_word(crctab + ((crc >> 8) & 255)) ^ (crc << 8) ) ^ cp))
#else
#define updcrc(cp, crc) ( ( (crctab[((crc >> 8) & 255)] ^ (crc << 8) ) ^ cp))
#endif

/*
 * Copyright (C) 1986 Gary S. Brown.  You may use this program, or
 * code or tables extracted from it, as desired without restriction.
 */

/* First, the polynomial itself and its table of feedback terms.  The  */
/* polynomial is                                                       */
/* X^32+X^26+X^23+X^22+X^16+X^12+X^11+X^10+X^8+X^7+X^5+X^4+X^2+X^1+X^0 */
/* Note that we take it "backwards" and put the highest-order term in  */
/* the lowest-order bit.  The X^32 term is "implied"; the LSB is the   */
/* X^31 term, etc.  The X^0 term (usually shown as "+1") results in    */
/* the MSB being 1.                                                    */

/* Note that the usual hardware shift register implementation, which   */
/* is what we're using (we're merely optimizing it by doing eight-bit  */
/* chunks at a time) shifts bits into the lowest-order term.  In our   */
/* implementation, that means shifting towards the right.  Why do we   */
/* do it this way?  Because the calculated CRC must be transmitted in  */
/* order from highest-order term to lowest-order term.  UARTs transmit */
/* characters in order from LSB to MSB.  By storing the CRC this way,  */
/* we hand it to the UART in the order low-byte to high-byte; the UART */
/* sends each low-bit to hight-bit; and the result is transmission bit */
/* by bit from highest- to lowest-order term without requiring any bit */
/* shuffling on our part.  Reception works similarly.                  */

/* The feedback terms table consists of 256, 32-bit entries.  Notes:   */
/*                                                                     */
/*     The table can be generated at runtime if desired; code to do so */
/*     is shown later.  It might not be obvious, but the feedback      */
/*     terms simply represent the results of eight shift/xor opera-    */
/*     tions for all combinations of data and CRC register values.     */
/*                                                                     */
/*     The values must be right-shifted by eight bits by the "updcrc"  */
/*     logic; the shift must be unsigned (bring in zeroes).  On some   */
/*     hardware you could probably optimize the shift in assembler by  */
/*     using byte-swap instructions.                                   */

// Pete (El_Supremo) add 'unsigned'
// Dylan (monte_carlo_ecm, bitflipper, etc.) - Moved to PROGMEM
extern const unsigned long cr3tab[];

#define UPDC32(b, c) (cr3tab[((int)c ^ b) & 0xff] ^ ((c >> 8) & 0x00FFFFFF))

/* End of crctab.c */
#endif

