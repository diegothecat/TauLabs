/*
 * utils.h
 *
 *  Created on: Nov 1, 2013
 *      Author: kai
 */

#ifndef UTILS_H_
#define UTILS_H_

/*
 * Unaligned variable access
 */

#define UAR16(val) UnalignedRd16((uint8_t *) &(val))
#define UAR32(val) UnalignedRd32((uint8_t *) &(val))

static inline uint8_t UnalignedRd8(const uint8_t *p)
{
   return p[0];
}

static inline uint16_t UnalignedRd16(const uint8_t *p)
{
   return p[0] | p[1] << 8;
}

static inline uint32_t UnalignedRd32(const uint8_t *p)
{
   return p[0] | p[1] << 8 | p[2] << 16 | p[3] << 24;
}

static inline uint64_t UnalignedRd64(const uint8_t *p)
{
   return ((uint64_t) UnalignedRd32(p + 4) << 32) | UnalignedRd32(p);
}

static inline uint8_t* UnalignedWr8(uint8_t val, uint8_t *p)
{
   *p++ = val;
   return p;
}

static inline uint8_t* UnalignedWr16(uint16_t val, uint8_t *p)
{
   *p++ = val;
   *p++ = val >> 8;
   return p;
}

static inline uint8_t* UnalignedWr32(uint32_t val, uint8_t *p)
{
   UnalignedWr16(val >> 16, p + 2);
   UnalignedWr16(val, p);
   return p + 4;
}

#endif /* UTILS_H_ */
