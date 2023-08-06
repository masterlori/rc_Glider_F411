/**
 ******************************************************************************
 * @file      crc.h
 * @author    Neat Tech
 * @brief     Header crc interface module
 *
 *            Software for crc32 calculation
 *
 ******************************************************************************/


#ifndef SRC_CRC_INTERFACE_H_
#define SRC_CRC_INTERFACE_H_

#include "../inc/main.h"
#include <inttypes.h>


extern uint32_t crc32_calc (uint16_t init, uint32_t *buf, uint16_t len);
extern uint16_t crc16_calc(uint8_t *buf, uint16_t len);
extern uint16_t crc16_RingBuf(uint8_t *buf, uint16_t tail, uint16_t len, uint16_t mask);
extern uint8_t crc8_calc(uint8_t init, uint8_t *buf, uint16_t len);

#endif /* SRC_CRC_INTERFACE_H_ */
