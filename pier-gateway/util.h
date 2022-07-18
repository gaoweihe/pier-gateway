/*
 * util.h
 *
 * Created: 5/20/2021 1:51:13 PM
 *  Author: Gao, Weihe
 */ 

#ifndef UTIL_H_
#define UTIL_H_

bool CheckChecksum(uint8_t *pack, uint16_t size) __ATTR_PURE__;
uint8_t GetChecksum(uint8_t *pack, uint16_t size) __ATTR_PURE__;
uint16_t GetWordFromBytes(uint8_t byte1, uint8_t byte2) __ATTR_PURE__;
uint16_t GetWordFrom14Bits(uint8_t byte1, uint8_t byte2) __ATTR_PURE__;
uint8_t GetBytesFromWord(uint16_t word, uint8_t byte_num) __ATTR_PURE__; 
uint8_t GetBytesFrom14Bits(uint8_t byte1, uint8_t byte2, uint8_t byte_num) __ATTR_PURE__;
uint8_t GetBSBBySocketNumber(uint8_t socket_num) __ATTR_PURE__; 

#endif /* UTIL_H_ */