/*
 * util.c
 *
 * Created: 5/20/2021 1:51:04 PM
 *  Author: Gao, Weihe
 */ 

#include <xc.h>
#include "dependency.h"
#include "nic.h"

bool CheckChecksum(uint8_t *pack, uint16_t size)
{
	uint8_t checksumA = 0;
	uint8_t checkSumB = pack[size - 1];
	for (uint8_t i = 0; i < size - 1; i++)
	{
		checksumA += pack[i];
	}
	return ((checksumA & 0x7F) == (checkSumB & 0x7F)) ? true : false;
}

uint8_t GetChecksum(uint8_t *pack, uint16_t size) {
	uint8_t checksum = 0;
	for (uint8_t i = 0; i < size; i++)
	{
		checksum += pack[i];
	}
	return checksum;
}

uint16_t GetWordFromBytes(uint8_t byte1, uint8_t byte2) {
	union {
		uint16_t word;
		uint8_t bytes[2];
	} data;
	data.bytes[0] = byte2;
	data.bytes[1] = byte1;
	return data.word;
}

uint16_t GetWordFrom14Bits(uint8_t byte1, uint8_t byte2) {
	uint16_t word = 0;
	word = byte2 & 0b011111111;
	word += (byte1 & 0b011111111) << 7;
	return word;
}

uint8_t GetBytesFromWord(uint16_t word, uint8_t byte_num) {
	if (byte_num >= 2) return 0;
	union {
		uint8_t bytes[2];
		uint16_t word;
	} data;
	data.word = word;
	if (byte_num == 0) {
		return data.bytes[1];
	}
	else if (byte_num == 1) {
		return data.bytes[0];
	}
	return 0; // dummy
}

uint8_t GetBytesFrom14Bits(uint8_t byte1, uint8_t byte2, uint8_t byte_num) {
	uint16_t word = GetWordFrom14Bits(byte1, byte2);
	uint8_t byte = GetBytesFromWord(word, byte_num);
	return byte;
}

uint8_t GetBSBBySocketNumber(uint8_t socket_num) {
	return 0;
}