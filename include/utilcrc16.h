#ifndef SERIALCRC16CHECK_H

#define SERIALCRC16CHECK_H

#include <stdint.h>

using uint16_t = unsigned short;


/*

@brief：		算CRC校验

@param: 	const unsigned short init_value - CRC初始值，默认为0.

@param: 	const unsigned char * pdata - 待计算数据

@param: 	const unsigned int data_len - 待计算数据长度

@return: 	unsigned short

CRC 结果

*/
// uint16_t CRC16_Check(const uint16_t init_value, const unsigned char* pdata, unsigned int data_len);
unsigned short usMBCRC16( unsigned char * pucFrame, unsigned short usLen );
#endif  //SERIALCRC16CHECK_H


