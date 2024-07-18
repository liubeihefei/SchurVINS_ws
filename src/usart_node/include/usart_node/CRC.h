//
// Created by yuan on 23-7-13.
//

#ifndef CAPIMG_CRC_H
#define CAPIMG_CRC_H
#include <iostream>

extern const std::uint8_t CRC8_INIT;
extern const std::uint16_t CRC_INIT;
extern const std::uint8_t CRC8_TAB[256];
extern const std::uint16_t wCRC_Table[256];

/*
** Descriptions: CRC8 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
std::uint8_t Get_CRC8_Check_Sum(std::uint8_t *pchMessage, std::uint32_t dwLength, std::uint8_t ucCRC8);

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
std::uint32_t Verify_CRC8_Check_Sum(std::uint8_t *pchMessage, std::uint32_t dwLength);

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(std::uint8_t *pchMessage, std::uint32_t dwLength);

/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
std::uint16_t Get_CRC16_Check_Sum(std::uint8_t *pchMessage, std::uint32_t dwLength, std::uint16_t wCRC);

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
std::uint32_t Verify_CRC16_Check_Sum(std::uint8_t *pchMessage, std::uint32_t dwLength);

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(std::uint8_t *pchMessage, std::uint32_t dwLength);

#endif //CAPIMG_CRC_H
