/******************************************************************************
*
*COPYRIGHT(c) 2022 by OMS,Inc。
*
*This file belongs to OMS,Inc。It is considered a trade secret,
*and is not to be divulged or used by parties who have not
*received written authorization from the owner。
*
*****************************************************************************/
#ifndef __RAW_TO_BMP_H__
#define __RAW_TO_BMP_H__

#include <stdint.h>
#include <string>


bool ReadRaw12File(char *file_name, unsigned short *rawBuff, int RAW_DATA_SIZE);
void rawToBmp(unsigned short *rawBuff, char *bmpFileName);
 

#endif 
