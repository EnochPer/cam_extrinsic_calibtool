/******************************************************************************
 *
 *COPYRIGHT(c) 2022 by OMS,Inc。
 *
 *This file belongs to OMS,Inc。It is considered a trade secret,
 *and is not to be divulged or used by parties who have not
 *received written authorization from the owner。
 *
 *****************************************************************************/
#include "rawToBmp.h"

int main(int argc, char* argv[]) {
  if (argc < 3) {
    printf("Usage: %s <input_raw_file> <output_bmp_file_prefix>\n", argv[0]);
    return -1;
  }
  char* bmpFileName = argv[2];
  int nSize = 240 * 181 * 2;
  unsigned short* rawBuff = new unsigned short[nSize];
  ReadRaw12File(argv[1], rawBuff, nSize);
  rawToBmp(rawBuff, bmpFileName);
  delete rawBuff;
  return 0;
}
