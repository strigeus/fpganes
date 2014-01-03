#include "stdafx.h"
#define	_CRT_SECURE_NO_WARNINGS
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

unsigned int swapbits(unsigned int a) {
  a &= 0xff;
  unsigned int b = 0;
  for(int i = 0; i < 8; i++, a >>= 1, b <<= 1) b |= (a & 1);
  return b>>1;
}

#define POLY 0x8408
unsigned int crc16(BYTE *data_p, unsigned short length, unsigned int crc)
{
      unsigned char i;
      unsigned int data;
      if (length == 0)
            return crc;
      do {
        for (i=0, data=(unsigned int)0xff & *data_p++; i < 8; i++, data <<= 1) {
              if ((crc & 0x0001) ^ ((data >> 7) & 0x0001))
                    crc = (crc >> 1) ^ POLY;
              else  crc >>= 1;
        }
      } while (--length);
      return crc;
}

unsigned int crc16b(BYTE *data_p, unsigned short length, unsigned int crc) {
  byte c[16], newcrc[16];
  byte d[8];
  for(int j = 0; j < 16; j++) c[j] = (crc >> (15-j)) & 1;

  for(int i = 0; i < length; i++) {
    for(int j = 0; j < 8; j++) d[j] = (data_p[i] >> j) & 1;
    
    newcrc[0] = d[4] ^ d[0] ^ c[8] ^ c[12];
    newcrc[1] = d[5] ^ d[1] ^ c[9] ^ c[13];
    newcrc[2] = d[6] ^ d[2] ^ c[10] ^ c[14];
    newcrc[3] = d[7] ^ d[3] ^ c[11] ^ c[15];
    newcrc[4] = d[4] ^ c[12];
    newcrc[5] = d[5] ^ d[4] ^ d[0] ^ c[8] ^ c[12] ^ c[13];
    newcrc[6] = d[6] ^ d[5] ^ d[1] ^ c[9] ^ c[13] ^ c[14];
    newcrc[7] = d[7] ^ d[6] ^ d[2] ^ c[10] ^ c[14] ^ c[15];
    newcrc[8] = d[7] ^ d[3] ^ c[0] ^ c[11] ^ c[15];
    newcrc[9] = d[4] ^ c[1] ^ c[12];
    newcrc[10] = d[5] ^ c[2] ^ c[13];
    newcrc[11] = d[6] ^ c[3] ^ c[14];
    newcrc[12] = d[7] ^ d[4] ^ d[0] ^ c[4] ^ c[8] ^ c[12] ^ c[15];
    newcrc[13] = d[5] ^ d[1] ^ c[5] ^ c[9] ^ c[13];
    newcrc[14] = d[6] ^ d[2] ^ c[6] ^ c[10] ^ c[14];
    newcrc[15] = d[7] ^ d[3] ^ c[7] ^ c[11] ^ c[15];

    memcpy(c, newcrc, 16);
  }

  unsigned int r = 0;
  for(int j = 0; j < 16; j++) r = r * 2 + c[j];
  return r;
}

size_t FormatPacket(byte *buf, int address, const void *data, int data_size) {
  byte *org = buf;
  while (data_size) {
    int n = data_size > 256 ? 256 : data_size;
    int cksum = address + n;
    buf[1] = address;
    buf[2] = n;
    for(int i = 0; i < n; i++) {
      int v = ((byte*)data)[i];
      buf[i+3] = v;
      cksum += v;
    }
    buf[0] = -cksum;
    buf += n + 3;
    data = (char*)data + n;
    data_size -= n;
  }
  return buf - org;
}

void WritePacket(HANDLE h, int address, const void *data, size_t data_size) {
  byte buf[(3+64) * 256];
  size_t n = FormatPacket(buf, address, data, data_size);
  DWORD written;
  if (!WriteFile(h, buf, n, &written, NULL) || written != n) {
    printf("WriteFile failed\n");
    return;
  }
}

int main(int argc, char * argv[]) {
  HANDLE h = CreateFile(L"\\\\.\\COM7", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
  if (!h) {
    printf("CreateFile failed\n");
    return 0;
  }

  DCB dcb = {0};
  dcb.DCBlength = sizeof(DCB);
  dcb.ByteSize = 8;
  dcb.StopBits = ONESTOPBIT;
  dcb.BaudRate = 2147720;
  dcb.fBinary = TRUE;
  if (!SetCommState(h, &dcb)) {
    printf("SetCommState failed\n");
    return 0;
  }

  FILE *f = fopen(argv[1], "rb");
  if (!f) { printf("File open fail\n"); return 1; }

  { char v = 1; WritePacket(h, 0x35, &v, 1); }
  { char v = 0; WritePacket(h, 0x35, &v, 1); }

  size_t total_read = 0xffffff;//10180;
  size_t pos = 0;

  while (pos < total_read) {
    char buf[16384];
    size_t want_read = (total_read - pos) > sizeof(buf) ? sizeof(buf) : (total_read - pos);
    int n = fread(buf, 1, want_read, f);
    if (n <= 0) {
      break;
    }
    WritePacket(h, 0x37, buf, n);
    pos += n;
  }

  int last_keys = -1;

  for(;;) {
	  JOYINFOEX joy;
	  joy.dwSize = sizeof(joy);
	  joy.dwFlags = JOY_RETURNALL;
	  if (joyGetPosEx(JOYSTICKID2, &joy) != MMSYSERR_NOERROR) {
      printf("Joystick error!\n");
      return 1;
    }
    unsigned char keys = 0;
		keys |= !!(joy.dwButtons & 4) * 1;
		keys |= !!(joy.dwButtons & 8) * 2;
    keys |= !!(joy.dwButtons & 256) * 4;
		keys |= !!(joy.dwButtons & 512) * 8;
		keys |= (joy.dwYpos < 0x4000) * 16;
		keys |= (joy.dwYpos >= 0xC000) * 32;
		keys |= (joy.dwXpos < 0x4000) * 64;
		keys |= (joy.dwXpos >= 0xC000) * 128;

    if (keys != last_keys) {
      printf("Keys %.2x\n", keys);
      WritePacket(h, 0x40, &keys, 1);
      last_keys = keys;
    }
    Sleep(1);
  }
  return 0;
}

