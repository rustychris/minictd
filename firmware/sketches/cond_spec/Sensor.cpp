#include "Sensor.h"

binary_format_t binary_format;

uint8_t hexmap[]={'0','1','2','3','4','5','6','7',
                  '8','9','A','B','C','D','E','F'};

void write_base16(Print &out,uint8_t *buff,int count)
{
  static uint8_t hexbuff[121];

  int i=0;
  int pos=0;

  if( binary_format==BIN_HEX) {
    for(;i<count;i++) {
      hexbuff[pos]  =hexmap[ (buff[i]>>4) & 0xF ];
      hexbuff[pos+1]=hexmap[  buff[i]     & 0xF ];
      pos+=2;
      if( pos == 120 ) {
        hexbuff[pos]='\n';
        out.write(hexbuff,121);
        pos=0;
      }
    }
    if( pos > 0 ){
      hexbuff[pos]='\n';
      out.write(hexbuff,pos+1);
    }
  } else {
    out.write(buff,count);
  }
}
