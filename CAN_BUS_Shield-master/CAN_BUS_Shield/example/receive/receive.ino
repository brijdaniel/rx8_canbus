// demo: CAN-BUS Shield, receive data
#include "mcp_can.h"
#include <SPI.h>
#include <stdio.h>
#define INT8U unsigned char

INT8U Flag_Recv = 0;
INT8U len = 0;
INT8U buf[8];
char str[20];
void setup()
{
  CAN.begin(CAN_500KBPS);                   // init can bus : baudrate = 500k
  attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt
  Serial.begin(9600);
}

void MCP2515_ISR()
{
     Flag_Recv = 1;
}

void loop()
{
    if(Flag_Recv)                   // check if get data
    {
      Flag_Recv = 0;                // clear flag
      CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
      Serial.println("CAN_BUS GET DATA!");
      Serial.print("data len = ");Serial.println(len);
      for(int i = 0; i<len; i++)    // print the data
      {
        Serial.print(buf[i]);Serial.print("\t");
      }
      Serial.println();
    }
}
