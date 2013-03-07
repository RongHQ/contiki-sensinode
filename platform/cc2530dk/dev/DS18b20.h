#ifndef _18B20_H_
#define _18B20_H_

#include "sfr-bits.h"
#include "cc253x.h"
#include "contiki-conf.h"
#include "contiki.h"

#include "dev/leds.h"

#include "dev/adc-sensor.h"



#define DQ P0_6

/**************************************************/
#ifndef uchar
#define uchar unsigned char
#define uint unsigned int

#endif
/********************函数定义**********************/
void ReadTemp(uchar *TempInt, uchar *TempDot, uchar *SignedFlag); //使用时调用这条函数就OK了
void Init_DS18B20(void);
uchar ReadOneChar(void);
void WriteOneChar(unsigned char dat);



/***************************************************
void Init_DS18B20()
功能说明： 初始化18B20
****************************************************/
void Init_DS18B20(void)
{
unsigned char x=0;

P1SEL &= ~(0x40);
P0DIR &= ~0x40;
DQ = 1; //DQ复位
clock_delay_usec(48); //稍做延时
P0DIR |= 0x40;
DQ = 0; //单片机将DQ拉低
clock_delay_usec(480); //精确延时 大于 480us
P0DIR &= ~0x40;
DQ = 1; //拉高总线
clock_delay_usec(60);
x=DQ; //稍做延时后 如果x=0则初始化成功 x=1则初始化失败
clock_delay_usec(240);
}
/***************************************************
void ReadOneChar()
功能说明： 从18B20中读取一个字节的数据
返回值： 返回具体数据
****************************************************/
unsigned char ReadOneChar(void)
{
unsigned char i=0;
unsigned char dat = 0;
for (i=8;i>0;i--)
{
P0DIR |= 0x40;
DQ = 0; // 给脉冲信号
clock_delay_usec(1);
dat>>=1;
P0DIR &= ~0x40;
DQ = 1; // 给脉冲信号
clock_delay_usec(1);
if(DQ)
dat|=0x80;
clock_delay_usec(45);
}
return(dat);
}
/***************************************************
void WriteOneChar()
功能说明： 字一个字节的数据进18B20中
****************************************************/
void WriteOneChar(unsigned char dat)
{
unsigned char i=0;
for (i=8; i>0; i--)
{
P0DIR |= 0x40;
DQ = 0;
clock_delay_usec(8);
DQ = dat&0x01;
clock_delay_usec(25);
P0DIR &= ~0x40;
DQ = 1;
dat>>=1;
}
clock_delay_usec(30);
}
/**************************************************
void ReadTemp(uchar *TempInt,uchar *TempDot, uchar *SignedFlag)
功能说明：从18B20中读出温度
出口： TempInt 温度整数部分
TempDot 温度整数部分
SignedFlag为温度正负标志，正为0，负为1
**************************************************/
void ReadTemp(uchar *TempInt,uchar *TempDot, uchar *SignedFlag)
{
unsigned char a=0;
unsigned char b=0;
unsigned int i;
unsigned long int temp;
Init_DS18B20();
WriteOneChar(0xCC); // 跳过读序号列号的操作
WriteOneChar(0x44); //12位温度分辨率，默认9位
clock_delay_usec(600);
Init_DS18B20();
WriteOneChar(0xCC); //跳过读序号列号的操作
WriteOneChar(0xBE); //读取温度寄存器等（共可读9个寄存器） 前两个就是温度
a=ReadOneChar(); //读低位
b=ReadOneChar(); //读高位
*SignedFlag=b>>7; //符号标志
if(*SignedFlag>0) //负数处理
{
temp=b;
temp=temp<<8|a;
temp=0xffff-temp; //负数时为补码，因而要取反加一
temp+=1;
b=temp>>8;
a=temp;
}
i=a&0x0f; //小数部分三位二制码
*TempInt=(a>>4)|(b<<4);
i=i*625; //小数部分转换
*TempDot=i/1000; //只取一位小数。TS:你可以修改1000的大小，以达自己想要的分辨率
}

#endif
