////////////////////////////////////////////////////////////////////
//     20m HiPerformance TRX with ATMega664P and AD9951 as VFO    //
//            and AD9834 as LO, LCD: Nokia 5110                   //
////////////////////////////////////////////////////////////////////
//  Microcontroller:                                              //
//  ATMEL AVR ATmega644P 16 MHz with ext. resonator               //
//  EESAVE fuse activated, JTAG off                               //
//  Fuses for ATMega644p: E:FF, H:D1, L:FF                        //
//  -U lfuse:w:0xff:m -U hfuse:w:0xd1:m -U efuse:w:0xff:m         // 
////////////////////////////////////////////////////////////////////
//  Compiler:         GCC (GNU AVR C-Compiler)                    //
//  Author:           Peter Rachow  DK7IH                         //
//  Last Change: 2019-06-21                                       //
////////////////////////////////////////////////////////////////////
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <util/delay.h>
//////////////////////////////////////////////////
 
//Port usage  
//O U T P U T 

//DDS1
//IO_UD:       PB0 (1) (green)  
//SDIO (DATA): PB1 (2) (white)
//SCLK         PB2 (4) (blue)
//RESET        PB3     (violet)
 
//DDS2
//FSYNC:       PC0 (1) (green)  
//SDIO (DATA): PC1 (2) (white)
//SCLK         PC2 (4) (blue)
//RESET        PC3  (pink)

//LCD
//RES:  PD4
//DC:   PD5
//DIN:  PD6
//SCLK: PD7

// I N P U T
//PD0: TX/RX detect
//PD1 sideband switch
//PD2, PD3: Rotary encoder

//ADC
//ADC Channels
//PA0 - ADC0: 4 user keys in front panel
//PA1 - ADC1: voltage measurement voltage divider
//PA2 - ADC2: S-meter from AGC voltage
//PA3 - ADC3: TX PWR meter
//PA4 - ADC4: Temp. sensor PA

//  EEPROM  //////////////////////////////////////////////////
// Bytes
// 0:63: Memory frequencies of MEM 0:15 in blocks by 4 bytes
// 64:67: Freq VFO A memplace=16
// 68:71: Freq VFO B memplace=17
// 127: Last memory selected
// 128: Last VFO in use
// 129: Scan threshold (0:80)
// 132:135: scanfreq[0] on memplace=33
// 136:139: scanfreq[1] on memplace=34
// 140:143: f_lo[0] on memplace=35
// 144:147: f_lo[1] on memplace=36
//////////////////////////////////////////////////////////////

#define VOLTAGEFACTOR 4.6 //Const for voltage calculation

#undef F_CPU
#define F_CPU 16000000
#define INTERFREQUENCY 9000000

// SPI DDS1 (AD9951)
#define DDS1_PORT PORTB
#define DDS1_IO_UD 1      //PB0
#define DDS1_SDIO 2       //PB1
#define DDS1_SCLK 4       //PB2
#define DDS1_RESETPIN 3   //PB3

//  SPI DDS2 (AD9834)
#define DDS2_PORT PORTC 
#define DDS_FSYNC 1
#define DDS_SDATA 2
#define DDS_SCLK 4
#define DDS2_RESETPIN 3
 
// PORT definitions Nokia 5110 LCD
#define LCD_PORT PORTD
#define RES 16       //PD4
#define DC 32        //PD5
#define DN 64        //PD6
#define LCDSCLK 128  //PD7

//LCD
#define FONTWIDTH 6

////////////////////////
// F U N C T I O N S  //
////////////////////////
int main(void);

//Menu
long menux(long, int);
void print_menu_head(char*, char*, int);
void print_menu_item(char*, int, int);
void print_menu_head(char*, char*, int);
int navigate_thru_item_list(int, int);

//Scanning & VFO
long scan(int);
void set_scan_threshold(void);
long set_scan_frequency(int, long);
int calc_tuningfactor(void);
int set_vfo(int, int);

//SPI for DDS1
void spi1_send_bit1(int);
void spi1_send_byte1(unsigned int);
void spi1_send_word1(unsigned int);
void set_frequency1(long);

//SPI for DDS2
void spi2_start(void);
void spi2_send_bit(int);
void spi2_stop(void);
void set_frequency2(unsigned long);

//LO setting
void set_lo_freq(int);

//STRING FUNCTIONS
int int2asc(long, int, char*, int);
int strlen(char *);

//SPI for LCD
void lcd_sendbyte(char, int);
void lcd_senddata(char);
void lcd_sendcmd(char);
void lcd_reset(void);
void lcd_gotoxy(char, char);
void lcd_cleanram(void);
int xp2(int);
void lcd_putchar2(int, int, char, int);
void lcd_putchar1(int, int, char, int);
void lcd_putstring(int, int, char*, char, int);
void lcd_putnumber(int, int, long, int, int, int);
void lcd_init(void);
void lcd_clearsection(int, int, int);
void lcd_cls(int, int, int, int);
void lcd_drawbox(int, int, int, int);

//DATA DISPLAY ROUTINES
void show_frequency(unsigned long);
void show_frequency2(unsigned long);
void show_voltage(int);
void show_meter(int);
void reset_smax(void);
void show_sideband(int, int);
void show_mem_addr(int, int);
void show_mem_freq(unsigned long, int);
void show_pa_temp(int);
void show_vfo(int, int);
void show_all_data(unsigned long, int, int, int, int, int);
void show_meter_scale(int);

//ADC
//ADC Channels
//ADC0: keys
//ADC1: voltage
int get_adc(int);
int get_keys(void);
int get_temp(void);

//EEPROM and frequency storage
#define MAXMEM 15
void store_frequency(long, int);
unsigned long load_frequency(int);
int is_mem_freq_ok(unsigned long);
void store_last_mem(int);
int load_last_mem(void);
void store_last_vfo(int);
int load_last_vfo(void);
void store_vfo_data(int, unsigned long, unsigned long);

long recall_mem_freq(void);
int save_mem_freq(long, int);

//////////////////////////
//   V A R I A B L E S
//////////////////////////
//Rotray encoder
int tuningknob = 0;
int tuningcount = 0;

//Timer
unsigned long runseconds10 = 0;

//Tuning
unsigned long f_vfo[2];
int vfo_x, vfo_y;

//LO settings
long f_lo[] = {9000600, 8998200}; //USB, LSB
int sideband = 0; //Sets sideband to USB	

//MEMORY
int last_memplace = 0;
int last_mem = 0; //Stored in Byte ( 4 * 16 + 1) = 65

//Scanning
int s_threshold = 30;
long scanfreq[2];

//S-Meter max value
int smax = 0;
long runseconds10s = 0;

// Font 6x8 for LCD Display Nokia 5110
static const __flash char xchar[] = {
0x00,0x00,0x00,0x00,0x00,0x00,	// 0x00
0x00,0x3E,0x45,0x51,0x45,0x3E,	// 0x01
0x00,0x3E,0x6B,0x6F,0x6B,0x3E,	// 0x02
0x00,0x1C,0x3E,0x7C,0x3E,0x1C,	// 0x03
0x00,0x18,0x3C,0x7E,0x3C,0x18,	// 0x04
0x00,0x30,0x36,0x7F,0x36,0x30,	// 0x05
0x00,0x18,0x5C,0x7E,0x5C,0x18,	// 0x06
0x00,0x00,0x00,0x00,0x00,0x00,	// 0x07
0x00,0x00,0x00,0x00,0x00,0x00,	// 0x08
0x00,0x00,0x00,0x00,0x00,0x00,	// 0x09
0x00,0x00,0x00,0x00,0x00,0x00,	// 0x0A
0x00,0x30,0x48,0x4A,0x36,0x0E,	// 0x0B
0x00,0x06,0x29,0x79,0x29,0x06,	// 0x0C
0x00,0x00,0x00,0x00,0x00,0x00,	// 0x0D
0x00,0x60,0x7E,0x0A,0x35,0x3F,	// 0x0E
0x00,0x2A,0x1C,0x36,0x1C,0x2A,	// 0x0F
0x00,0x00,0x7F,0x3E,0x1C,0x08,	// 0x10
0x00,0x08,0x1C,0x3E,0x7F,0x00,	// 0x11
0x00,0x14,0x36,0x7F,0x36,0x14,	// 0x12
0x00,0x00,0x5F,0x00,0x5F,0x00,	// 0x13
0x00,0x06,0x09,0x7F,0x01,0x7F,	// 0x14
0x00,0x22,0x4D,0x55,0x59,0x22,	// 0x15
0x00,0x60,0x60,0x60,0x60,0x00,	// 0x16
0x00,0x14,0xB6,0xFF,0xB6,0x14,	// 0x17
0x00,0x04,0x06,0x7F,0x06,0x04,	// 0x18
0x00,0x10,0x30,0x7F,0x30,0x10,	// 0x19
0x00,0x08,0x08,0x3E,0x1C,0x08,	// 0x1A
0x00,0x08,0x1C,0x3E,0x08,0x08,	// 0x1B
0x00,0x78,0x40,0x40,0x40,0x40,	// 0x1C
0x00,0x08,0x3E,0x08,0x3E,0x08,	// 0x1D
0x00,0x30,0x3C,0x3F,0x3C,0x30,	// 0x1E
0x00,0x03,0x0F,0x3F,0x0F,0x03,	// 0x1F
0x00,0x00,0x00,0x00,0x00,0x00,	// 0x20
0x00,0x00,0x06,0x5F,0x06,0x00,	// 0x21
0x00,0x07,0x03,0x00,0x07,0x03,	// 0x22
0x00,0x24,0x7E,0x24,0x7E,0x24,	// 0x23
0x00,0x24,0x2B,0x6A,0x12,0x00,	// 0x24
0x00,0x63,0x13,0x08,0x64,0x63,	// 0x25
0x00,0x36,0x49,0x56,0x20,0x50,	// 0x26
0x00,0x00,0x07,0x03,0x00,0x00,	// 0x27
0x00,0x00,0x3E,0x41,0x00,0x00,	// 0x28
0x00,0x00,0x41,0x3E,0x00,0x00,	// 0x29
0x00,0x08,0x3E,0x1C,0x3E,0x08,	// 0x2A
0x00,0x08,0x08,0x3E,0x08,0x08,	// 0x2B
0x00,0x00,0xE0,0x60,0x00,0x00,	// 0x2C
0x00,0x08,0x08,0x08,0x08,0x08,	// 0x2D
0x00,0x00,0x60,0x60,0x00,0x00,	// 0x2E
0x00,0x20,0x10,0x08,0x04,0x02,	// 0x2F
0x00,0x3E,0x41,0x49,0x41,0x3E,	// 0x30
0x00,0x00,0x42,0x7F,0x40,0x00,	// 0x31
0x00,0x62,0x51,0x49,0x49,0x46,	// 0x32
0x00,0x22,0x49,0x49,0x49,0x36,	// 0x33
0x00,0x18,0x14,0x12,0x7F,0x10,	// 0x34
0x00,0x2F,0x49,0x49,0x49,0x31,	// 0x35
0x00,0x3C,0x4A,0x49,0x49,0x30,	// 0x36
0x00,0x01,0x71,0x09,0x05,0x03,	// 0x37
0x00,0x36,0x49,0x49,0x49,0x36,	// 0x38
0x00,0x06,0x49,0x49,0x29,0x1E,	// 0x39
0x00,0x00,0x6C,0x6C,0x00,0x00,	// 0x3A
0x00,0x00,0xEC,0x6C,0x00,0x00,	// 0x3B
0x00,0x08,0x14,0x22,0x41,0x00,	// 0x3C
0x00,0x24,0x24,0x24,0x24,0x24,	// 0x3D
0x00,0x00,0x41,0x22,0x14,0x08,	// 0x3E
0x00,0x02,0x01,0x59,0x09,0x06,	// 0x3F
0x00,0x3E,0x41,0x5D,0x55,0x1E,	// 0x40
0x00,0x7E,0x11,0x11,0x11,0x7E,	// 0x41
0x00,0x7F,0x49,0x49,0x49,0x36,	// 0x42
0x00,0x3E,0x41,0x41,0x41,0x22,	// 0x43
0x00,0x7F,0x41,0x41,0x41,0x3E,	// 0x44
0x00,0x7F,0x49,0x49,0x49,0x41,	// 0x45
0x00,0x7F,0x09,0x09,0x09,0x01,	// 0x46
0x00,0x3E,0x41,0x49,0x49,0x7A,	// 0x47
0x00,0x7F,0x08,0x08,0x08,0x7F,	// 0x48
0x00,0x00,0x41,0x7F,0x41,0x00,	// 0x49
0x00,0x30,0x40,0x40,0x40,0x3F,	// 0x4A
0x00,0x7F,0x08,0x14,0x22,0x41,	// 0x4B
0x00,0x7F,0x40,0x40,0x40,0x40,	// 0x4C
0x00,0x7F,0x02,0x04,0x02,0x7F,	// 0x4D
0x00,0x7F,0x02,0x04,0x08,0x7F,	// 0x4E
0x00,0x3E,0x41,0x41,0x41,0x3E,	// 0x4F
0x00,0x7F,0x09,0x09,0x09,0x06,	// 0x50
0x00,0x3E,0x41,0x51,0x21,0x5E,	// 0x51
0x00,0x7F,0x09,0x09,0x19,0x66,	// 0x52
0x00,0x26,0x49,0x49,0x49,0x32,	// 0x53
0x00,0x01,0x01,0x7F,0x01,0x01,	// 0x54
0x00,0x3F,0x40,0x40,0x40,0x3F,	// 0x55
0x00,0x1F,0x20,0x40,0x20,0x1F,	// 0x56
0x00,0x3F,0x40,0x3C,0x40,0x3F,	// 0x57
0x00,0x63,0x14,0x08,0x14,0x63,	// 0x58
0x00,0x07,0x08,0x70,0x08,0x07,	// 0x59
0x00,0x71,0x49,0x45,0x43,0x00,	// 0x5A
0x00,0x00,0x7F,0x41,0x41,0x00,	// 0x5B
0x00,0x02,0x04,0x08,0x10,0x20,	// 0x5C
0x00,0x00,0x41,0x41,0x7F,0x00,	// 0x5D
0x00,0x04,0x02,0x01,0x02,0x04,	// 0x5E
0x80,0x80,0x80,0x80,0x80,0x80,	// 0x5F
0x00,0x00,0x03,0x07,0x00,0x00,	// 0x60
0x00,0x20,0x54,0x54,0x54,0x78,	// 0x61
0x00,0x7F,0x44,0x44,0x44,0x38,	// 0x62
0x00,0x38,0x44,0x44,0x44,0x28,	// 0x63
0x00,0x38,0x44,0x44,0x44,0x7F,	// 0x64
0x00,0x38,0x54,0x54,0x54,0x08,	// 0x65
0x00,0x08,0x7E,0x09,0x09,0x00,	// 0x66
0x00,0x18,0xA4,0xA4,0xA4,0x7C,	// 0x67
0x00,0x7F,0x04,0x04,0x78,0x00,	// 0x68
0x00,0x00,0x00,0x7D,0x40,0x00,	// 0x69
0x00,0x40,0x80,0x84,0x7D,0x00,	// 0x6A
0x00,0x7F,0x10,0x28,0x44,0x00,	// 0x6B
0x00,0x00,0x00,0x7F,0x40,0x00,	// 0x6C
0x00,0x7C,0x04,0x18,0x04,0x78,	// 0x6D
0x00,0x7C,0x04,0x04,0x78,0x00,	// 0x6E
0x00,0x38,0x44,0x44,0x44,0x38,	// 0x6F
0x00,0xFC,0x44,0x44,0x44,0x38,	// 0x70
0x00,0x38,0x44,0x44,0x44,0xFC,	// 0x71
0x00,0x44,0x78,0x44,0x04,0x08,	// 0x72
0x00,0x08,0x54,0x54,0x54,0x20,	// 0x73
0x00,0x04,0x3E,0x44,0x24,0x00,	// 0x74
0x00,0x3C,0x40,0x20,0x7C,0x00,	// 0x75
0x00,0x1C,0x20,0x40,0x20,0x1C,	// 0x76
0x00,0x3C,0x60,0x30,0x60,0x3C,	// 0x77
0x00,0x6C,0x10,0x10,0x6C,0x00,	// 0x78
0x00,0x9C,0xA0,0x60,0x3C,0x00,	// 0x79
0x00,0x64,0x54,0x54,0x4C,0x00,	// 0x7A
0x00,0x08,0x3E,0x41,0x41,0x00,	// 0x7B
0x00,0x00,0x00,0x77,0x00,0x00,	// 0x7C
0x00,0x00,0x41,0x41,0x3E,0x08,	// 0x7D
0x00,0x02,0x01,0x02,0x01,0x00,	// 0x7E
0x00,0x3C,0x26,0x23,0x26,0x3C,	// 0x7F
0x00,0x1E,0xA1,0xE1,0x21,0x12,	// 0x80
0x00,0x3D,0x40,0x20,0x7D,0x00,	// 0x81
0x00,0x38,0x54,0x54,0x55,0x09,	// 0x82
0x00,0x20,0x55,0x55,0x55,0x78,	// 0x83
0x00,0x20,0x55,0x54,0x55,0x78,	// 0x84
0x00,0x20,0x55,0x55,0x54,0x78,	// 0x85
0x00,0x20,0x57,0x55,0x57,0x78,	// 0x86
0x00,0x1C,0xA2,0xE2,0x22,0x14,	// 0x87
0x00,0x38,0x55,0x55,0x55,0x08,	// 0x88
0x00,0x38,0x55,0x54,0x55,0x08,	// 0x89
0x00,0x38,0x55,0x55,0x54,0x08,	// 0x8A
0x00,0x00,0x01,0x7C,0x41,0x00,	// 0x8B
0x00,0x00,0x01,0x7D,0x41,0x00,	// 0x8C
0x00,0x00,0x01,0x7C,0x40,0x00,	// 0x8D
0x00,0x70,0x29,0x24,0x29,0x70,	// 0x8E
0x00,0x78,0x2F,0x25,0x2F,0x78,	// 0x8F
0x00,0x7C,0x54,0x54,0x55,0x45,	// 0x90
0x00,0x34,0x54,0x7C,0x54,0x58,	// 0x91
0x00,0x7E,0x09,0x7F,0x49,0x49,	// 0x92
0x00,0x38,0x45,0x45,0x39,0x00,	// 0x93
0x00,0x38,0x45,0x44,0x39,0x00,	// 0x94
0x00,0x39,0x45,0x44,0x38,0x00,	// 0x95
0x00,0x3C,0x41,0x21,0x7D,0x00,	// 0x96
0x00,0x3D,0x41,0x20,0x7C,0x00,	// 0x97
0x00,0x9C,0xA1,0x60,0x3D,0x00,	// 0x98
0x00,0x3D,0x42,0x42,0x3D,0x00,	// 0x99
0x00,0x3C,0x41,0x40,0x3D,0x00,	// 0x9A
0x80,0x70,0x68,0x58,0x38,0x04,	// 0x9B
0x00,0x48,0x3E,0x49,0x49,0x62,	// 0x9C
0x00,0x7E,0x61,0x5D,0x43,0x3F,	// 0x9D
0x00,0x22,0x14,0x08,0x14,0x22,	// 0x9E
0x00,0x40,0x88,0x7E,0x09,0x02,	// 0x9F
0x00,0x20,0x54,0x55,0x55,0x78,	// 0xA0
0x00,0x00,0x00,0x7D,0x41,0x00,	// 0xA1
0x00,0x38,0x44,0x45,0x39,0x00,	// 0xA2
0x00,0x3C,0x40,0x21,0x7D,0x00,	// 0xA3
0x00,0x7A,0x09,0x0A,0x71,0x00,	// 0xA4
0x00,0x7A,0x11,0x22,0x79,0x00,	// 0xA5
0x00,0x08,0x55,0x55,0x55,0x5E,	// 0xA6
0x00,0x4E,0x51,0x51,0x4E,0x00,	// 0xA7
0x00,0x30,0x48,0x4D,0x40,0x20,	// 0xA8
0x3E,0x41,0x5D,0x4B,0x55,0x3E,	// 0xA9
0x04,0x04,0x04,0x04,0x04,0x1C,	// 0xAA
0x00,0x17,0x08,0x4C,0x6A,0x50,	// 0xAB
0x00,0x17,0x08,0x34,0x2A,0x78,	// 0xAC
0x00,0x00,0x30,0x7D,0x30,0x00,	// 0xAD
0x00,0x08,0x14,0x00,0x08,0x14,	// 0xAE
0x00,0x14,0x08,0x00,0x14,0x08,	// 0xAF
0x44,0x11,0x44,0x11,0x44,0x11,	// 0xB0
0xAA,0x55,0xAA,0x55,0xAA,0x55,	// 0xB1
0xBB,0xEE,0xBB,0xEE,0xBB,0xEE,	// 0xB2
0x00,0x00,0x00,0xFF,0x00,0x00,	// 0xB3
0x08,0x08,0x08,0xFF,0x00,0x00,	// 0xB4
0x00,0x70,0x28,0x25,0x29,0x70,	// 0xB5
0x00,0x70,0x29,0x25,0x29,0x70,	// 0xB6
0x00,0x70,0x29,0x25,0x28,0x70,	// 0xB7
0x3E,0x41,0x5D,0x55,0x41,0x3E,	// 0xB8
0x0A,0xFB,0x00,0xFF,0x00,0x00,	// 0xB9
0x00,0xFF,0x00,0xFF,0x00,0x00,	// 0xBA
0x0A,0xFA,0x02,0xFE,0x00,0x00,	// 0xBB
0x0A,0x0B,0x08,0x0F,0x00,0x00,	// 0xBC
0x00,0x18,0x24,0x66,0x24,0x00,	// 0xBD
0x00,0x29,0x2A,0x7C,0x2A,0x29,	// 0xBE
0x08,0x08,0x08,0xF8,0x00,0x00,	// 0xBF
0x00,0x00,0x00,0x0F,0x08,0x08,	// 0xC0
0x08,0x08,0x08,0x0F,0x08,0x08,	// 0xC1
0x08,0x08,0x08,0xF8,0x08,0x08,	// 0xC2
0x00,0x00,0x00,0xFF,0x08,0x08,	// 0xC3
0x08,0x08,0x08,0x08,0x08,0x08,	// 0xC4
0x08,0x08,0x08,0xFF,0x08,0x08,	// 0xC5
0x00,0x20,0x56,0x55,0x56,0x79,	// 0xC6
0x00,0x70,0x2A,0x25,0x2A,0x71,	// 0xC7
0x00,0x0F,0x08,0x0B,0x0A,0x0A,	// 0xC8
0x00,0xFE,0x02,0xFA,0x0A,0x0A,	// 0xC9
0x0A,0x0B,0x08,0x0B,0x0A,0x0A,	// 0xCA
0x0A,0xFA,0x02,0xFA,0x0A,0x0A,	// 0xCB
0x00,0xFF,0x00,0xFB,0x0A,0x0A,	// 0xCC
0x0A,0x0A,0x0A,0x0A,0x0A,0x0A,	// 0xCD
0x0A,0xFB,0x00,0xFB,0x0A,0x0A,	// 0xCE
0x00,0x5D,0x22,0x22,0x22,0x5D,	// 0xCF
0x00,0x22,0x55,0x59,0x30,0x00,	// 0xD0
0x00,0x08,0x7F,0x49,0x41,0x3E,	// 0xD1
0x00,0x7C,0x55,0x55,0x55,0x44,	// 0xD2
0x00,0x7C,0x55,0x54,0x55,0x44,	// 0xD3
0x00,0x7C,0x55,0x55,0x54,0x44,	// 0xD4
0x00,0x00,0x00,0x07,0x00,0x00,	// 0xD5
0x00,0x00,0x44,0x7D,0x45,0x00,	// 0xD6
0x00,0x00,0x45,0x7D,0x45,0x00,	// 0xD7
0x00,0x00,0x45,0x7C,0x45,0x00,	// 0xD8
0x08,0x08,0x08,0x0F,0x00,0x00,	// 0xD9
0x00,0x00,0x00,0xF8,0x08,0x08,	// 0xDA
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,	// 0xDB
0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,	// 0xDC
0x00,0x00,0x00,0x77,0x00,0x00,	// 0xDD
0x00,0x00,0x45,0x7D,0x44,0x00,	// 0xDE
0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,	// 0xDF
0x00,0x3C,0x42,0x43,0x3D,0x00,	// 0xE0
0x00,0xFE,0x4A,0x4A,0x34,0x00,	// 0xE1
0x00,0x3C,0x43,0x43,0x3D,0x00,	// 0xE2
0x00,0x3D,0x43,0x42,0x3C,0x00,	// 0xE3
0x00,0x32,0x49,0x4A,0x31,0x00,	// 0xE4
0x00,0x3A,0x45,0x46,0x39,0x00,	// 0xE5
0x00,0xFC,0x20,0x20,0x1C,0x00,	// 0xE6
0x00,0xFE,0xAA,0x28,0x10,0x00,	// 0xE7
0x00,0xFF,0xA5,0x24,0x18,0x00,	// 0xE8
0x00,0x3C,0x40,0x41,0x3D,0x00,	// 0xE9
0x00,0x3C,0x41,0x41,0x3D,0x00,	// 0xEA
0x00,0x3D,0x41,0x40,0x3C,0x00,	// 0xEB
0x00,0x9C,0xA0,0x61,0x3D,0x00,	// 0xEC
0x00,0x04,0x08,0x71,0x09,0x04,	// 0xED
0x00,0x00,0x02,0x02,0x02,0x00,	// 0xEE
0x00,0x00,0x07,0x03,0x00,0x00,	// 0xEF
0x00,0x00,0x08,0x08,0x08,0x00,	// 0xF0
0x00,0x00,0x24,0x2E,0x24,0x00,	// 0xF1
0x00,0x24,0x24,0x24,0x24,0x24,	// 0xF2
0x05,0x17,0x0A,0x34,0x2A,0x78,	// 0xF3
0x00,0x06,0x09,0x7F,0x01,0x7F,	// 0xF4
0x00,0x22,0x4D,0x55,0x59,0x22,	// 0xF5
0x00,0x08,0x08,0x2A,0x08,0x08,	// 0xF6
0x00,0x00,0x08,0x18,0x18,0x00,	// 0xF7
0x00,0x06,0x09,0x09,0x06,0x00,	// 0xF8
0x00,0x00,0x08,0x00,0x08,0x00,	// 0xF9
0x00,0x00,0x08,0x00,0x00,0x00,	// 0xFA
0x00,0x02,0x0F,0x00,0x00,0x00,	// 0xFB
0x00,0x09,0x0F,0x05,0x00,0x00,	// 0xFC
0x00,0x09,0x0D,0x0A,0x00,0x00,	// 0xFD
0x00,0x3C,0x3C,0x3C,0x3C,0x00,	// 0xFE
0x00,0x00,0x00,0x00,0x00,0x00 	// 0xFF
};

  ////////////////////////
 //    Misc functions  //
////////////////////////
int calc_tuningfactor(void)
{
	return (tuningcount * tuningcount) << 1; //2
}	

  ////////////////////////
 //    SPI for DDS 1   //
////////////////////////
void spi1_send_bit(int sbit)
{
    DDS1_PORT &= ~(DDS1_SCLK);  //SCLK lo
    	
    if(sbit)
	{
		DDS1_PORT|= DDS1_SDIO;  //SDATA  set
	}
	else
	{
		DDS1_PORT &= ~(DDS1_SDIO);  //SDATA  erase
	}
	
    DDS1_PORT |= DDS1_SCLK; //SCLK hi
	
}

void spi1_send_byte(unsigned int sbyte)
{
    int t1, x = (1 << 7);
	
	for(t1 = 0; t1 < 8; t1++)
	{
	    spi1_send_bit(sbyte & x);	
		x >>= 1;
	}	
}

void spi1_send_word(unsigned int sword)
{
    int t1, x = (1 << 15);
	
	for(t1 = 0; t1 < 16; t1++)
	{
	    spi1_send_bit(sword & x);	
		x >>= 1;
	}	
}

//SET frequency AD9951 DDS
//f.clock = 400MHz
void set_frequency1(long frequency)
{
    unsigned long f;
    unsigned long fword;
    int t1, t2, shiftbyte = 24, resultbyte, x;
    unsigned long comparebyte = 0xFF000000;
	
	f = frequency;
		 
	if(!sideband)//Calculate correct offset from center frequency in display for each sideband
	{
	     fword = (unsigned long) (f + INTERFREQUENCY + 1300) * 10.73741824; //USB//21.47483648 200MHz
	}    
	else
    {
	     fword = (unsigned long) (f + INTERFREQUENCY - 1300) * 10.73741824; //USB
	}    
	
    //Start transfer to DDS
    DDS1_PORT &= ~(DDS1_IO_UD); //DDS1_IO_UD lo
    
	//Send instruction bit to set fequency by frequency tuning word
	x = (1 << 7);
	for(t1 = 0; t1 < 8; t1++)
	{
	    spi1_send_bit(0x04 & x);	
		x >>= 1;
	}	
	
    //Calculate and transfer the 4 bytes of the tuning word to DDS
    //Start with msb
    for(t1 = 0; t1 < 4; t1++)
    {
        resultbyte = (fword & comparebyte) >> shiftbyte;
        comparebyte >>= 8;
        shiftbyte -= 8;       
        x = (1 << 7);
        for(t2 = 0; t2 < 8; t2++)
	    {
	        spi1_send_bit(resultbyte & x);	
		    x >>= 1;
	    }	
    }    
	
	//End transfer sequence
    DDS1_PORT|= (DDS1_IO_UD); //DDS1_IO_UD hi 
}

  /////////////////
 //  SPI DDS2   //
/////////////////
void spi2_start(void)
{
	DDS2_PORT |= DDS_SCLK;      //SCLK hi
    DDS2_PORT &= ~(DDS_FSYNC);  //FSYNC lo
}

void spi2_stop(void)
{
	DDS2_PORT |= DDS_FSYNC; //FSYNC hi
}

void spi2_send_bit(int sbit)
{
    if(sbit)
	{
		DDS2_PORT |= DDS_SDATA;  //SDATA hi
	}
	else
	{
		DDS2_PORT &= ~(DDS_SDATA);  //SDATA lo
	}
	
	DDS2_PORT |= DDS_SCLK;     //SCLK hi
    
    DDS2_PORT &= ~(DDS_SCLK);  //SCLK lo
}

void set_frequency2(unsigned long f)
{

    double fword0;
    long fword1, x;
    int l[] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int m[] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, t1;
    
    fword0 = (double) 3.579139413 * f; // 3.579139413 = 268435456 / 75000000
    fword1 = (long) fword0;

    //Transfer frequency word to byte array
    x = (1 << 13);      //2^13
    for(t1 = 2; t1 < 16; t1++)
    {
		if(fword1 & x)
	    {
			l[t1] = 1;
	    }
	    x >>= 1;
    }
    
    x = (1L << 27);  //2^27
    for(t1 = 2; t1 < 16; t1++)
    {
	    if(fword1 & x)
	    {
	        m[t1] = 1;
	    }
	    x >>= 1;
    }
    ////////////////////////////////////////
    
    //Transfer to DDS
    //Send start command
    spi2_start();
    for(t1 = 15; t1 >= 0; t1--)
    {
       spi2_send_bit(0x2000 & (1 << t1));
    }
    spi2_stop();
        
    //Transfer frequency word	
    //L-WORD
    spi2_start();
    for(t1 = 0; t1 < 16; t1++)
    {
       spi2_send_bit(l[t1]);
    }
    spi2_stop();
	
	//M-WORD
	spi2_start();
    for(t1 = 0; t1 < 16; t1++)
    {
       spi2_send_bit(m[t1]);
    }
    spi2_stop();
}

void set_lo_freq(int sb)
{
			
	int key;
	long f = f_lo[sb];
	
	lcd_cls(0, 83, 0, 47);
		
	lcd_putstring(18, 0, " LO FREQ ", 0, 1);
	if(!sb)
	{
		lcd_putstring(18, 2, "USB", 0, 0);
	}
	else	
	{
		lcd_putstring(18, 2, "LSB", 0, 0);
	}
		
	key = get_keys();
	show_frequency2(f);
	
	while(key == 0)
	{
		if(tuningknob <= -1) //Turn CW
		{
			f += 10;
		    tuningknob = 0;
	        show_frequency2(f);
	        set_frequency2(f);
		}

		if(tuningknob >= 1)  //Turn CCW
		{    
		    f -= 10;
		    tuningknob = 0;
		    show_frequency2(f);
		    set_frequency2(f);
		}		
		key = get_keys();
	}
	
	if(key == 2)
	{
		f_lo[sb] = f; //Confirm
		store_frequency(f, 35 + sb);
	}	
	else
	{
		set_frequency2(f_lo[sb]); //Abort and restore old data
	}	
}	

  //////////////////////
 //    SPI for LCD   //
//////////////////////
//Send the information to LCD (command or data)
void lcd_sendbyte(char x, int command)
{ 
    int t1, bx = 128;
	
	if(command)
	{   //CMD
	    LCD_PORT &= ~(DC);
	}
	else 
    {  //DATA
	    LCD_PORT |= DC;
	} 
	
    for(t1 = 0; t1 < 8; t1++)
    { 
	    LCD_PORT &= ~(LCDSCLK);
			    
        if(x & bx)
	    {
            LCD_PORT |= DN; 
		}
	    else
        {
	        LCD_PORT &= ~(DN);
	    }	
        
		LCD_PORT |= LCDSCLK;
    	
		bx >>= 1;
    }
}

//Send display data to Nokia LCD 5110
void lcd_senddata(char x)
{
    lcd_sendbyte(x, 0);
}

//Send  command to Nokia LCD 5110
void lcd_sendcmd(char x)
{
    lcd_sendbyte(x, 1);
}

//Reset LCD when program starts
void lcd_reset(void)
{
    LCD_PORT &= ~(RES);
	_delay_us(100);
    LCD_PORT |= RES;
}	

//Init NOKIA 5110 LCD
void lcd_init(void)
{

    _delay_ms(20);
	lcd_reset();
		
	//Configure the display
    lcd_sendcmd(0x21);    //Extended commands follow
    lcd_sendcmd(0xBF);    //Set Vop (Contrast)
    lcd_sendcmd(0x04);    //Set Temp. coefficient
    lcd_sendcmd(0x14);    //LCD bias mode
    lcd_sendcmd(0x20);    //Normal instruction set
    lcd_sendcmd(0x0C);    //Set display to normal mode.
	
	lcd_cleanram();
    _delay_ms(100);
}	

//Clear part of LCD
void lcd_clearsection(int x0, int x1, int y0)
{
    int t1;
	
	for(t1 = x0; t1 < x1; t1++)
	{
	    lcd_gotoxy(t1, y0);
		lcd_senddata(0x00);
	}	

}

//Clear whole LCD
void lcd_cls(int x0, int x1, int y0, int y1)
{
    int x, y;
	
	for(y = y0; y < y1; y++)
	{
	    for(x = x0; x < x1; x++)
		{
            lcd_gotoxy(x, y);
		    lcd_senddata(0x00);
		}
	}	
}

//Set pos of cursor to start next text display on LCD
void lcd_gotoxy(char x, char y)
{ 
    lcd_sendcmd(0x40 | (y & 0x07));
    lcd_sendcmd(0x80 | (x & 0x7F));
}

//Init RAM of LCD
void lcd_cleanram(void)
{
    int i;
    
	lcd_gotoxy(0,0);
	
    _delay_ms(10);
    
	for(i = 0; i < 768; i++)
    {
        lcd_senddata(0x00);
	}	
    
	_delay_ms(1);
}

//Power of 2
int xp2(int xp)
{
    int t1, r = 1;
    for(t1 = 0; t1 < xp; t1++)
    {
	    r <<= 1;
    }
    return r;

}

//Print character in normal size
void lcd_putchar1(int col, int row, char ch1, int inv)
{ 
    int p, t1;
    char ch2;
	
	lcd_gotoxy(col, row);
    
    p = (FONTWIDTH * ch1); // - 32 * FONTWIDTH;
    for(t1 = FONTWIDTH; t1 > 0; t1--)
    { 
	    if(!inv)
		{
	        ch2 = xchar[p];
		}
		else
		{
	        ch2 = ~xchar[p];
		}
		
        lcd_senddata(ch2);
        p++;
    }
	
	if(!inv)
	{
	    lcd_senddata(0x00);
	}
	else
	{
	    lcd_senddata(0xFF);
	}
}

//Print character in double size
void lcd_putchar2(int col, int row, char ch1, int inv)
{ 
    int p, t1, t2, x;
	int b, b1, b2; 
    char colval;
	   
    p = (FONTWIDTH * ch1);// - FONTWIDTH * 32;
    	
	for(t2 = 0; t2 < FONTWIDTH; t2++)
    { 
	    //Get vertical byte data of char
		if(!inv)
		{
	        colval = xchar[p];
		}
		else
		{
	        colval = ~xchar[p];
		}
	  		
		b = 0;
		x = 1;
        
		for(t1 = 0; t1 < 7; t1++)
        {
            if(colval & x)
            {
	            b += xp2(t1 * 2);
	            b += xp2(t1 * 2 + 1);
            }
            x <<= 1;
	    }
    
        b1 = b & 0xFF; //Lower byte
        b2 = (b & 0xFF00) >> 8; //Upper byte
		
		//Print data to screen
		lcd_gotoxy(col + t2 * 2, row);
		lcd_senddata(b1);
		lcd_gotoxy(col + t2 * 2, row + 1);
		lcd_senddata(b2);
		lcd_gotoxy(col + t2 * 2 + 1, row);
		lcd_senddata(b1);
		lcd_gotoxy(col + t2 * 2 + 1, row + 1);
		lcd_senddata(b2);
		p++;
	}	
    
    lcd_senddata(0x00);
}

//Print string in certain size
void lcd_putstring(int col, int row, char *s, char lsize, int inv)
{
    int c = col;
	
	while(*s)
	{
	    if(!lsize)
		{
	        lcd_putchar1(c, row, *s++, inv);
		}
        else
        {
            lcd_putchar2(c, row, *s++, inv);
		}	
		c += (lsize + 1) * FONTWIDTH;
	}
}

//Print a number
void lcd_putnumber(int col, int row, long num, int dec, int lsize, int inv)
{
    char *s = malloc(16);
	if(s != NULL)
	{
	    int2asc(num, dec, s, 16);
	    lcd_putstring(col, row, s, lsize, inv);
	    free(s);
	}	
}

void lcd_drawbox(int x0, int y0, int x1, int y1)
{
	int t1;
	
	for(t1 = x0 + FONTWIDTH; t1 < x1; t1 += FONTWIDTH)
	{
		lcd_putchar1(t1, y0, 0xC4, 0);
		lcd_putchar1(t1, y1, 0xC4, 0);
	}
	
	for(t1 = y0 + 1; t1 < y1; t1++)
	{
		lcd_putchar1(x0, t1, 0xB3, 0);
		lcd_putchar1(x1, t1, 0xB3, 0);
	}
	
	lcd_putchar1(x0, y0, 0xDA, 0); //Left top corner
	lcd_putchar1(x1, y0, 0xBF, 0); //Right top corner
	lcd_putchar1(x0, y1, 0xC0, 0); //Left bottom corner
	lcd_putchar1(x1, y1, 0xD9, 0); //Right bottom corner
}
		

  //////////////////////
 // STRING FUNCTIONS //
//////////////////////
//INT 2 ASC
int int2asc(long num, int dec, char *buf, int buflen)
{
    int i, c, xp = 0, neg = 0;
    long n, dd = 1E09;

    if(!num)
	{
	    *buf++ = '0';
		*buf = 0;
		return 1;
	}	
		
    if(num < 0)
    {
     	neg = 1;
	    n = num * -1;
    }
    else
    {
	    n = num;
    }

    //Fill buffer with \0
    for(i = 0; i < 12; i++)
    {
	    *(buf + i) = 0;
    }

    c = 9; //Max. number of displayable digits
    while(dd)
    {
	    i = n / dd;
	    n = n - i * dd;
	
	    *(buf + 9 - c + xp) = i + 48;
	    dd /= 10;
	    if(c == dec && dec)
	    {
	        *(buf + 9 - c + ++xp) = '.';
	    }
	    c--;
    }

    //Search for 1st char different from '0'
    i = 0;
    while(*(buf + i) == 48)
    {
	    *(buf + i++) = 32;
    }

    //Add minus-sign if neccessary
    if(neg)
    {
	    *(buf + --i) = '-';
    }

    //Eleminate leading spaces
    c = 0;
    while(*(buf + i))
    {
	    *(buf + c++) = *(buf + i++);
    }
    *(buf + c) = 0;
	
	return c;
}

//STRLEN
int strlen(char *s)
{
   int t1 = 0;

   while(*(s + t1++));

   return (t1 - 1);
}

 ////////////////////////////////////////////
 //     RADIO DATA DISPLAY ROUTINES        // 
////////////////////////////////////////////
//Current frequency (double letter height)
void show_frequency(unsigned long f)
{
	if(f == 0)
	{
	    lcd_putstring(0, 2, "       ", 1, 0);
	}
	else
	{
		lcd_putnumber(0, 2, f / 100, 1, 1, 0);
	}	    
	    
}

//Memeory frequency on selection memplace in menu
void show_frequency2(unsigned long f)
{
	if(!f)
	{
		lcd_putstring(12, 4, "-------", 0, 0);
	}
	else
	{	
        lcd_putstring(12, 4, "       ", 0, 0);
	    lcd_putnumber(12, 4, f / 100, 1, 0, 0);
	}    
}	

void show_sideband(int sb, int invert)
{
	int xpos = 0, ypos = 0, xlen = 3;
    char *sb_str[] = {"USB", "LSB"};
		
	//Clear section of LCD
	lcd_clearsection(xpos * FONTWIDTH, xpos + xlen * 6, ypos);
	
	//Write string to position
	lcd_putstring(xpos * FONTWIDTH, ypos, sb_str[sb], 0, invert);
}

void show_voltage(int v1)
{
    char *buf;
	int t1, p;
	int xpos = 9, ypos = 0, xlen = 5;
	
	lcd_clearsection(xpos * FONTWIDTH, (xpos + xlen) * FONTWIDTH, ypos);
	
	buf = malloc(10);
	//Init buffer string
	for(t1 = 0; t1 < 10; t1++)
	{
	    *(buf + t1) = 0;
	}
    p = int2asc(v1, 1, buf, 6) * FONTWIDTH;
    lcd_putstring(xpos * FONTWIDTH, ypos, buf, 0, 0);
	lcd_putchar1(xpos * FONTWIDTH + p, ypos, 'V', 0);
	free(buf);
}

//S-Meter bargraph (Page 6)
void show_meter(int sv0)
{
    int t1;
    
    int sv = sv0 + (sv0 >> 1);
	
    if(sv > 83)
	{
	    sv = 83;
	}
	
	//Clear bar graph
	for(t1 = 0; t1 < smax - 2; t1 += 2)
	{
	    lcd_gotoxy(t1, 4);
		lcd_senddata(0x00);
	}	
    
	//Draw new bar graph
	for(t1 = 0; t1 < sv; t1 += 2)
	{
	    lcd_gotoxy(t1, 4);
		lcd_senddata(0x7E);
	}	
	
	if(sv > smax)
	{
		smax = sv;
		runseconds10s = runseconds10;
	}	
	
}

//Reset max value of s meter
void reset_smax(void)
{
	int t1;
	
	//Clear bar graph
	for(t1 = 0; t1 < smax; t1 += 2)
	{
	    lcd_gotoxy(t1, 4);
		lcd_senddata(0x00);
	}	
	
	runseconds10s = runseconds10;
	smax = 0;	
}	

void show_mem_addr(int mem_addr, int invert)
{
	int xpos = 0, ypos = 1, xlen = 3;
    unsigned long mem_freq;
    		
	//Clear section of LCD
	lcd_clearsection(xpos * FONTWIDTH, xpos + xlen * 6, ypos);
	
	//Show number of mem place
	lcd_putstring(xpos * FONTWIDTH, ypos, "M", 0, invert);
	if(mem_addr < 10)
	{
	     lcd_putnumber((xpos + 1) * FONTWIDTH, ypos, 0, -1, 0, invert);				
	     lcd_putnumber((xpos + 2) * FONTWIDTH, ypos, mem_addr, -1, 0, invert);				
	}
	else
	{
	     lcd_putnumber((xpos + 1) * FONTWIDTH, ypos, mem_addr, -1, 0, invert);				
	}
	
	//Show respective frequency
	mem_freq = load_frequency(mem_addr);
	if(is_mem_freq_ok(mem_freq))
	{
	    show_mem_freq(mem_freq, invert);
	}
	else    
	{
	    show_mem_freq(0, invert);
	}   
}

void show_mem_freq(unsigned long f, int invert)
{
	
	int xpos = 4, ypos = 1, xlen = 8;
    		
	//Clear section of LCD
	lcd_clearsection(xpos * FONTWIDTH, xpos + xlen * 6, ypos);
	
	if(f)
	{
	    lcd_putnumber(xpos * FONTWIDTH, ypos, f / 100, 1, 0, invert);			
	}   
	else
	{
	    lcd_putstring(xpos * FONTWIDTH, ypos, " ----- ", 0, invert);			
	}    
}	

void show_pa_temp(int patemp)
{
    char *buf;
    int t1, p;
	int xpos = 4, ypos = 0, xlen = 5;
	
	lcd_clearsection(xpos * FONTWIDTH, (xpos + xlen) * 6, ypos);
		
	buf = malloc(10);
	//Init buffer string
	for(t1 = 0; t1 < 10; t1++)
	{
	    *(buf + t1) = 0;
	}
    p = int2asc(patemp / 10, -1, buf, 6);
    lcd_putstring(xpos * FONTWIDTH, ypos, buf, 0, 0);
    lcd_putchar1((xpos + p) * FONTWIDTH, ypos, 0xF8, 0); //Symbol #248
	lcd_putstring((xpos + p + 1) * FONTWIDTH, ypos, "C", 0, 0);
	free(buf);
}

void show_all_data(unsigned long f,  int sb, int v, int mem, int vfo, int spl)
{
	unsigned long f2;
	
	lcd_cls(0, 84, 0, 48);
	
    show_frequency(f);
    show_sideband(sb, 0);
    show_meter_scale(PIND & (1 << PD0));
    show_voltage(v);
    show_pa_temp(get_temp());    				
    show_mem_addr(mem, 0);
	f2 = load_frequency(mem);
	if(is_mem_freq_ok(f2))
	{
		show_mem_freq(f2, 0);
	}
	else
	{
		show_mem_freq(0, 0);
	}
	show_vfo(vfo, spl);
		
}

void show_vfo(int n_vfo, int split)
{
	int xpos = 12, ypos = 1;
	
	//Write string to position
	if(!split)
	{
	    lcd_putchar1(xpos * FONTWIDTH, ypos, n_vfo + 65, 0);
	    lcd_putchar1((xpos + 1) * FONTWIDTH, ypos, 32, 0);
	}   
	else
	{
		if(!n_vfo)
		{
	        lcd_putchar1(xpos * FONTWIDTH, ypos, 65, 1);
	        lcd_putchar1((xpos + 1) * FONTWIDTH, ypos,  66, 0);
	    }   
	    else
	    {
	        lcd_putchar1(xpos * FONTWIDTH, ypos, 65, 0);
	        lcd_putchar1((xpos + 1) * FONTWIDTH, ypos, 66, 1);
	    }   
	}   
}	

void show_meter_scale(int scale)
{
	int t1;
	
	//Define characters for the display line
    char scalestr[] = {0, 76, 146, 146, 100, 0, 0, 254, 0, 0, 0, 68, 130, 146, 108, 0, 0, 0, 94, 146, 146, 98, 0, 0, 0, 194, 34, 18, 14, 0, 0, 0, 76, 146, 146, 108, 0, 0, 0, 16, 56, 16, 0, 254, 0, 120, 132, 132, 120, 0, 0, 0, 16, 56, 16, 0, 228, 146, 146, 140, 0, 120, 132, 132, 120, 0, 0, 0, 112, 136, 136, 254, 0, 254, 146, 146, 108};

	if(!scale)
	{
		lcd_gotoxy(0, 5);
    
        for(t1 = 0; t1 < 77; t1++)
        {
            lcd_senddata(scalestr[t1]);
        }   
	}
	else
	{
		lcd_putstring(0, 5, "2 4 6 8 10W   ", 0, 0);
	}
}			
	
//////////////////////
//   E  E  P  R  O  M
//////////////////////
void store_frequency(long f, int memplace)
{
    long hiword, loword;
    unsigned char hmsb, lmsb, hlsb, llsb;
	
    int start_adr = memplace * 4;
    
	cli();
    hiword = f >> 16;
    loword = f - (hiword << 16);
    hmsb = hiword >> 8;
    hlsb = hiword - (hmsb << 8);
    lmsb = loword >> 8;
    llsb = loword - (lmsb << 8);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr, hmsb);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr + 1, hlsb);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr + 2, lmsb);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr + 3, llsb);
    
    sei();	
}

unsigned long load_frequency(int memplace)
{
    long rf;
    unsigned char hmsb, lmsb, hlsb, llsb;
    int start_adr = memplace * 4;
		
    cli();
    hmsb = eeprom_read_byte((uint8_t*)start_adr);
    hlsb = eeprom_read_byte((uint8_t*)start_adr + 1);
    lmsb = eeprom_read_byte((uint8_t*)start_adr + 2);
    llsb = eeprom_read_byte((uint8_t*)start_adr + 3);
	sei();
	
    rf = (long) 16777216 * hmsb + (long) 65536 * hlsb + (unsigned int) 256 * lmsb + llsb;
		
	return rf;
	
}

void store_last_mem(int mem)
{
	eeprom_write_byte((uint8_t*)127, mem);
}

void store_last_vfo(int vfo)
{
	eeprom_write_byte((uint8_t*)128, vfo);
}

int load_last_mem(void)
{
	return eeprom_read_byte((uint8_t*)127);
}

int load_last_vfo(void)
{
	return eeprom_read_byte((uint8_t*)128);
}

void store_vfo_data(int vfo, unsigned long f0, unsigned long f1)
{
	store_last_vfo(vfo);
	store_frequency(f0, 16); //VFO A
	store_frequency(f1, 17); //VFO B
}	

long recall_mem_freq(void)
{
	int mem_addr = 0;
	int key;
	
	lcd_cls(0, 83, 0, 47);
	lcd_putstring(12, 0, "RECALL QRG", 0, 0);
	
	//Load initial freq
	if(is_mem_freq_ok(load_frequency(mem_addr)))
	{
		show_mem_addr(mem_addr, 0);
	    set_frequency1(load_frequency(mem_addr));
		show_frequency(load_frequency(mem_addr));
	}  
	else  
	{
		show_mem_addr(mem_addr, 0);
		show_frequency(0);
	}	
	
	key = 0;
	while(!key)
	{
		if(tuningknob >= 1)  
		{    
		    if(mem_addr > 0)
		    {
			    mem_addr--;
			}
			else     
			{
			    mem_addr = MAXMEM;
			}	 
			
			tuningknob = 0;
			
			show_mem_addr(mem_addr, 0);
			if(is_mem_freq_ok(load_frequency(mem_addr)))
			{
			    set_frequency1(load_frequency(mem_addr));
			    show_frequency(load_frequency(mem_addr));
			}    
	    }
		
		if(tuningknob <= -1)
		{    
		    if(mem_addr < MAXMEM)
		    {
			    mem_addr++;
			}
			else     
			{
			    mem_addr = 0;
			}	 
			
			tuningknob = 0;
			
			show_mem_addr(mem_addr, 0);
			if(is_mem_freq_ok(load_frequency(mem_addr)))
			{
			    set_frequency1(load_frequency(mem_addr));
			    show_frequency(load_frequency(mem_addr));
            }    
	    }
	    
	    key = get_keys();
	}	
	            
	switch(key)
	{
	    case 2: if(is_mem_freq_ok(load_frequency(mem_addr)))
	            {
	                store_last_mem(mem_addr);
	                return(load_frequency(mem_addr));
	            }    
				break;
	}	
								
    while(get_keys());
    
    return 0;
}	

int save_mem_freq(long f, int mem)
{
    int mem_addr = mem;
	int key;
	
	lcd_cls(0, 83, 0, 47);
	lcd_putstring(12, 0, "STORE QRG", 0, 0);
	
	//Load initial mem
	show_mem_addr(mem_addr, 0);
	set_frequency1(load_frequency(mem_addr));
	show_frequency(f);
			
	key = 0;
	while(!key)
	{
		if(tuningknob >= 1)  
		{    
		    if(mem_addr > 0)
		    {
			    mem_addr--;
			}
			else     
			{
			    mem_addr = MAXMEM;
			}	 
			
			tuningknob = 0;
			
			show_mem_addr(mem_addr, 0);
			if(is_mem_freq_ok(load_frequency(mem_addr)))
			{
			    set_frequency1(load_frequency(mem_addr));
			}    
	    }
		
		if(tuningknob <= -1)
		{    
		    if(mem_addr < MAXMEM)
		    {
			    mem_addr++;
			}
			else     
			{
			    mem_addr = 0;
			}	 
			
			tuningknob = 0;
			
			show_mem_addr(mem_addr, 0);
			if(is_mem_freq_ok(load_frequency(mem_addr)))
			{
			    set_frequency1(load_frequency(mem_addr));
			}    
	    }
	    
	    key = get_keys();
	}	
	            
	switch(key)
	{
	    case 2:     store_last_mem(mem_addr);
	                store_frequency(f, mem_addr);
	                return mem_addr;
	            	break;
	}	
								
    while(get_keys());
    
    return -1;
    
    
}	

  //////////////////////
 //    A   D   C     //
/////////////////////
//Read ADC value
int get_adc(int adc_channel)
{
	int adc_val = 0;
	
	//ADC config and ADC init
    ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1); //Activate ADC, Prescaler=64

	ADMUX = (1 << REFS0) + adc_channel;     //Read ADC channel with Vref=VCC
	
    _delay_ms(3);
	
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
	_delay_ms(3);
	
	ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
	
	adc_val = ADCL;
    adc_val += ADCH * 256;   
	
	ADCSRA &= ~(1<<ADEN); //Deactivate ADC
	
	return adc_val;
}	

//Read keys via ADC0
int get_keys(void)
{

    int key_value[] = {86, 31, 50, 38};
    int t1;
    int adcval = get_adc(0);
        
    //TEST display of ADC value 
    //lcd_putstring(0, 3, "----", 0, 0);    
    //lcd_putnumber(0, 3, adcval, -1, 0, 0);    
    		
    for(t1 = 0; t1 < 4; t1++)
    {
        if(adcval > key_value[t1] - 3 && adcval < key_value[t1] + 3)
        {
             return t1 + 1;
        }
    }
    return 0;
}

//Measure temperature of final amplifier
//Sensor = KTY81-210
int get_temp(void)
{
    double temp;
	double rt, rv = 5100, ut;
	double r0 = 1630; //Resistance of temperature sensor at 0°C
	double m = 17.62;   //slope of temperature sensor in Ohms/K
	
	//Calculate voltage from ADC value
	ut = (double)get_adc(4) * 5 / 1024;
	
	//Calculate thermal resistor value from ADC voltage ut
	rt = rv / (5 / ut - 1);
	
	//Calculate temperature from rt
	temp = 10 * ((rt - r0) / m);
		
	return (int)(temp);
}	

  /////////////////////////
 // INTERRUPT HANDLERS  //
/////////////////////////
//Rotary encoder
ISR(INT0_vect)
{ 
    int gray = (PIND & 0x0C) >> 2;           // Read PD2 and PD3
	int state = (gray >> 1) ^ gray;         // Convert from Gray code to binary

    if((state == 0) || state == 2) 
    {
		tuningknob = 1;
		tuningcount++;
	}
	
	if((state == 1) || state == 3) 
    {
		tuningknob = -1;
		tuningcount++;
	}	
}

//Timer1
ISR(TIMER1_OVF_vect)
{
    runseconds10++;
    tuningcount = 0;
    
    TCNT1 = 63973;
}

  ////////////////////////////////////////////
 //    Store or recall Frequency handling //
//////////////////////////////////////////
int is_mem_freq_ok(unsigned long f)
{
	if(f >= 13900000 && f <= 14400000)
	{
		return 1;
	}	
	else
	{
		return 0;
	}		
}	

//Scan==0: scan memories, scan=1: scan band
long scan(int mode)
{
    int t1 = 0;
    long f0, f1, df, runsecsold10scan = 0;
    int key = 0;
    int sval;
    
    int scan_skip[MAXMEM];
    
    for(t1 = 0; t1 < MAXMEM; t1++)
    {
		scan_skip[t1] = 0;
	}
	
    while(get_keys());
    
    if(!mode)
    {
		key = 0;
        while(key != 1 && key != 2) //Scan memories
	    {
		    t1 = 0;
		    while(t1 < 15 && !key)
		    {
			    f0 = load_frequency(t1);
				if(is_mem_freq_ok(f0) && !scan_skip[t1])
				{
					set_frequency1(f0);
				    show_frequency(f0);
				    show_mem_addr(t1, 0);
				    				    
				    sval = get_adc(2); //ADC voltage on ADC2 SVAL
				    show_meter(sval); //S-Meter
				    while(sval > s_threshold && !key)
				    {
		 	            runsecsold10scan = runseconds10;
		 	            key = get_keys();
		 	            while(runseconds10 < runsecsold10scan + 1 && !key)
		 	            {
							key = get_keys();
					    }	
		 	            sval = get_adc(2);
		 	            show_meter(sval); //S-Meter
		 	        }
					
				    runsecsold10scan = runseconds10;
				    while(runseconds10 < runsecsold10scan + 20 && !key)
			        {
						key = get_keys();
						sval = get_adc(2);
		 	            show_meter(sval); //S-Meter
					}	
			    } 
			    else  
			    {
					key = get_keys();
				}	
				
				while(get_keys());
				
				if(key == 4)
				{
					scan_skip[t1] = 1;
					key = 0;
				}	
				t1++;
				reset_smax();
			}
		}
		t1--;
				
		while(get_keys());
				
		if(key == 2)
		{
			return(t1); //Set this memory frequency as new operating QRG
		}
		else
		{
			return(-1);
		}	
	}
	
	if(mode == 1)  //Scan band
	{			
	    while(!key) 
	    {
	        f0 = scanfreq[0];
			f1 = scanfreq[1];
		    
		    df = 0;
		    
		    while(f0 + df <= f1 && !key)
		    {
		        set_frequency1(f0 + df);
			    show_frequency(f0 + df);
			    df += 100;
			    
			    sval = get_adc(2); //ADC voltage on ADC2 SVAL
			    show_meter(sval); //S-Meter
				
		 	    while(sval > s_threshold && !get_keys())
				{
					runsecsold10scan = runseconds10;
		 	        key = get_keys();
		 	        while(runseconds10 < runsecsold10scan + 1 && !key)
		 	        {
						key = get_keys();
					}	
		 	        sval = get_adc(2);
		 	        show_meter(sval); //S-Meter
		 	    }
			    key = get_keys();
			}
		}
								
		while(get_keys());
				
		if(key == 2)
		{
			return(f0 + df); //Set this memory frequency as new operating QRG
		}
		else
		{
			return(-1);
		}			
	}				    
	return(-1);
}	

//Scans 16 memories
void set_scan_threshold(void)
{
	int xpos0 = 3;
	int ypos0 = 0;
    int key = 0;
    int thresh = s_threshold;
    
    lcd_cls(0, 83, 0, 47);
    show_meter(thresh);
    show_meter_scale(0);
        
    lcd_putstring(6, ypos0, " SCAN THRESH ", 0, 1);
        
    lcd_putstring(xpos0, ypos0 + 2, "  ", 0, 0);
    lcd_putnumber(xpos0, ypos0 + 2, thresh, -1, 0, 0);
    
    	
    while(!key)
    {
        if(tuningknob <= -1) //Turn CW
		{
			if(thresh < 80)
			{
				thresh++;
			}
			show_meter(thresh);
	
            lcd_putstring(xpos0, ypos0 + 2, "  ", 0, 0);
            lcd_putnumber(xpos0, ypos0 + 2, thresh, -1, 0, 0);
    
			tuningknob = 0;
		}

		if(tuningknob >= 1)  //Turn CCW
		{    
			if(thresh > 0)
			{
				thresh--;
			}
			show_meter(thresh);
			 
            lcd_putstring(xpos0, ypos0 + 2, "  ", 0, 0);
            lcd_putnumber(xpos0, ypos0 + 2, thresh, -1, 0, 0);
    
			tuningknob = 0;
		}		
		key = get_keys();
	}
	
	if(key == 2)
	{
		s_threshold = thresh;
		eeprom_write_byte((uint8_t*)129, s_threshold);
	}	
	
}	

//Scans a frequency range defined by 2 edge frequencies
long set_scan_frequency(int fpos, long f0)
{
	int xpos0 = 3;
	int ypos0 = 0;
    int key = 0;
    long f1 = f0;
    
    lcd_cls(0, 83, 0, 47);
        
    lcd_putstring(xpos0, ypos0, "SET SCAN FREQ", 0, 1);
    if(!fpos)
    {
        lcd_putstring(xpos0, ypos0 + 1, "FREQUENCY0:", 0, 0);
    }   
    else
    {
        lcd_putstring(xpos0, ypos0 + 1, "FREQUENCY1:", 0, 0);
    }   
        
    show_frequency(f1);
        	
    while(!key)
    {
        if(tuningknob <= -1) //Turn CW
		{
			if(f1 < 14400000)
			{
				f1 += 100;
			}
				
            show_frequency(f1);
            set_frequency1(f1);
			tuningknob = 0;
		}

		if(tuningknob >= 1)  //Turn CCW
		{    
			if(f1 > 0)
			{
				f1 -= 100;
			}
			 
            show_frequency(f1);
            set_frequency1(f1);
			tuningknob = 0;
		}		
		key = get_keys();
	}
	
	if(key == 2)
	{
		store_frequency(f1, 33 + fpos);
		scanfreq[fpos] = f1;
		return f1;
	}	
	
	return 0;
	
}

//Displays the data of the currently used main VFO
int set_vfo(int xvfo, int xsplit)
{
    //Store values
 	store_frequency(f_vfo[0], 16); //VFO A
	store_frequency(f_vfo[1], 17); //VFO B
	    
	show_vfo(xvfo, xsplit);
	set_frequency1(f_vfo[xvfo]);
	show_frequency(f_vfo[xvfo]);
	store_last_vfo(xvfo);
	
    return xvfo;			        
}	

  //////////
 // MENU //
//////////
void print_menu_head(char *head_str0, char *head_str1, int m_items)
{	
    int xpos0 = 3;
	int ypos0 = 1;
		
	lcd_cls(0, 84, 0, 48);
	lcd_drawbox(34, 0, 80, m_items + 2);
	lcd_putstring(xpos0, ypos0, head_str0, 0, 0);
	lcd_putstring(xpos0, ypos0 + 1, head_str1, 0, 0);
}

void print_menu_item(char *m_str, int ypos, int inverted)
{
	int xpos1= 40;
	
	lcd_putstring(xpos1, ypos + 1, m_str, 0, inverted);
}
	
//Print the itemlist or single item
void print_menu_item_list(int m, int item, int invert)
{
	int menu_items[] =    {3, 1, 3, 1, 2}; 
	
	char *menu_str[5][4] =    {{"VFO A ", "VFO B ", "A=B   ", "B=A   "},
		                       {"RECALL", "STORE ", "      ", "      "}, 
	                           {"MEMORY", "BAND  ", "LIMITS", "THRESH"},
	                           {"ON    ", "OFF   ", "      ", "      "}, 
	                           {"USB   ", "LSB   ", "RESET ", "      "}};
    int t1;
    
    if(item == -1)
    {
        //Print item list for menu
	    for(t1 = 0; t1 < menu_items[m] + 1; t1++)
	    {
		    print_menu_item(menu_str[m][t1], t1, 0);   
	    }	
	}
	else	
	{
		print_menu_item(menu_str[m][item], item, invert);   
	}	
}

//Returns menu_pos if OK or -1 if aborted
int navigate_thru_item_list(int m, int maxitems)
{
	int menu_pos = 0;
	
	print_menu_item_list(m, menu_pos, 1)   ;     //Write 1st entry in normal color
	
	int key = get_keys();
	
    while(key == 0)
	{
		if(tuningknob <= -1) //Turn CW
		{
			print_menu_item_list(m, menu_pos, 0); //Write old entry in normal color
		    if(menu_pos < maxitems)
		    {
				menu_pos++;
			}
			else	
			{
				menu_pos = 0;
			}
			print_menu_item_list(m, menu_pos, 1); //Write new entry in reverse color
		    tuningknob = 0;
		}

		if(tuningknob >= 1)  //Turn CCW
		{    
		    print_menu_item_list(m, menu_pos, 0); //Write old entry in normal color
		    if(menu_pos > 0)
		    {
				menu_pos--;
			}
			else	
			{
				menu_pos = maxitems;
			}
			print_menu_item_list(m, menu_pos, 1); //Write new entry in reverse color
		    tuningknob = 0;
		}		
				
		key = get_keys();
	}
		
	while(get_keys());
	
	switch(key)
	{   case 1: return -1;       //Next menu!
		        break;
	    case 2: return menu_pos; //OK
	            break;
	    case 3: return -3;       //Quit menu!
	            break;
	}
	
	return -1;
}	
			
long menux(long f, int c_vfo)
{
	
	int result = 0;
	int menu;
	int menu_items[] = {3, 1, 3, 1, 2};
	
	////////////////
	// VFO FUNCS  //
	////////////////
	while(get_keys());
		
	menu = 0;
	print_menu_head("VFO", "", menu_items[menu]);	//Head outline of menu
	print_menu_item_list(menu, -1, 0);              //Print item list in full
	
	//Navigate thru item list
	result = navigate_thru_item_list(menu, menu_items[menu]);
	if(result > -1)
	{
		return(menu * 10 + result);
	}
	else
	{
		switch(result)
		{				
		    case -3: return -3; //Quit menu         
		             break;
		    case -1: break;
		}
    }		
    
	////////////////
	//MEMORY FUNCS//
	////////////////
	menu = 1;
	print_menu_head("MEMO", "", menu_items[menu]);	//Head outline of menu
	print_menu_item_list(menu, -1, 0);              //Print item list in full
		
	//Navigate thru item list
	result = navigate_thru_item_list(menu, menu_items[menu]);
	if(result > -1)
	{
		return(menu * 10 + result);
	}
	else
	{
		switch(result)
		{	
		    case -3: return -3; //Quit menu         
		             break;
		    case -1: break;
		}
    }
		
	////////////////
	// SCAN FUNCS //
	////////////////
	while(get_keys());
		
	menu = 2;
    print_menu_head("SCAN", "", menu_items[menu]);	//Head outline of menu
	print_menu_item_list(menu, -1, 0);              //Print item list in full
	   
	//Navigate thru item list
	result = navigate_thru_item_list(menu, menu_items[menu]);
					
	if(result > -1)
	{
		return(menu * 10 + result);
	}
	else
	{
		switch(result)
		{	
		    case -3: return -3; //Quit menu         
		             break;
		    case -1: break;
		}
    }
    
	////////////////
	// SPLIT MODE //
	////////////////
	while(get_keys());
	
	menu = 3;
	print_menu_head("SPLIT", "MODE", menu_items[menu]);	//Head outline of menu
	print_menu_item_list(menu, -1, 0);              //Print item list in full
	
    //Navigate thru item list
	result = navigate_thru_item_list(menu, menu_items[menu]);
					
	if(result > -1)
	{
		return(menu * 10 + result);
	}
	else
	{
		switch(result)
		{	
		    case -3: return -3; //Quit menu         
		             break;
		    case -1: break;
		}
    }
    
	/////////////////
	// LO SET MODE //
	/////////////////
	while(get_keys());
	
	menu = 4;
	print_menu_head("LO", "FREQ", menu_items[menu]);	//Head outline of menu
	print_menu_item_list(menu, -1, 0);              //Print item list in full
	   
	//Navigate thru item list
	result = navigate_thru_item_list(menu, menu_items[menu]);
					
	if(result > -1)
	{
		return(menu * 10 + result);
	}
	else
	{
		switch(result)
		{	
		    case -3: return -3; //Quit menu         
		             break;
		    case -1: break;
		}
    }
    
	return -2; //Nothing to do in main()
}

  //////////
 // MAIN //
//////////
int main(void)
{
	//Universal counter(s)
	int t1; 
	
	unsigned long runseconds10sc = 0;
	
	//ADC
	int adcval;
	
	//Frequencies and VFO
    unsigned long freq_temp;
    int cur_vfo = 0;
    int split = 0;
    
    //Menu
	long menu_ret = 0;
		
    //Counting
    //unsigned long runseconds10a = 0;
    long runseconds10b = 0;
            
    //Key detection
    int key = 0;
    long runseconds10key = 0;

    //Voltage measurement
    double volts0 = 0;
    long runseconds10e = 0;
    int volts1 = 0, volts1_old = 0;

    //Meter
    unsigned long runseconds10c = 0;
        
    //PA temp measurement
    int pa_temp, pa_temp_old = 0;
    unsigned long runseconds10patemp = 0;
    
    //TX/RX detection
    int txrx = 0, txrx_old = 0;
        
    //Sideband detection
    int sideband_old = sideband;
        
    //DDS 1          
    //Set DDRB of DDSPort1 and DDS Resetport  
	DDRB = 0x0F; //SPI-Lines + RESET line on PB0..PB3
	
	//DDS 2
    DDRC = 0x0F;
    
	//Set DDR of LCDPort
	DDRD = 0xF0; //PD0..PD3: Display lines: CLK DIN DC RST
    
    //Input (analog & digital)
    PORTA = 0x01; //Pull-up resistor for key recognition on PA0

    //PORTD pullup
    PORTD = (1 << PD1); //Sideband switch detection
    
	//Interrupt definitions for rotary encoder attached to PD2 and PD3
	EIMSK = (1 << INT0);; //Activate INT0 only
	EICRA = (1 << ISC00);   // Trigger INT0 on pin change
	PCICR = (1 << PCIE0); //Pin Change Interrupt Enable 0
	
    //Timer 1 as 10th-second counter
    TCCR1A = 0;             // normal mode, no PWM
    TCCR1B = (1<<CS12) | (1<<CS10); // Prescaler = /1024 based on system clock 16MHz => 15625 inc per s
	TIMSK1 = (1 << TOIE1);  // overflow active
	TCNT1 = 63973;          // start value for 10 overflows per s
	
	//INIT LCD
	lcd_init();

	//Reset DDS1 (AD9951)
	DDS1_PORT &= ~(1 << DDS1_RESETPIN);          
    _delay_ms(1);
	DDS1_PORT |= (1 << DDS1_RESETPIN);     
       
    //Reset DDS2 (AD9834)	
	_delay_ms(10);	
	DDS2_PORT |= (1 << DDS2_RESETPIN);   //Bit set
    _delay_ms(10);       
	DDS2_PORT &= ~(1 << DDS2_RESETPIN);  //Bit erase        
	_delay_ms(10);	

    //Load scan threshold
    s_threshold = eeprom_read_byte((uint8_t*)129);          	
    if(s_threshold < 0 || s_threshold > 80)
    {
		s_threshold = 30;
	}
	
	//Load scan edge frequencies
	scanfreq[0] = load_frequency(33);
	if(!is_mem_freq_ok(scanfreq[0]))
	{
		scanfreq[0] = 14100000;
	}	
	scanfreq[1] = load_frequency(34);
	if(!is_mem_freq_ok(scanfreq[1]))
	{
		scanfreq[1] = 14300000;
	}
		
    //Load last stored freqeuncy
    //Check if memory place is not a random number anywhere in EEPROM
    last_memplace = load_last_mem();
    if(last_memplace < 0 || last_memplace > 15)
    {
		last_memplace = 0;
	}
	
	if(is_mem_freq_ok(load_frequency(last_memplace)))
	{
		show_mem_freq(load_frequency(last_memplace), 0);
	}
	else
	{	
		show_mem_freq(0, 0);	
	}	
	
	//Load LO frequencies set by user
	for(t1 = 0; t1 < 2; t1++)
	{
		f_lo[t1] = load_frequency(35 + t1);
		if(f_lo[t1] < 8995000 || f_lo[t1] > 9005000)
		{
			switch(t1)
			{
				case 0: f_lo[t1] = 9001500; //USB
				         break;
				case 1: f_lo[t1] = 8998500; //LSB
				         break;        
			}
		}
	}			          
				
	//Load VFO data
	//Set VFO 
    cur_vfo = load_last_vfo();
    if(cur_vfo < 0 || cur_vfo > 1)
    {
		cur_vfo = 0;
	}
	for(t1 = 0; t1 < 2; t1++)    
	{
        f_vfo[t1] = load_frequency(16 + t1);

        //Check if freq is in 20m band
        if(!is_mem_freq_ok(f_vfo[t1]))
        {
		    f_vfo[t1] = 14220000;
		    store_frequency(f_vfo[t1], 16 + t1);
		}    
    }
    //Set this frequency
    set_frequency1(f_vfo[cur_vfo]);
        
    //Set LO
    set_frequency2(f_lo[sideband]); 
    set_frequency2(f_lo[sideband]); 
    set_frequency2(f_lo[sideband]); 
    
    //Initial voltage measurement
    volts0 = (double) get_adc(1) * 5 / 1024 * VOLTAGEFACTOR * 10; 
	volts1 = (int) volts0;
	
	//Fill lcd with all available information 
    show_all_data(f_vfo[cur_vfo], sideband, volts1, last_memplace, cur_vfo, split);
        
	sei();
	
	runseconds10b = runseconds10;           	
    
    for(;;) 
	{
		if(tuningknob <= -1)
		{
		    f_vfo[cur_vfo] += calc_tuningfactor();    
		    tuningknob = 0;
			set_frequency1(f_vfo[cur_vfo]);    		 
	        show_frequency(f_vfo[cur_vfo]);    		
		}

		if(tuningknob >= 1)  
		{    
		    f_vfo[cur_vfo] -= calc_tuningfactor();    
		    tuningknob = 0;
			set_frequency1(f_vfo[cur_vfo]);    		 
			show_frequency(f_vfo[cur_vfo]);    	
		}		
        
		//Check if key pressed
        if(runseconds10 > runseconds10key + 1)
        {
		    key = get_keys();
		    runseconds10key = 0;
		}   	
		
		switch(key)
		{
			case 1: menu_ret = menux(f_vfo[cur_vfo], cur_vfo);    //Return values: 0..15: Recall MEM
			                                            //               16..31: Store current freq in MEM
			        lcd_cls(0, 83, 0, 47);	            //               32    : Scan MEMs via function scan(int)
	                while(get_keys());                  // 33: Scan SSB portion , 34: scan CW portion, 
	                                                    // 64 : Split TX: A, RX B:, 65 vice versa, 66: Split off
	                volts0 = (double) get_adc(1) * 5 / 1024 * VOLTAGEFACTOR * 10; //Refresh voltage measurement
	                volts1 = (int) volts0;
	                
	                //Fill lcd with all information available
                    show_all_data(f_vfo[cur_vfo], sideband, volts1, last_memplace, cur_vfo, split);
    		        	
	                key = 0;
			        
			        //React to user's request in menu
			        switch(menu_ret)
			        {
						case 0:     cur_vfo = 0;  //Set to VFO A
						            show_vfo(0, 0);
						            set_frequency1(f_vfo[cur_vfo]);
						            show_frequency(f_vfo[cur_vfo]);
						            break;
						        
						case 1:     cur_vfo = 1;   //Set to VFO B
						            show_vfo(1, 0);
						            set_frequency1(f_vfo[cur_vfo]);
						            show_frequency(f_vfo[cur_vfo]);
						            break;
						        
						case 2:     f_vfo[0] = f_vfo[1]; //VFO A = VFO B
						            break; 
						        
						case 3:     f_vfo[1] = f_vfo[0]; //VFO B = VFO A
					                break;
					    
					    case 10:    freq_temp = recall_mem_freq();     //Recall QRG  
					                if(is_mem_freq_ok(freq_temp))
					                {
										f_vfo[cur_vfo] = freq_temp;
										set_frequency1(f_vfo[cur_vfo]);
										show_frequency(f_vfo[cur_vfo]);
										last_memplace = load_last_mem();
										show_mem_addr(last_mem, 0);
									}	
									else
									{
										set_frequency1(f_vfo[cur_vfo]);
										show_frequency(f_vfo[cur_vfo]);
									}	
					                break;
					    
					    case 11:    t1 = save_mem_freq(f_vfo[cur_vfo], last_memplace);
					                
					                if(t1 > -1)
					                {
										store_frequency(f_vfo[cur_vfo], 16 + cur_vfo);
										last_memplace = t1;
										store_last_mem(t1);
					    			}    
					    			show_frequency(f_vfo[cur_vfo]);
					                set_frequency1(f_vfo[cur_vfo]);
					                
					                break;
					                
					    case 20:    t1 = scan(0);
					                freq_temp = load_frequency(t1);
					                if(is_mem_freq_ok(freq_temp))
					                {
										f_vfo[cur_vfo] = freq_temp;
										set_frequency1(f_vfo[cur_vfo]);
										show_frequency(f_vfo[cur_vfo]);
									}	
					                break;
					    
					    case 21:    freq_temp = scan(1);
					                if(is_mem_freq_ok(freq_temp))
					                {
										f_vfo[cur_vfo] = freq_temp;
										set_frequency1(f_vfo[cur_vfo]);
										show_frequency(f_vfo[cur_vfo]);
									}	
					                break;
					                  
					    case 22:    scanfreq[0] = set_scan_frequency(0, f_vfo[cur_vfo]);
					                scanfreq[1] = set_scan_frequency(1, f_vfo[cur_vfo]); 
					                break;
					                
					    case 23:    set_scan_threshold();            
					                break;
					                
					    case 30:  	split = 1;
					                if(cur_vfo == 0)
						            {
										vfo_x = 0;
										vfo_y = 1;
									}	
									else
									{
										vfo_x = 1;
										vfo_y = 0;
									}	
						            show_vfo(cur_vfo, 1);
						            break;
						
						case 31:  	split = 0;
						            show_vfo(cur_vfo, 0);
						            break;
						
						case 40:    set_lo_freq(0);
						            break;       
						              
						case 41:    set_lo_freq(1);
						            break;   
						            
						case 42:    f_lo[0] = 9001500;
						            store_frequency(f_lo[0], 35);
						            f_lo[1] = 8998500;
						            store_frequency(f_lo[1], 36);
						            set_frequency2(f_lo[sideband]);
						            break;                                 
					}	
					show_all_data(f_vfo[cur_vfo], sideband, volts1, last_memplace, cur_vfo, split);
					break;
					
			case 2: store_last_vfo(cur_vfo);
			        store_frequency(f_vfo[0], 16); //VFO A
			        store_frequency(f_vfo[1], 17); //VFO B
			        store_last_mem(last_memplace);
			        break;
			        
 			case 4: while(get_keys());
 			        //Swap VFOs
				
 			        //Store values
 			        store_frequency(f_vfo[0], 16); //VFO A
			        store_frequency(f_vfo[1], 17); //VFO B
			        if(cur_vfo)
			        {
			            cur_vfo = 0;
			        }
			        else
			        {
			            cur_vfo = 1;
			        }
			       
			        show_vfo(cur_vfo, split);
			        set_frequency1(f_vfo[cur_vfo]);
			        show_frequency(f_vfo[cur_vfo]);
			        store_last_vfo(cur_vfo);
			        key = 0;
			        break;
 		}	

		//Show voltage every sec
        if(runseconds10 > runseconds10e + 10)
        {
			volts0 = (double) get_adc(1) * 5 / 1024 * VOLTAGEFACTOR * 10; 
		    volts1 = (int) volts0;
   		    if(volts1 != volts1_old)
		    {
    	        show_voltage(volts1);
	     		volts1_old = volts1;
		    }	
		    runseconds10e = runseconds10;
		}   	
		
		//PA temp every sec
		if(runseconds10 > runseconds10patemp + 10)
        {
			pa_temp = get_temp();
			if(pa_temp != pa_temp_old)
			{
		        show_pa_temp(pa_temp);
		        pa_temp_old = pa_temp;
		    }    
		    runseconds10patemp = runseconds10;
		}    
		
		//METER
		//After 1/10th sec check S-Val resp. PWR value
		if(runseconds10 > runseconds10c)
		{
			if(!txrx)
		 	{
				show_meter(get_adc(2)); //S-Meter * 1
		 	}
		 	else
		 	{
				adcval = get_adc(3);
				show_meter((adcval >> 1)); //*0.5
			}    
 		 	runseconds10c = runseconds10;
 		}	
		
		//Sideband detection via PORTD (PD1);
		if(!(PIND & (1 << PD1)))
		{
			sideband = 1;
			set_frequency1(f_vfo[cur_vfo]);    		 
	        show_frequency(f_vfo[cur_vfo]);    		
		}		
		else
		{
			sideband = 0;
			set_frequency1(f_vfo[cur_vfo]);    		 
	        show_frequency(f_vfo[cur_vfo]);    		
		}		
		
		if(sideband_old != sideband)
		{
		    set_frequency2(f_lo[sideband]);
			sideband_old = sideband;
			show_sideband(sideband, 0);
		}
		
		//TX/RX detection via PORTD (PD0);
		if(!(PIND & (1 << PD0)))
		{
			txrx = 0;
		}		
		else
		{
			txrx = 1;
		}		
				
		if(txrx_old != txrx) //PTT switched
		{
		    show_meter_scale(txrx);
		    txrx_old = txrx;
		    show_meter(0);
		    
		    //Set frequency if SPLIT activated
		    if(split)
		    {
		        if(txrx)
		        {
		            cur_vfo = vfo_y;       // TX
			        
			    }
		        else	
		        {
			        cur_vfo = vfo_x;       // RX    
			    }   
			    show_vfo(cur_vfo, split);
			    set_frequency1(cur_vfo);
			    show_frequency(cur_vfo);    
			         
			}
		}
				
		//Store VFO data every 10 minutes
		if(runseconds10 > runseconds10b + 6000)
		{
			store_vfo_data(cur_vfo, f_vfo[0], f_vfo[1]);
			lcd_putchar1(13 * 6, 1, '.', 0);
			runseconds10b = runseconds10;
		}
		
		//Delete max value of meter every 2 seconds
		if(runseconds10 > runseconds10s + 20)
		{
			reset_smax();
			show_meter(get_adc(2));			
			
		}	
		
		if(runseconds10 > runseconds10sc + 5)
		{
			lcd_putchar1(13 * 6, 4, '*', 0);
		}	
		else
		{
			lcd_putchar1(13 * 6, 4, '.', 0);
		}	
		
		if(runseconds10 > runseconds10sc + 10)
		{
			runseconds10sc = runseconds10;
			lcd_putchar1(13 * 6, 4, ' ', 0);
		}	
		
		//Reset all timer variables
		if(runseconds10 > 16777216)
 		{
			runseconds10 = 0;
		    runseconds10e = 0;
            runseconds10key = 0;
            runseconds10patemp = 0;
            runseconds10sc = 0;
    	}	
		
    }
    return 0;
}
