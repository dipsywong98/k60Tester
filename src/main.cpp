/* main.cpp
 * Copyright (c) 2014-2018 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Dipsy
 *
 */
#include "libbase/k60/mcg.h"
#include "libsc/system.h"
#include "libsc/battery_meter.h"
#include "libsc/lcd.h"
#include "libsc/st7735r.h"
#include "libsc/lcd_console.h"
#include "libsc/led.h"
#include "libsc/joystick.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/ab_encoder.h"
#include "libbase/k60/flash.h"
#include "libsc/k60/touchscreen_lcd.h"
#include "libbase/k60/gpio.h"
#include "libbase/k60/spi_master.h"

#include <vector>
#include <functional>

#include "test.h"
#include "decoder.h"


#define    IODIRA    (0x00)      // MCP23x17 I/O Direction Register
#define    IODIRB    (0x01)      // 1 = Input (default), 0 = Output

#define    IPOLA     (0x02)      // MCP23x17 Input Polarity Register
#define    IPOLB     (0x03)      // 0 = Normal (default)(low reads as 0), 1 = Inverted (low reads as 1)

#define    GPINTENA  (0x04)      // MCP23x17 Interrupt on Change Pin Assignements
#define    GPINTENB  (0x05)      // 0 = No Interrupt on Change (default), 1 = Interrupt on Change

#define    DEFVALA   (0x06)      // MCP23x17 Default Compare Register for Interrupt on Change
#define    DEFVALB   (0x07)      // Opposite of what is here will trigger an interrupt (default = 0)

#define    INTCONA   (0x08)      // MCP23x17 Interrupt on Change Control Register
#define    INTCONB   (0x09)      // 1 = pin is compared to DEFVAL, 0 = pin is compared to previous state (default)

#define    IOCON     (0x0A)      // MCP23x17 Configuration Register
//                   (0x0B)      //     Also Configuration Register

#define    GPPUA     (0x0C)      // MCP23x17 Weak Pull-Up Resistor Register
#define    GPPUB     (0x0D)      // INPUT ONLY: 0 = No Internal 100k Pull-Up (default) 1 = Internal 100k Pull-Up

#define    INTFA     (0x0E)      // MCP23x17 Interrupt Flag Register
#define    INTFB     (0x0F)      // READ ONLY: 1 = This Pin Triggered the Interrupt

#define    INTCAPA   (0x10)      // MCP23x17 Interrupt Captured Value for Port Register
#define    INTCAPB   (0x11)      // READ ONLY: State of the Pin at the Time the Interrupt Occurred

#define    GPIOA     (0x12)      // MCP23x17 GPIO Port Register
#define    GPIOB     (0x13)      // Value on the Port - Writing Sets Bits in the Output Latch

#define    OLATA     (0x14)      // MCP23x17 Output Latch Register
#define    OLATB     (0x15)      // 1 = Latch High, 0 = Latch Low (default) Reading Returns Latch State, Not Port Value!

#define    OPCODEW       (0b01000000)  // Opcode for MCP23S17 with LSB (bit0) set to write (0), address OR'd in later, bits 1-3
#define    OPCODER       (0b01000001)  // Opcode for MCP23S17 with LSB (bit0) set to read (1), address OR'd in later, bits 1-3
#define    ADDR_ENABLE   (0b00001000)  // Configuration register for MCP23S17, the only thing we change is enabling hardware addressing



namespace libbase {
namespace k60 {
Mcg::Config Mcg::GetMcgConfig() {
  Mcg::Config config;
  config.external_oscillator_khz = 50000;
  config.core_clock_khz = 150000;
  return config;
}
}  // namespace k60
}  // namespace libbase

using libsc::System;
using namespace libsc;
using namespace libbase::k60;
using libsc::k60::TouchScreenLcd;
using std::vector;


#define byteWrite(address, reg, value) spi.ExchangeData(0,OPCODEW | (address << 1));spi.ExchangeData(0,reg);spi.ExchangeData(0,value);


int main() {
	System::Init();

	Led::Config led_config;
	led_config.id = 0;
	Led led0(led_config);

	SpiMaster::Config config;
	SpiMaster::Config::Slave slave;
	slave.cs_pin = Pin::Name::kPtb20;
	slave.is_active_high = true;
	config.baud_rate_khz = 10000;
	config.sin_pin = Pin::Name::kPtb23;
	config.sout_pin = Pin::Name::kPtb22;
	config.sck_pin = Pin::Name::kPtb21;
	config.slaves[0] = slave;

	SpiMaster spi(config);

	St7735r::Config lcd_config;
	St7735r lcd(lcd_config);
	lcd.Clear();

	LcdTypewriter::Config writerconfig;
	writerconfig.lcd = &lcd;
	LcdTypewriter writer(writerconfig);

	Gpo::Config gpoconfig;
	gpoconfig.pin = Pin::Name::kPtb19;
	Gpo ss = Gpo(gpoconfig);

	byteWrite(0,IOCON, ADDR_ENABLE);
	while(1){
		ss.Reset();
		uint16_t data1 = spi.ExchangeData(0,OPCODER);
		uint16_t data2 = spi.ExchangeData(0,0x12);
		uint16_t data3 = spi.ExchangeData(0,0x13);
		uint16_t data4 = spi.ExchangeData(0,0x00);
		ss.Set();
//		uint16_t data2 = spi.ExchangeData(0,(0b01000001<<8)|0x13);
		char c[20];
		sprintf(c,"%d %d %d %d",data1,data2,data3,data4);
		lcd.SetRegion({0,0,128,20});
		writer.WriteString(c);

		bool state[16];
		uint8_t mask = 1;
		for(int i=0; i<8; i++){
			state[i] = data3 & mask;
			mask<<=1;
		}
		mask = 1;
		for(int i=8; i<16; i++){
			state[i] = data4 & mask;
			mask<<=1;
		}
		lcd.SetRegion({0,60,100,15});
		sprintf(c,"%d%d%d%d %d%d%d%d",state[0],state[1],state[2],state[3],state[4],state[5],state[6],state[7]);
		writer.WriteString(c);

		lcd.SetRegion({0,80,100,15});
		sprintf(c,"%d%d%d%d %d%d%d%d",state[8],state[9],state[10],state[11],state[12],state[13],state[14],state[15]);
		writer.WriteString(c);

		led0.Switch();
		System::DelayMs(100);
	}

	return 0;
}
