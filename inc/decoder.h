/*
 * decoder.h
 *
 *  Created on: Jul 18, 2018
 *      Author: dipsy
 */

#ifndef INC_DECODER_H_
#define INC_DECODER_H_

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

#include <vector>
#include <functional>

#include "test.h"

namespace test{


using libsc::System;
using namespace libsc;
using namespace libbase::k60;
using libsc::k60::TouchScreenLcd;
using std::vector;

std::function<void(void)> Update;

void GPIListener(Gpi *gpi) {
//	while(1);
    Update();
}

void decoder() {
	bool state[16]={};

  	St7735r::Config lcd_config;
	St7735r lcd(lcd_config);
	lcd.Clear();

	LcdTypewriter::Config writerconfig;
	writerconfig.lcd = &lcd;
	LcdTypewriter writer(writerconfig);

	Led::Config led_config;
	led_config.id = 0;
	Led led0(led_config);
	led_config.id = 1;
	Led led1(led_config);
	led_config.id = 2;
	Led led2(led_config);
	led_config.id = 3;
	Led led3(led_config);

	AbEncoder::Config encoder_config;
	encoder_config.id = 0;
	AbEncoder encoder(encoder_config);

	uint32_t ftmcnt = 0;
	uint32_t lcnt = 0;
	uint32_t fuck = 0;

	Pin::Name pins[16] = {		//on schematics
		Pin::Name::kPtb16,	//DMA0
		Pin::Name::kPtb17,
		Pin::Name::kPtb18,
		Pin::Name::kPtb19,
		Pin::Name::kPtb20,
		Pin::Name::kPtb21,
		Pin::Name::kPtb22,
		Pin::Name::kPtb23,	//DMA7
		Pin::Name::kPtc0,	//SCCB_SCL
		Pin::Name::kPtc1,	//SCCB_SDA
		Pin::Name::kPta19,	//a19
		Pin::Name::kPta24,	//a24
		Pin::Name::kPtb6,	//pclk
		Pin::Name::kPtc2,	//vsync
		Pin::Name::kPtb11,	//href
		Pin::Name::kPtb1
	};

	vector<Gpi> gpis;
	Gpi::Config ConfigGPI;
	ConfigGPI.interrupt = Pin::Config::Interrupt::kBoth;
	ConfigGPI.isr = GPIListener;
	ConfigGPI.config.set(Pin::Config::kPassiveFilter);
	for(int i=0; i<16; i++){
		ConfigGPI.pin = pins[i];
		gpis.emplace_back(ConfigGPI);
	}

	ConfigGPI.pin = Pin::Name::kPtb0;
	Gpi test(ConfigGPI);

	Gpo::Config ConfigGPO;
	ConfigGPO.pin = Pin::Name::kPte25;
	Gpo out(ConfigGPO);

	int trigger_time = 0;

	Update = [&](){
		led0.Switch();
		encoder.Update();
		int32_t delta = encoder.GetCount();
		if(delta>=1000){
			ftmcnt+=lcnt;
		} else {
			ftmcnt += lcnt = delta;
		}
		char buf[20];
		int cnt = 0, cnt2 = 0;
		for(int i=15; i>=0; i--){
			cnt<<=1;
			cnt2<<=1;
			cnt2=gpis[15-i].Get();
			cnt|=gpis[i].Get();
			state[i] = gpis[i].Get();
		}
		led3.SetEnable(fuck>cnt);
		fuck = cnt;
		lcd.SetRegion({0,0,100,15});
		sprintf(buf,"%d; %d",cnt,ftmcnt%256);
		writer.WriteString(buf);

		lcd.SetRegion({0,20,100,15});
		sprintf(buf,"%d; %d",cnt2,ftmcnt%65536);
		writer.WriteString(buf);

		lcd.SetRegion({0,40,100,15});
		sprintf(buf,"%d",test.Get());
		writer.WriteString(buf);

		lcd.SetRegion({0,60,100,15});
		sprintf(buf,"%d%d%d%d %d%d%d%d",state[0],state[1],state[2],state[3],state[4],state[5],state[6],state[7]);
		writer.WriteString(buf);

		lcd.SetRegion({0,80,100,15});
		sprintf(buf,"%d%d%d%d %d%d%d%d",state[8],state[9],state[10],state[11],state[12],state[13],state[14],state[15]);
		writer.WriteString(buf);
		trigger_time++;
	};

	bool flag = false;
	while(1){
		led2.SetEnable(test.Get());
		led1.Switch();
		out.Set(flag = !flag);
	    System::DelayMs(250);
	}
}
}



#endif /* INC_DECODER_H_ */
