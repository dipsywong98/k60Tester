/*
 * test.h
 *
 *  Created on: Jul 9, 2017
 *      Author: dipsy
 */

#ifndef INC_TEST_H_
#define INC_TEST_H_

#include "libsc/led.h"
#include "libsc/joystick.h"
#include "libsc/k60/ov7725.h"
#include "libsc/battery_meter.h"
#include "libsc/lcd_console.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/futaba_s3010.h"
#include "libsc/dir_motor.h"
#include "libsc/alternate_motor.h"
#include "libsc/dir_encoder.h"
#include "libsc/ab_encoder.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libsc/lcd_console.h"

#include "debug_console.h"

using namespace libsc;

namespace test{

void led();		//flash program and run
void battery(); //battery and lcd
void joystick();
void camera();
void servo();
void motor();
void alternateMotor();
void encoder();
void abencoder();
void bluetooth();
void ServoRange();

}



#endif /* INC_TEST_H_ */
