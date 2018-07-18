/*
 * test.cpp
 *
 *  Created on: Jul 9, 2017
 *      Author: dipsy
 */

#include "test.h"

using namespace libsc;
using namespace libsc::k60;

namespace test{
uint16_t xd = 100;

int32_t encoder_value0 = 0,encoder_value1 = 0;
uint16_t servoAngle = 0;
int32_t motorSpeed = 0;
int car = 0;
bool yo = false;

void battery(){

	BatteryMeter::Config ConfigBM;
	ConfigBM.voltage_ratio = 0.4;
	BatteryMeter bm(ConfigBM);

	St7735r::Config lcd_config;
	St7735r lcd(lcd_config);
	lcd.Clear();

	LcdConsole::Config console_config;
	console_config.lcd = &lcd;
	LcdConsole console(console_config);

	float voltage;
	do {
	  voltage = bm.GetVoltage();

	  console.SetTextColor(voltage <= 7.4 ? Lcd::kRed : Lcd::kGreen);

	  char temp[32];
	  sprintf(temp, " Voltage: %.2fV", voltage);
	  console.WriteString(temp);

	  System::DelayMs(1000);
	} while (voltage <= 7.4);
}

void led(){
	  Led::Config led_config;
	  led_config.id = 0;
	  Led led0(led_config);
	  led0.SetEnable(1);
	  led_config.id = 1;
	  Led led1(led_config);
	  led1.SetEnable(0);
	  led_config.id = 2;
	  Led led2(led_config);
	  led2.SetEnable(0);
	  led_config.id = 3;
	  Led led3(led_config);
	  led3.SetEnable(1);

	  while(1){
		  led0.Switch();
		  led1.Switch();
		  led2.Switch();
		  led3.Switch();
		  System::DelayMs(250);
	  }
}

void joystick(){

	Joystick::Config joystick_config;
	joystick_config.id=0;
	joystick_config.is_active_low = true;
	Joystick joystick(joystick_config);

	St7735r::Config lcd_config;
	St7735r lcd(lcd_config);

	while(1){
		switch(joystick.GetState()){
			case Joystick::State::kDown:
				lcd.FillColor(Lcd::kBlack);
				break;
			case Joystick::State::kIdle:
				lcd.FillColor(Lcd::kBlue);
				break;
			case Joystick::State::kLeft:
				lcd.FillColor(Lcd::kCyan);
				break;
			case Joystick::State::kRight:
				lcd.FillColor(Lcd::kGreen);
				break;
			case Joystick::State::kSelect:
				lcd.FillColor(Lcd::kPurple);
				break;
			case Joystick::State::kUp:
				lcd.FillColor(Lcd::kRed);
				break;

		}
	}
}

void camera(){
	St7735r::Config lcd_config;
	St7735r lcd(lcd_config);

	Ov7725::Config cameraConfig;
	cameraConfig.id = 0;
	cameraConfig.w = 80;
	cameraConfig.h = 60;
	cameraConfig.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
	cameraConfig.contrast=0x40;
	k60::Ov7725 cam(cameraConfig);
	cam.Start();


	while(1){
		if(System::Time()%50==0 && cam.IsAvailable()){
			lcd.SetRegion(Lcd::Rect(0,0,80,60));
			lcd.FillBits(Lcd::kBlack,Lcd::kWhite,cam.LockBuffer(),8*cam.GetBufferSize());
			cam.UnlockBuffer();
		}
	}
}

void servo(){
	uint16_t servoAngle = 800;

	FutabaS3010::Config ConfigServo;
	ConfigServo.id = 0;
	FutabaS3010 servo(ConfigServo);
	servo.SetDegree(servoAngle);

	while(1){
		++servoAngle%=1000;
		servoAngle+=400;
	}
}

void motor(){
	DirMotor::Config ConfigMotor;
	ConfigMotor.id = 0;
	DirMotor motor0(ConfigMotor);
	ConfigMotor.id = 1;
	DirMotor motor1(ConfigMotor);
	motor1.SetClockwise(false);

	motor0.SetPower(200);
	motor1.SetPower(200);
}

void alternateMotor(){

	int32_t speed = 0;

	AlternateMotor::Config ConfigMotor;
	ConfigMotor.id = 0;
	AlternateMotor motor0(ConfigMotor);
	ConfigMotor.id = 1;
	AlternateMotor motor1(ConfigMotor);

	St7735r::Config lcd_config;
	lcd_config.orientation = 0;
	St7735r lcd(lcd_config);
	lcd.Clear();

	LcdTypewriter::Config writerconfig;
	writerconfig.lcd = &lcd;
	LcdTypewriter writer(writerconfig);

	Joystick::Config joystick_config;
	joystick_config.id = 0;
	joystick_config.is_active_low = true;
	Joystick joystick(joystick_config);

	DebugConsole console(&joystick,&lcd,&writer,10);
	console.PushItem("speed", &speed, 100);
	console.ListItems();
	while(1){
		if(System::Time()%100 ==0 ){
			console.Listen();
			console.ListItemValues();
			motor0.SetPower(abs(speed));
			motor1.SetPower(abs(speed));
			motor1.SetClockwise(speed>0);
			motor0.SetClockwise(speed>0);
		}
	}

}

void encoder(){
	DirEncoder::Config ConfigEncoder;
	ConfigEncoder.id = 0;
	DirEncoder encoder0(ConfigEncoder);
	ConfigEncoder.id = 1;
	DirEncoder encoder1(ConfigEncoder);

	St7735r::Config lcd_config;
	St7735r lcd(lcd_config);
	lcd.Clear();

	LcdTypewriter::Config writerconfig;
	writerconfig.lcd = &lcd;
	LcdTypewriter writer(writerconfig);

	while (true){

		if(System::Time()%100==0){
			encoder0.Update();
			encoder1.Update();
			char buff[10];
			sprintf(buff,"%d\n%d",encoder_value0+=encoder0.GetCount(),encoder_value1+=encoder1.GetCount());
			writer.WriteBuffer(buff,10);
		}

	}
}

void abencoder(){
	AbEncoder::Config ConfigEncoder;
	ConfigEncoder.id = 0;
	AbEncoder encoder0(ConfigEncoder);
	ConfigEncoder.id = 1;
	AbEncoder encoder1(ConfigEncoder);

	St7735r::Config lcd_config;
	St7735r lcd(lcd_config);
	lcd.Clear();

	LcdTypewriter::Config writerconfig;
	writerconfig.lcd = &lcd;
	LcdTypewriter writer(writerconfig);

	while (true){

		if(System::Time()%100==0){
			encoder0.Update();
			encoder1.Update();
			char buff[10];
			sprintf(buff,"%d\n%d",encoder_value0+=encoder0.GetCount(),encoder_value1+=encoder1.GetCount());
			writer.WriteBuffer(buff,10);
		}

	}
}

int hi = 0;

JyMcuBt106 *pBt = nullptr;

bool bluetoothListener(const Byte *data, const size_t size) {
	if(pBt)pBt->SendBuffer(data,size);
	return 1;
}

void bluetooth(){
	JyMcuBt106::Config bt_config;
	bt_config.id = 0;
	bt_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	bt_config.rx_isr = &bluetoothListener;
	JyMcuBt106 bt(bt_config);
	pBt = &bt;

	Led::Config led_config;
	led_config.id=0;
	Led led(led_config);

	char buf[8]="hello";
	while(1){
//		bt.SendStr(buf);
		led.Switch();
		System::DelayMs(250);
	}
}

void Reset(){
	if (car==0){
		servoAngle = 755;
	}
	else{
		servoAngle = 845;
	}

	encoder_value0 = 0;
	encoder_value1 = 0;


}

void Clear(){
	encoder_value0 = 0;
	encoder_value1 = 0;
}

void ServoRange(){
	Led::Config ConfigLed;
	  ConfigLed.is_active_low = true;
	  ConfigLed.id = 0;
	  Led led0(ConfigLed);
	  ConfigLed.id=1;
	  Led led1(ConfigLed);

	  led0.SetEnable(true);

	  FutabaS3010::Config ConfigServo;
	  ConfigServo.id = 0;
	  FutabaS3010 servo(ConfigServo);
	  servo.SetDegree(servoAngle);

	  Reset();

	  DirMotor::Config ConfigMotor;
	ConfigMotor.id = 0;
	DirMotor motor0(ConfigMotor);
	ConfigMotor.id = 1;
	DirMotor motor1(ConfigMotor);
//	ConfigMotor.id = 2;
//	DirMotor motor2(ConfigMotor);

	motor0.SetPower(0);
	motor1.SetPower(0);

	  DirEncoder::Config ConfigEncoder;
	  ConfigEncoder.id = 0;
	  DirEncoder encoder0(ConfigEncoder);
	  ConfigEncoder.id = 1;
	  DirEncoder encoder1(ConfigEncoder);


	  St7735r::Config lcd_config;
	  lcd_config.orientation = 0;
	  St7735r lcd(lcd_config);
	  lcd.Clear();

	  LcdTypewriter::Config writerconfig;
	  writerconfig.lcd = &lcd;
	  LcdTypewriter writer(writerconfig);

	  Joystick::Config joystick_config;
	  joystick_config.id = 0;
	  joystick_config.is_active_low = true;
	  Joystick joystick(joystick_config);

	  DebugConsole console(&joystick,&lcd,&writer,10);
	  console.PushItem("servo_angle",&servoAngle,1);
	  console.PushItem("encoder0",&encoder_value0,float(0.0));
	  console.PushItem("encoder1",&encoder_value1,float(0.0));
	  console.PushItem("reset",(bool*)(nullptr),"","");
	  console.PushItem("0/1800",&yo,"1800","0");
	  console.PushItem("motorSpeed",&motorSpeed,10);
	  Item item = console.GetItem(3);
	  item.listener = &Clear;
	  console.SetItem(3,item);

	  console.ListItems();

	  while (true){
		  servoAngle = yo*1800;
		  console.Listen();
		  encoder0.Update();
		  encoder1.Update();
		  int dx=encoder0.GetCount(),dy=encoder1.GetCount();
		  if(dx>1000||dx<-1000||dy>1000||dy<-1000)continue;
		  encoder_value0 += dx;
		  encoder_value1 += dy;
		  if(System::Time()%500==0){
			  console.ListItems();
		  }

		  servo.SetDegree(servoAngle);
		  motor0.SetClockwise(motorSpeed<0);
		  motor1.SetClockwise(motorSpeed<0);
//		  motor2.SetClockwise(motorSpeed<0);
		  motor0.SetPower(abs(motorSpeed));
		  motor1.SetPower(abs(motorSpeed));
//		  motor2.SetPower(abs(motorSpeed));
	  }
}

}
