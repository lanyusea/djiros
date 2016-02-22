#include "DJI_LED.h"
#include <string.h>

using namespace DJI::onboardSDK;

LED::LED(DJI::onboardSDK::CoreAPI *ControlAPI) { api = ControlAPI; }

CoreAPI *LED::getApi() const {return api;}

void LED::getLEDInfo(CallBack callback, UserData userData)
{
	api->send(2, encrypt, SET_LED, LED_GET_INFO, 0, 0,
			100, 3, callback ? callback : LED::getLEDInfoCallback, userData);
}

void LED::getLEDInfoCallback(CoreAPI *This, Header *header, UserData userData __UNUSED)
{
	LED::LEDInfo ledInfo;
	
	memcpy((unsigned char *)&ledInfo,
		((unsigned char *)header)+sizeof(Header),
		(header -> length - EXC_DATA_SIZE));
	printf("colors_num: %d\n", ledInfo.color_num);
	printf("max_action_num: %d\n", ledInfo.max_action_num);
	printf("surplus_action_num: %d\n", ledInfo.surplus_action_num);
	
	for(int i = 0; i < 12; i++)
	{
		printf("color choice %d: %d \n", i,ledInfo.colors[i]);

	}
}


void LED::registerLED(RegisterLED registerLED, CallBack callback, UserData userData)
{

	LED::RegisterLED registerLed;

	//if (registerLED == 0){
		registerLed.generate_flag = 0;
		registerLed.requester_id = 1;
		registerLed.identity_id = 128;
		registerLed.actions = { {1, 4, 3, 0, 0, 1, 6, 9},
								{5, 10, 20, 50, 0, 0, 0, 2} };
		registerLed.priority = 1;
		registerLed.show_times = 0;
	//}
	//else {
	//	registerLed = registerLED;
	//}

	api->send(2, encrypt, SET_LED, LED_REGISTER, (unsigned char*)&registerLed, 
			sizeof(registerLED), 100, 3, 
			callback ? callback:LED::registerLEDCallback, userData);
}

void LED::registerLEDCallback(CoreAPI *This, Header *header, UserData userData) 
{
	LED::RegisterLEDACK ack;
	memcpy((unsigned char *)&ack, 
			((unsigned char *)header) + sizeof(Header),
			(header -> length - EXC_DATA_SIZE));

	printf("result: %d \n", ack.result);
	printf("requester_id: %d \n", ack.requester_id);
	printf("identify_id: %d \n", ack.identity_id);
	printf("action_id: %d \n", ack.action_id);
	printf("surplus_action_num: %d \n", ack.surplus_action_num);
}

void LED::logoutLED (LogoutLED logoutLed, CallBack callback, UserData userData )
{
	LED::LogoutLED logoutLED;
	//if (logoutLed == 0) {

		logoutLED.action_num = 5;
		logoutLED.requester_id = 1;
		logoutLED.action_id = {0, 1, 2, 3, 4};

	//}
	//else {
	//	logoutLED = logoutLed;
	//}

	api->send(2, encrypt, SET_LED, LED_LOGOUT, (unsigned char*)&logoutLED, 
			sizeof(logoutLED), 100, 3, 
			callback ? callback:LED::logoutLEDCallback, userData);
}

void LED::logoutLEDCallback (CoreAPI *This, Header * header, UserData userData)
{
	LED::LogoutLEDACK ack;
	memcpy((unsigned char *)&ack, 
			((unsigned char *)header) + sizeof(Header),
			(header -> length - EXC_DATA_SIZE));
	printf("logout result %d \n", ack.result);
	printf("logout surplus_action_num%d \n", ack.surplus_action_num);
}

void LED::setLEDAction(SetLEDAction ledAction, CallBack callback, UserData userData)
{
	LED::SetLEDAction LEDAction;
	//if(ledActionStatus == 0) {
		LEDAction.action_num = 1;
		LEDAction.requester_id = 1;
		LEDAction.action_status[0].id = 0;
		LEDAction.action_status[0].status= 1;
	//}
	//else {
	//	LEDActionStatus = ledActionStatus;
	//}

	api->send(2, encrypt, SET_LED, LED_SET_STATUS, 
			(unsigned char*)&LEDAction, 
			sizeof(LEDAction), 100, 3, 
			callback ? callback:LED::setLEDActionCallback, userData);
}

void LED::setLEDActionCallback(CoreAPI *This, Header *header, UserData userData)
{
	unsigned char ack;
	memcpy((unsigned char *)&ack, 
			((unsigned char *)header) + sizeof(Header),
			(header -> length - EXC_DATA_SIZE));
	printf("result %d \n", ack);
}
