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

void LED::getLEDInfoCallback(CoreAPI *This, Header *header, UserData userData)
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

	api->send(2, encrypt, SET_LED, LED_REGISTER, (unsigned char*)&registerLED, 
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

void LED::logoutLED (LogoutLED logoutLED, CallBack callback, UserData userData )
{
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

void LED::startLED(SetLEDAction LEDAction, CallBack callback, UserData userData)
{

	api->send(2, encrypt, SET_LED, LED_SET_STATUS, 
			(unsigned char*)&LEDAction, 
			sizeof(LEDAction), 100, 3, 
			callback ? callback:LED::startLEDCallback, userData);
}

void LED::startLEDCallback(CoreAPI *This, Header *header, UserData userData)
{
	unsigned char ack;
	memcpy((unsigned char *)&ack, 
			((unsigned char *)header) + sizeof(Header),
			(header -> length - EXC_DATA_SIZE));
	printf("result %d \n", ack);
}
