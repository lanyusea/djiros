#ifndef DJI_LED_H
#define DJI_LED_H

#include "DJI_API.h"

namespace DJI
{

namespace onboardSDK
{


class LED
{
public:
#pragma pack(1)
	typedef struct LEDInfo
	{
		uint8_t color_num;
		uint8_t max_action_num; 
		uint8_t surplus_action_num;
		uint8_t colors[11];
	} LEDInfo;

	typedef struct RegisterLED
	{
		uint8_t generate_flag;
		uint32_t requester_id;
		uint8_t identity_id;
		uint16_t actions[8][2];
		uint8_t priority;
		uint8_t show_times;
		char description[10];
	} RegisterLED;

	typedef struct RegisterLEDACK
	{
		uint8_t result;
		uint32_t requester_id;
		uint8_t identity_id;
		uint8_t action_id;
		uint8_t surplus_action_num; 
	} RegisterLEDACK;

	typedef struct LogoutLED
	{
		uint8_t action_num;
		uint32_t requester_id;
		uint8_t action_id[5];
	} LogoutLED;

	typedef struct LogoutLEDACK
	{
		uint8_t result;
		uint8_t surplus_action_num;
	} LogoutLEDACK;

	typedef struct LEDActionStatus
	{
		uint8_t id;
		uint8_t status;
	} LEDActionStatus;

	typedef struct SetLEDAction
	{
		uint8_t action_num;
		uint32_t requester_id;
		LEDActionStatus action_status[5];
	} SetLEDAction;

#pragma pack()

	enum LEDAction
	{
		LED_GET_INFO = 0x00,
		LED_REGISTER = 0x01,
		LED_LOGOUT = 0x02,
		LED_SET_STATUS = 0x03
	};

public:
	LED(CoreAPI *ControlAPI = 0);
	CoreAPI *getApi() const;

	void getLEDInfo (CallBack callback = 0, UserData userData = 0);
	void registerLED (RegisterLED registerLED, CallBack callback = 0, UserData userData = 0);
	void logoutLED (LogoutLED logoutLED, CallBack callback = 0, UserData userData = 0);
	void startLED(SetLEDAction LEDAction, CallBack callback = 0, UserData userData = 0);
	
	static void getLEDInfoCallback(CoreAPI *This, Header *header, UserData userData = 0);
	static void registerLEDCallback(CoreAPI *This, Header *header, UserData userData = 0);
	static void logoutLEDCallback(CoreAPI *This, Header *header, UserData userData = 0);
	static void startLEDCallback(CoreAPI *This, Header *header, UserData userData = 0);
	
private:
	CoreAPI *api;

};

}
}
#endif
