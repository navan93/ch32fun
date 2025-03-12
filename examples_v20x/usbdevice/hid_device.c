/* Example usb HID device using the Full speed USB device only
   peripheral connected to pins PA11 and PA12.
*/
#include "ch32fun.h"
#include <stdio.h>
#include <string.h>
#include "usbdevice.h"

uint32_t USBDEBUG0, USBDEBUG1, USBDEBUG2;

uint8_t scratchpad[256];

int HandleHidUserSetReportSetup( struct _USBState * ctx, tusb_control_request_t * req )
{
	puts("HandleHidUserSetReportSetup\n");
	int id = req->wValue & 0xff;
	if( id == 0xaa && req->wLength <= sizeof(scratchpad) )
	{
		memset( scratchpad, 0x55, sizeof(scratchpad) );
		printf( "SET REPORT! %d [%02x]\n", req->wLength, scratchpad[200] );
		ctx->pCtrlPayloadPtr = scratchpad;
		return req->wLength;
	}
	return 0;
}

int HandleHidUserGetReportSetup( struct _USBState * ctx, tusb_control_request_t * req )
{
	puts("HandleHidUserGetReportSetup\n");
	int id = req->wValue & 0xff;
	if( id == 0xaa )
	{
		printf( "GET REPORT! %d\n", req->wLength );
		ctx->pCtrlPayloadPtr = scratchpad;
		return sizeof(scratchpad) - 1;
	}
	return 0;
}

// Data sent to host
int HandleHidUserReportDataIn( struct _USBState * ctx, uint8_t * data, int len )
{
	printf( "IN %d %d %08x %08x\n", len, ctx->USBD_SetupReqLen, data, USBDCTX.ENDPOINTS[0] );
	memset( data, 0xcc, len );
	puts("HandleHidUserReportDataIn\n");
	return len;
}

// Data received from host
void HandleHidUserReportDataOut( struct _USBState * ctx, uint8_t * data, int len )
{
	puts("HandleHidUserReportDataOut\n");
	return;
}

void HandleHidUserReportOutComplete( struct _USBState * ctx )
{
	puts("HandleHidUserReportOutComplete\n");
	return;
}

int main()
{
	SystemInit();

	funGpioInitAll();

	funPinMode( PA0, GPIO_CFGLR_OUT_10Mhz_PP );

	USBDSetup();

	int clicks = 0;

	while(1)
	{
		//printf( "%lu %08lx %lu %d %d\n", USBDEBUG0, USBDEBUG1, USBDEBUG2, 0, 0 );
		int i;
		for( i = 1; i < 3; i++ )
		{

			uint32_t * buffer = (uint32_t*)USBD_GetEPBufferIfAvailable( i );
			if( buffer )
			{
				int tickDown =  ((SysTick->CNT)&0x10000);
				static int wasTickMouse, wasTickKeyboard;
				if( i == 1 )
				{
					// Keyboard
					buffer[0] = 0x00000000;
					if(tickDown && !wasTickKeyboard && clicks < 50 )
					{
						buffer[0] = 0x00250000;
						clicks++;
					}
					buffer[1] = 0x00000000;
					wasTickKeyboard = tickDown;
				}
				else
				{
					buffer[0] = (tickDown && !wasTickMouse)?0x0010100:0x00000000;
					wasTickMouse = tickDown;
				}
				USBD_SendEndpoint( i, (i==1)?8:4 );
			}
		}
	}
}