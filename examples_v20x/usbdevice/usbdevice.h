#ifndef _USBDEVICE_H
#define _USBDEVICE_H

#include <stdint.h>
#include "ch32fun.h"
#include "usb_defines.h"
#include "usb_config.h"

extern uint32_t USBDEBUG0, USBDEBUG1, USBDEBUG2;

struct _USBState;

// Provided functions:
int USBDSetup();
uint8_t USBD_Endp_DataUp(uint8_t endp, const uint8_t *pbuf, uint16_t len, uint8_t mod);
static inline uint8_t * USBD_GetEPBufferIfAvailable( int endp );
static inline int USBD_SendEndpoint( int endp, int len );

static inline void DMA7FastCopy( uint8_t * dest, const uint8_t * src, int len );
static inline void DMA7FastCopyComplete();


// Implement the following:
#if FUSB_HID_USER_REPORTS
int HandleHidUserGetReportSetup( struct _USBState * ctx, tusb_control_request_t * req );
int HandleHidUserSetReportSetup( struct _USBState * ctx, tusb_control_request_t * req );
void HandleHidUserReportDataOut( struct _USBState * ctx, uint8_t * data, int len );
int HandleHidUserReportDataIn( struct _USBState * ctx, uint8_t * data, int len );
void HandleHidUserReportOutComplete( struct _USBState * ctx );
#endif


struct _USBState
{
	// Setup Request
	uint8_t  USBD_SetupReqCode;
	uint8_t  USBD_SetupReqType;
	uint16_t USBD_SetupReqLen;   // Used for tracking place along send.
	uint32_t USBD_IndexValue;

	// USB Device Status
	uint8_t  USBD_DevConfig;
	uint8_t  USBD_DevAddr;
	uint8_t  USBD_DevSleepStatus;
	uint8_t  USBD_DevEnumStatus;

	uint8_t  *  pCtrlPayloadPtr;

	uint8_t ENDPOINTS[FUSB_CONFIG_EPS][64];

	#define CTRL0BUFF					(USBDCTX.ENDPOINTS[0])
	#define pUSBD_SetupReqPak			((tusb_control_request_t*)CTRL0BUFF)

#if FUSB_HID_INTERFACES > 0
	uint8_t USBD_HidIdle[FUSB_HID_INTERFACES];
	uint8_t USBD_HidProtocol[FUSB_HID_INTERFACES];
#endif
	volatile uint8_t USBD_Endp_Busy[FUSB_CONFIG_EPS];
};

extern struct _USBState USBDCTX;

#include "usbdevice.c"

#endif // _USBDEVICE_H