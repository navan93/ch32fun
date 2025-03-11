#include "ch32fun.h"
#include "usbdevice.h"
#include <stdio.h>
#include <string.h>
#include "usbd_priv.h"


uint32_t USBDEBUG0, USBDEBUG1, USBDEBUG2;

struct _USBState USBDCTX;

static inline void DMA7FastCopy( uint8_t * dest, const uint8_t * src, int len )
{
	while( DMA1_Channel7->CNTR );
	DMA1_Channel7->CFGR = 0;
	DMA1_Channel7->MADDR = (uintptr_t)src;
	DMA1_Channel7->PADDR = (uintptr_t)dest;
	DMA1_Channel7->CNTR  = (len+3)/4;
	DMA1_Channel7->CFGR  =
		DMA_M2M_Enable |
		DMA_DIR_PeripheralDST |
		DMA_Priority_Low |
		DMA_MemoryDataSize_Word |
		DMA_PeripheralDataSize_Word |
		DMA_MemoryInc_Enable |
		DMA_PeripheralInc_Enable |
		DMA_Mode_Normal | DMA_CFGR1_EN;
#if !( FUSB_CURSED_TURBO_DMA == 1 )
	// Somehow, it seems to work (unsafely) without this.
	// Really, though, it's probably fine.
	while( DMA1_Channel7->CNTR );
#endif
}

static inline void DMA7FastCopyComplete() { while( DMA1_Channel7->CNTR ); }

#if FUSB_USE_HPE
// There is an issue with some registers apparently getting lost with HPE, just do it the slow way.
void USB_LP_CAN1_RX0_IRQHandler() __attribute__((section(".text.vector_handler")))  __attribute((naked));
#else
void USB_LP_CAN1_RX0_IRQHandler() __attribute__((section(".text.vector_handler")))  __attribute((interrupt));
#endif

//--------------------------------------------------------------------+
// PMA read/write
//--------------------------------------------------------------------+
static inline fsdev_bus_t fsdevbus_unaligned_read(const uint8_t *src)
{
	return ((uint16_t)src[0]) | ((uint16_t)src[1] << 8);
}

static inline void fsdevbus_unaligned_write(uint8_t *dst, fsdev_bus_t value)
{
  dst[0] = (uint8_t)(value & 0xFF);
  dst[1] = (uint8_t)((value >> 8) & 0xFF);
}
// Write to packet memory area (PMA) from user memory
// - Packet memory must be either strictly 16-bit or 32-bit depending on FSDEV_BUS_32BIT
// - Uses unaligned for RAM (since M0 cannot access unaligned address)
static bool dcd_write_packet_memory(uint16_t dst, const void *__restrict src, uint16_t nbytes) {
	if (nbytes == 0) return true;
	uint32_t n_write = nbytes / 2;



	fsdev_pma_buf_t* pma_buf = PMA_BUF_AT(dst);
	const uint8_t *src8 = src;

	printf("Write PMA dest: %lx\n", pma_buf);

	while (n_write--) {
	  pma_buf->value = fsdevbus_unaligned_read(src8);
	  src8 += 2;
	  pma_buf++;
	}

	// odd bytes e.g 1 for 16-bit or 1-3 for 32-bit
	uint16_t odd = nbytes & 1;
	if (odd) {
	  fsdev_bus_t temp = 0;
	  for(uint16_t i = 0; i < odd; i++) {
		temp |= *src8++ << (i * 8);
	  }
	  pma_buf->value = temp;
	}

	return true;
  }

// Read from packet memory area (PMA) to user memory.
// - Packet memory must be either strictly 16-bit or 32-bit depending on FSDEV_BUS_32BIT
// - Uses unaligned for RAM (since M0 cannot access unaligned address)
  static bool dcd_read_packet_memory(void *__restrict dst, uint16_t src, uint16_t nbytes) {
	if (nbytes == 0) return true;
	uint32_t n_read = nbytes / 2;

	fsdev_pma_buf_t* pma_buf = PMA_BUF_AT(src);
	uint8_t *dst8 = (uint8_t *)dst;

	while (n_read--) {
	  fsdevbus_unaligned_write(dst8, (fsdev_bus_t ) pma_buf->value);
	  dst8 += 2;
	  pma_buf++;
	}

	// odd bytes e.g 1 for 16-bit or 1-3 for 32-bit
	uint16_t odd = nbytes & 1;
	if (odd) {
	  fsdev_bus_t temp = pma_buf->value;
	  while (odd--) {
		*dst8++ = (uint8_t) (temp & 0xfful);
		temp >>= 8;
	  }
	}

	return true;
  }

static void dcd_edpt_stall(uint8_t ep_addr)
{
	uint8_t const ep_num = tu_edpt_number(ep_addr);
	tusb_dir_t const dir = tu_edpt_dir(ep_addr);

	uint32_t ep_reg = ep_read(ep_num) | USBD_CTR_TX | USBD_CTR_RX; // reserve CTR bits
	ep_reg &= USBD_EPREG_MASK | EP_STAT_MASK(dir);
	ep_change_status(&ep_reg, dir, EP_STAT_STALL);

	ep_write(ep_num, ep_reg);
}

void dcd_edpt_clear_stall(uint8_t ep_addr)
{
	uint8_t const ep_num = tu_edpt_number(ep_addr);
	tusb_dir_t const dir = tu_edpt_dir(ep_addr);

	uint32_t ep_reg = ep_read(ep_num) | USBD_CTR_TX | USBD_CTR_RX; // reserve CTR bits
	ep_reg &= USBD_EPREG_MASK | EP_STAT_MASK(dir) | EP_DTOG_MASK(dir);

	if (!ep_is_iso(ep_reg)) {
	  ep_change_status(&ep_reg, dir, EP_STAT_NAK);
	}
	ep_change_dtog(&ep_reg, dir, 0); // Reset to DATA0
	ep_write(ep_num, ep_reg);
}

// Handle CTR interrupt for the TX/IN direction
static void handle_ctr_tx(uint32_t ep_id)
{
	struct _USBState * ctx = &USBDCTX;
	uint32_t ep_reg = ep_read(ep_id) | USBD_CTR_TX | USBD_CTR_RX;
	int len = 0;

	uint8_t const ep_num = ep_reg & USBD_EA;

	uint8_t * addr_ptr = (uint8_t*)btable_get_addr(ep_id, BTABLE_BUF_TX);

	if(ep_id) {
		if (ep_id < FUSB_CONFIG_EPS) {
			ctx->USBD_Endp_Busy[ ep_id ] = 0;
		}
	} else {
		/* end-point 0 data in interrupt */
		if( ctx->USBD_SetupReqLen == 0 )
		{
			// UEP_CTRL_RX(0) = USBOTG_UEP_R_RES_ACK | USBOTG_UEP_R_TOG;
			ep_change_status(&ep_reg, TUSB_DIR_IN, EP_STAT_VALID);
			ep_reg &= USBD_EPREG_MASK | EP_STAT_MASK(TUSB_DIR_IN); // only change TX Status, reserve other toggle bits
  			ep_write(ep_id, ep_reg);
		}

		if( ctx->pCtrlPayloadPtr )
		{
			// Shortcut mechanism, for descriptors or if the user wants it.
			len = ctx->USBD_SetupReqLen >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : ctx->USBD_SetupReqLen;
			dcd_write_packet_memory(addr_ptr, ctx->pCtrlPayloadPtr, len);
			// DMA7FastCopy( ctrl0buff, ctx->pCtrlPayloadPtr, len ); // FYI -> Would need to do this if using DMA
			ctx->USBD_SetupReqLen -= len;
			if( ctx->USBD_SetupReqLen > 0 )
				ctx->pCtrlPayloadPtr += len;
			else
				ctx->pCtrlPayloadPtr = 0;

			btable_set_count(ep_id, BTABLE_BUF_TX, len);
			ep_change_status(&ep_reg, TUSB_DIR_IN, EP_STAT_VALID);
			ep_reg &= USBD_EPREG_MASK | EP_STAT_MASK(TUSB_DIR_IN); // only change TX Status, reserve other toggle bits
  			ep_write(ep_id, ep_reg);
		}
		else if ( ( ctx->USBD_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
		{

#if FUSB_HID_USER_REPORTS
			len = ctx->USBD_SetupReqLen >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : ctx->USBD_SetupReqLen;
			if( len && USBDCTX.USBD_SetupReqCode == HID_GET_REPORT )
			{
				len = HandleHidUserReportDataIn( ctx, addr_ptr, len );
				btable_set_count(ep_id, BTABLE_BUF_TX, len);
				ep_change_status(&ep_reg, TUSB_DIR_IN, EP_STAT_VALID);
				ep_reg &= USBD_EPREG_MASK | EP_STAT_MASK(TUSB_DIR_IN); // only change TX Status, reserve other toggle bits
				ep_write(ep_id, ep_reg);
				ctx->USBD_SetupReqLen -= len;
				ctx->pCtrlPayloadPtr += len;
			}
#endif
		}
		else
		{
			switch( USBDCTX.USBD_SetupReqCode )
			{
				case USB_GET_DESCRIPTOR:
					break;

				case USB_SET_ADDRESS:
					USBD->DADDR = ( USBD->DADDR & USBD_EF ) | ctx->USBD_DevAddr;
					break;

				default:
					break;
			}
		}
	}
}

static void handle_ctr_setup(uint32_t ep_id)
{
	uint16_t rx_count = btable_get_count(ep_id, BTABLE_BUF_RX);
	uint16_t rx_addr = btable_get_addr(ep_id, BTABLE_BUF_RX);
	uint8_t setup_packet[8] TU_ATTR_ALIGNED(4);
	int len = 0;
	struct _USBState * ctx = &USBDCTX;
	uint32_t ep_reg = ep_read(ep_id) | USBD_CTR_TX | USBD_CTR_RX;
	uint8_t tmp_buf[2] = {0, 0};

	dcd_read_packet_memory(ctx->ENDPOINTS[0], rx_addr, rx_count);

	// Clear CTR RX, if another setup packet arrived before this, it will be discarded
	ep_write_clear_ctr(ep_id, TUSB_DIR_OUT);


	// Setup packet should always be 8 bytes. If not, we probably missed the packet
	if (rx_count == 8) {
		// Hardware should reset EP0 RX/TX to NAK and both toggle to 1
		/* Store All Setup Values */
		int USBD_SetupReqType = USBDCTX.USBD_SetupReqType  = pUSBD_SetupReqPak->bmRequestType;
		int USBD_SetupReqCode = USBDCTX.USBD_SetupReqCode  = pUSBD_SetupReqPak->bRequest;
		int USBD_SetupReqLen = USBDCTX.USBD_SetupReqLen    = pUSBD_SetupReqPak->wLength;
		int USBD_SetupReqIndex = pUSBD_SetupReqPak->wIndex;
		int USBD_IndexValue = USBDCTX.USBD_IndexValue = ( pUSBD_SetupReqPak->wIndex << 16 ) | pUSBD_SetupReqPak->wValue;
		printf( "SETUP: %02x %02x %02d %02x %04x\n", USBD_SetupReqType, USBD_SetupReqCode, USBD_SetupReqLen, USBD_SetupReqIndex, USBD_IndexValue );
		len = 0;
		uint8_t * addr_ptr = (uint8_t*)btable_get_addr(0, BTABLE_BUF_TX);

		if( ( USBD_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD ) {
#if FUSB_HID_INTERFACES > 0
			if( ( USBD_SetupReqType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS ) {
				/* Class Request */
				switch( USBD_SetupReqCode ) {
					case HID_SET_REPORT:
#if FUSB_HID_USER_REPORTS
						len = HandleHidUserSetReportSetup( ctx, pUSBD_SetupReqPak );
						if( len < 0 ) goto sendstall;
						// Prepare endpoint 0 to receive data
						ctx->USBD_SetupReqLen = len;
						btable_set_rx_bufsize(0, BTABLE_BUF_RX, len);
						ep_reg &= USBD_EPREG_MASK | EP_STAT_MASK(TUSB_DIR_OUT);
						ep_change_status(&ep_reg, TUSB_DIR_OUT, EP_STAT_VALID);
    					ep_write(0, ep_reg);
						// Previously would have been a CTRL_RX = ACK && TOG, but not here on the 203.
						// UEP_CTRL_RX(0) = CHECK_USBOTG_UEP_R_AUTO_TOG | USBOTG_UEP_R_RES_ACK | USBOTG_UEP_R_TOG;
						// UEP_CTRL_TX(0) = CHECK_USBOTG_UEP_T_AUTO_TOG | USBOTG_UEP_T_TOG;
						goto replycomplete;

					case HID_GET_REPORT:
						len = HandleHidUserGetReportSetup( ctx, pUSBD_SetupReqPak );
						if( len < 0 ) goto sendstall;
						ctx->USBD_SetupReqLen = len;
						len = len >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : len;
						if( !ctx->pCtrlPayloadPtr ) {
							len = HandleHidUserReportDataIn( ctx, addr_ptr, len );
						} else {
							// DMA7FastCopy( ctrl0buff, ctx->pCtrlPayloadPtr, len );
							dcd_write_packet_memory(addr_ptr, ctx->pCtrlPayloadPtr, len);
							ctx->pCtrlPayloadPtr += len;
						}
						btable_set_count(0, BTABLE_BUF_TX, len);
						ep_change_status(&ep_reg, TUSB_DIR_IN, EP_STAT_VALID);
						ep_reg &= USBD_EPREG_MASK | EP_STAT_MASK(TUSB_DIR_IN); // only change TX Status, reserve other toggle bits
						ep_write(ep_id, ep_reg);
						// UEP_CTRL_TX(0) = CHECK_USBOTG_UEP_T_AUTO_TOG | USBOTG_UEP_T_RES_ACK | USBOTG_UEP_T_TOG;
						ctx->USBD_SetupReqLen -= len;
						goto replycomplete;
#endif
						break;

					case HID_SET_IDLE:
						if( USBD_SetupReqIndex < FUSB_HID_INTERFACES )
							USBDCTX.USBD_HidIdle[ USBD_SetupReqIndex ] = (uint8_t)( USBD_IndexValue >> 8 );
						break;

					case HID_SET_PROTOCOL:
						if ( USBD_SetupReqIndex < FUSB_HID_INTERFACES )
							USBDCTX.USBD_HidProtocol[USBD_SetupReqIndex] = (uint8_t)USBD_IndexValue;
						break;

					case HID_GET_IDLE:
						if( USBD_SetupReqIndex < FUSB_HID_INTERFACES ) {
							// ctrl0buff[0] = USBDCTX.USBD_HidIdle[ USBD_SetupReqIndex ];
							tmp_buf[0] = USBDCTX.USBD_HidIdle[ USBD_SetupReqIndex ];
							dcd_write_packet_memory(addr_ptr, tmp_buf, 1);
							len = 1;
						}
						break;

					case HID_GET_PROTOCOL:
						if( USBD_SetupReqIndex < FUSB_HID_INTERFACES ) {
							// ctrl0buff[0] = USBDCTX.USBD_HidProtocol[ USBD_SetupReqIndex ];
							tmp_buf[0] = USBDCTX.USBD_HidProtocol[ USBD_SetupReqIndex ];
							dcd_write_packet_memory(addr_ptr, tmp_buf, 1);
							len = 1;
						}
						break;

					default:
						goto sendstall;
						break;
				}
			}
#else
			;
#endif
		} else {
			/* usb standard request processing */
			switch( USBD_SetupReqCode )	{
				/* get device/configuration/string/report/... descriptors */
				case USB_GET_DESCRIPTOR:
				{
					const struct descriptor_list_struct * e = descriptor_list;
					const struct descriptor_list_struct * e_end = e + DESCRIPTOR_LIST_ENTRIES;
					for( ; e != e_end; e++ ) {
						if( e->lIndexValue == USBD_IndexValue ) {
							ctx->pCtrlPayloadPtr = (uint8_t*)e->addr;
							len = e->length;
							break;
						}
					}
					if( e == e_end ) {
						goto sendstall;
					}

					/* Copy Descriptors to Endp0 DMA buffer */
					int totalLen = USBD_SetupReqLen;
					if( totalLen > len ) {
						totalLen = len;
					}
					len = ( totalLen >= DEF_USBD_UEP0_SIZE ) ? DEF_USBD_UEP0_SIZE : totalLen;
					// DMA7FastCopy( ctrl0buff, ctx->pCtrlPayloadPtr, len ); //memcpy( CTRL0BUFF, ctx->pCtrlPayloadPtr, len );
					printf("Writing %d B to PA\n", len);
					dcd_write_packet_memory(addr_ptr, ctx->pCtrlPayloadPtr, len);
					ctx->USBD_SetupReqLen = totalLen - len;
					ctx->pCtrlPayloadPtr += len;
					btable_set_count(0, BTABLE_BUF_TX, len);
					// UEP_CTRL_TX(0) = CHECK_USBOTG_UEP_T_AUTO_TOG | USBOTG_UEP_T_RES_ACK | USBOTG_UEP_T_TOG;
					ep_change_status(&ep_reg, TUSB_DIR_IN, EP_STAT_VALID);
					ep_reg &= USBD_EPREG_MASK | EP_STAT_MASK(TUSB_DIR_IN); // only change TX Status, reserve other toggle bits
					ep_write(ep_id, ep_reg);
					goto replycomplete;
				}

				/* Set usb address */
				case USB_SET_ADDRESS:
					ctx->USBD_DevAddr = (uint8_t)( ctx->USBD_IndexValue & 0xFF );
					// NOTE: Do not actually set addres here!  If we do, we won't get the PID_IN associated with this SETUP.
					break;

				/* Get usb configuration now set */
				case USB_GET_CONFIGURATION:
					// ctrl0buff[0] = ctx->USBD_DevConfig;
					dcd_write_packet_memory(addr_ptr, &(ctx->USBD_DevConfig), 1);
					if( ctx->USBD_SetupReqLen > 1 )
						ctx->USBD_SetupReqLen = 1;
					break;

				/* Set usb configuration to use */
				case USB_SET_CONFIGURATION:
					ctx->USBD_DevConfig = (uint8_t)( ctx->USBD_IndexValue & 0xFF );
					ctx->USBD_DevEnumStatus = 0x01;
					break;

				/* Clear or disable one usb feature */
				case USB_CLEAR_FEATURE:
#if FUSB_SUPPORTS_SLEEP
					if( ( USBOTG_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
					{
						/* clear one device feature */
						if( (uint8_t)( USBOTG_IndexValue & 0xFF ) == USB_REQ_FEAT_REMOTE_WAKEUP )
						{
							/* clear usb sleep status, device not prepare to sleep */
							ctx->USBOTG_DevSleepStatus &= ~0x01;
						}
						else
						{
							goto sendstall;
						}
					}
					else
#endif
					if( ( USBD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP ) {
						if( (uint8_t)( USBD_IndexValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT ) {
							/* Clear End-point Feature */
							if( ( USBD_SetupReqIndex & DEF_UEP_IN ) && ep_id < FUSB_CONFIG_EPS ) {
								// UEP_CTRL_TX(ep) = USBOTG_UEP_T_RES_STALL | CHECK_USBOTG_UEP_T_AUTO_TOG;
								dcd_edpt_clear_stall(USBD_SetupReqIndex);
							} else {
								goto sendstall;
							}
						} else {
							goto sendstall;
						}
					} else {
						goto sendstall;
					}
					break;

				/* set or enable one usb feature */
				case USB_SET_FEATURE:
					if( ( USBD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE ) {
#if FUSB_SUPPORTS_SLEEP
						/* Set Device Feature */
						if( (uint8_t)( USBD_IndexValue & 0xFF ) == USB_REQ_FEAT_REMOTE_WAKEUP )
						{
							/* Set Wake-up flag, device prepare to sleep */
							USBD_DevSleepStatus |= 0x01;
						}
						else
#endif
						{
							goto sendstall;
						}
					} else if( ( USBD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP ) {
						/* Set Endpoint Feature */
						if( (uint8_t)( USBD_IndexValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT ) {
							// ep_id = USBD_SetupReqIndex & 0xf;
							if( ( USBD_SetupReqIndex & DEF_UEP_IN ) && ep_id < FUSB_CONFIG_EPS ) {
								// UEP_CTRL_TX(ep) = ( UEP_CTRL_TX(ep) & ~USBOTG_UEP_T_RES_MASK ) | USBOTG_UEP_T_RES_STALL;
								dcd_edpt_stall(USBD_SetupReqIndex);
							}
						}
						else
							goto sendstall;
					}
					else
						goto sendstall;
					break;

				/* This request allows the host to select another setting for the specified interface  */
				case USB_GET_INTERFACE:
					// ctrl0buff[0] = 0x00;
					tmp_buf[0] = 0;
					dcd_write_packet_memory(addr_ptr, tmp_buf, 1);
					if( USBD_SetupReqLen > 1 ) USBD_SetupReqLen = 1;
					break;

				case USB_SET_INTERFACE:
					break;

				/* host get status of specified device/interface/end-points */
				case USB_GET_STATUS:
					// ctrl0buff[0] = 0x00;
					// ctrl0buff[1] = 0x00;
					tmp_buf[0] = 0;
					tmp_buf[1] = 0;
					dcd_write_packet_memory(addr_ptr, tmp_buf, 2);
					if( ( USBD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE ) {
#if FUSB_SUPPORTS_SLEEP
						ctrl0buff[0] = (ctx->USBOTG_DevSleepStatus & 0x01)<<1;
#else
						// ctrl0buff[0] = 0x00;
						tmp_buf[0] = 0;
						dcd_write_packet_memory(addr_ptr, tmp_buf, 1);
#endif
					} else if( ( USBD_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP ) {
						if( ( USBD_SetupReqIndex & DEF_UEP_IN ) && ep_id < FUSB_CONFIG_EPS ) {
							//Check if EP is stalled
							ep_reg &= USBD_EPREG_MASK | EP_STAT_MASK(TUSB_DIR_IN);
							tmp_buf[0] = ( ep_reg & USBD_STAT_TX) == (EP_STAT_STALL << 4);
							dcd_write_packet_memory(addr_ptr, tmp_buf, 1);
						}
						else
							goto sendstall;
					} else
						goto sendstall;

					if( USBD_SetupReqLen > 2 )
						USBD_SetupReqLen = 2;

					break;

				default:
					goto sendstall;
					break;
			}
		}
		/* end-point 0 data Tx/Rx */
		if( USBD_SetupReqType & DEF_UEP_IN ) {
			len = ( USBD_SetupReqLen > DEF_USBD_UEP0_SIZE )? DEF_USBD_UEP0_SIZE : USBD_SetupReqLen;
			USBD_SetupReqLen -= len;
			btable_set_count(0, BTABLE_BUF_TX, len);
			// UEP_CTRL_TX(0) = CHECK_USBOTG_UEP_T_AUTO_TOG | USBOTG_UEP_T_RES_ACK;
			ep_change_status(&ep_reg, TUSB_DIR_IN, EP_STAT_VALID);
			ep_reg &= USBD_EPREG_MASK | EP_STAT_MASK(TUSB_DIR_IN); // only change TX Status, reserve other toggle bits
			ep_write(ep_id, ep_reg);
		} else {
			if( USBD_SetupReqLen == 0 )	{
				btable_set_count(0, BTABLE_BUF_TX, 0);
				// UEP_CTRL_TX(0) = CHECK_USBOTG_UEP_T_AUTO_TOG | USBOTG_UEP_T_RES_ACK | USBOTG_UEP_T_TOG;
				ep_change_status(&ep_reg, TUSB_DIR_IN, EP_STAT_VALID);
				ep_reg &= USBD_EPREG_MASK | EP_STAT_MASK(TUSB_DIR_IN); // only change TX Status, reserve other toggle bits
				ep_write(ep_id, ep_reg);
			} else {
				ep_change_status(&ep_reg, TUSB_DIR_OUT, EP_STAT_VALID);
				ep_reg &= USBD_EPREG_MASK | EP_STAT_MASK(TUSB_DIR_OUT); // only change TX Status, reserve other toggle bits
				ep_write(ep_id, ep_reg);
				// UEP_CTRL_RX(0) = CHECK_USBOTG_UEP_R_AUTO_TOG | USBOTG_UEP_R_RES_ACK | USBOTG_UEP_R_TOG;
			}
		}


		// This might look a little weird, for error handling but it saves a nontrivial amount of storage, and simplifies
		// control flow to hard-abort here.
		sendstall:
			// if one request not support, return stall.  Stall means permanent error.
			dcd_edpt_stall(0);
		replycomplete:
	} else {
		// Missed setup packet !!!
		printf("USB: Missed setup packet !!!\n");
		edpt0_prepare_setup();
	}
}

// Handle CTR interrupt for the RX/OUT direction
// Note: This looks to be not able to rx data more than 64bytes
static void handle_ctr_rx(uint32_t ep_id)
{
	uint32_t ep_reg = ep_read(ep_id) | USBD_CTR_TX | USBD_CTR_RX;
	struct _USBState * ctx = &USBDCTX;

	uint16_t pma_addr = (uint16_t) btable_get_addr(ep_id, BTABLE_BUF_RX);

	if( ep_id == 0) {
		/* end-point 0 data out interrupt */
#if FUSB_HID_USER_REPORTS
		int len = btable_get_count(ep_id, BTABLE_BUF_RX);
		uint16_t const rx_count = btable_get_count(ep_id, BTABLE_BUF_RX);
		uint8_t * cptr = ctx->pCtrlPayloadPtr;

		if( !cptr ) {
			HandleHidUserReportDataOut( ctx, pma_addr, len );
		} else {
			int remain = ctx->USBD_SetupReqLen - len;
			if( remain < 0 ) {
				len += remain;
				remain = 0;
			}
			// DMA7FastCopy( cptr, ctrl0buff, len );
			dcd_read_packet_memory(cptr, pma_addr, len);
			ctx->USBD_SetupReqLen = remain;
			if( remain > 0 )
				ctx->pCtrlPayloadPtr = cptr + len;
			else
				ctx->pCtrlPayloadPtr = 0;
		}
#endif
		if( ctx->USBD_SetupReqLen == 0 ){
#if FUSB_HID_USER_REPORTS
			// DMA7FastCopyComplete();
			HandleHidUserReportOutComplete( ctx );
#endif
			btable_set_count(0, BTABLE_BUF_TX, 0);
			ep_reg &= USBD_EPREG_MASK | EP_STAT_MASK(TUSB_DIR_IN); // will change RX Status, reserved other toggle bits
			ep_change_status(&ep_reg, TUSB_DIR_IN, EP_STAT_VALID);
			ep_write(ep_id, ep_reg);
			// UEP_CTRL_TX(0) = USBOTG_UEP_T_TOG | CHECK_USBOTG_UEP_T_AUTO_TOG | USBOTG_UEP_T_RES_ACK;
		} else{
			ep_change_dtog(&ep_reg, BTABLE_BUF_RX, 1); // Reset to DATA0
			// UEP_CTRL_RX(0) ^= USBOTG_UEP_R_TOG;
		}
	}

	// if ((rx_count < xfer->max_packet_size) || (xfer->queued_len >= xfer->total_len)) {
	//   // all bytes received or short packet

	//   // For ch32v203: reset rx bufsize to mps to prevent race condition to cause PMAOVR (occurs with msc write10)
	//   btable_set_rx_bufsize(ep_id, BTABLE_BUF_RX, xfer->max_packet_size);

	//   // ch32 seems to unconditionally accept ZLP on EP0 OUT, which can incorrectly use queued_len of previous
	//   // transfer. So reset total_len and queued_len to 0.
	//   xfer->total_len = xfer->queued_len = 0;
	// } else {
	//   // Set endpoint active again for receiving more data. Note that isochronous endpoints stay active always
	// 	uint16_t const cnt = tu_min16(xfer->total_len - xfer->queued_len, xfer->max_packet_size);

	// 	ep_reg &= USBD_EPREG_MASK | EP_STAT_MASK(TUSB_DIR_OUT); // will change RX Status, reserved other toggle bits
	// 	ep_change_status(&ep_reg, TUSB_DIR_OUT, EP_STAT_VALID);
	// 	ep_write(ep_id, ep_reg);
	// }
}

static void edpt0_open()
{
	// dcd_ep_alloc(0x0, TUSB_XFER_CONTROL);
	// dcd_ep_alloc(0x80, TUSB_XFER_CONTROL);

	printf("USB: ep_buf_ptr = %ld\n", ep_buf_ptr);

	uint16_t pma_addr0 = dcd_pma_alloc(CFG_TUD_ENDPOINT0_SIZE);
	uint16_t pma_addr1 = dcd_pma_alloc(CFG_TUD_ENDPOINT0_SIZE);

	btable_set_addr(0, BTABLE_BUF_RX, pma_addr0);
	btable_set_addr(0, BTABLE_BUF_TX, pma_addr1);

	printf("Alloc EP0 RX buf: %d\n", pma_addr0);
	printf("Alloc EP0 TX buf: %d\n", pma_addr1);

	uint32_t ep_reg = ep_read(0) & ~USBD_EPREG_MASK; // only get toggle bits
	ep_reg |= USB_EP_CONTROL;
	ep_change_status(&ep_reg, TUSB_DIR_IN, EP_STAT_NAK);
	ep_change_status(&ep_reg, TUSB_DIR_OUT, EP_STAT_NAK);
	// no need to explicitly set DTOG bits since we aren't masked DTOG bit

	edpt0_prepare_setup(); // prepare for setup packet
	ep_write(0, ep_reg);
}

static void edpts_open()
{
	// Set all EPs at Interrupt type IN endpoints
	uint32_t ep_reg;
	for (int i = 1; i < FUSB_CONFIG_EPS; i++) {
		ep_reg = ep_read(i) & ~USBD_EPREG_MASK;
  		ep_reg |= i | USBD_CTR_TX | USBD_CTR_RX;
		ep_reg |= USB_EP_INTERRUPT;
		/* Create a packet memory buffer area. */
		uint16_t pma_addr = dcd_pma_alloc(64);
		btable_set_addr(i, BTABLE_BUF_TX, pma_addr);
		ep_change_status(&ep_reg, TUSB_DIR_IN, EP_STAT_NAK);
 		ep_change_dtog(&ep_reg, TUSB_DIR_IN, 0);
		ep_reg &= ~(USBD_STAT_RX | USBD_DTOG_RX);
		ep_write(i, ep_reg);
	}
}

static void handle_bus_reset()
{
	USBD->DADDR = 0u; // disable USB Function

	printf("USB: Bus reset\n");

	// Reset PMA allocation
	ep_buf_ptr = FSDEV_BTABLE_BASE + 8 * FSDEV_EP_COUNT;	// 8bytes per entry

	edpt0_open(); // open control endpoint (both IN & OUT)
	edpts_open();

	USBD->DADDR = USBD_EF; // Enable USB Function
}

void USB_LP_CAN1_RX0_IRQHandler()
{
	uint32_t int_status = USBD->ISTR;
	struct _USBState * ctx = &USBDCTX;

	/* Put SOF flag at the beginning of ISR in case to get least amount of jitter if it is used for timing purposes */
	if (int_status & USBD_SOF) {
		USBD->ISTR = (fsdev_bus_t)~USBD_SOF;
	}

	if (int_status & USBD_RESET) {
	  // USBRST is start of reset.
	  ctx->USBD_DevConfig = 0;
	  ctx->USBD_DevAddr = 0;
	  ctx->USBD_DevSleepStatus = 0;
	  ctx->USBD_DevEnumStatus = 0;

	  USBD->ISTR = (fsdev_bus_t)~USBD_RESET;
	  handle_bus_reset();
	  return; // Don't do the rest of the things here; perhaps they've been cleared?
	}

	if (int_status & USBD_WKUP) {
	  USBD->CNTR &= ~USBD_LPMODE;
	  USBD->CNTR &= ~USBD_FSUP;
	  USBD->ISTR = (fsdev_bus_t)~USBD_WKUP;
	  ctx->USBD_DevSleepStatus &= ~0x02;
	}

	if (int_status & USBD_SUSP) {
		/* Suspend is asserted for both suspend and unplug events. without Vbus monitoring,
		* these events cannot be differentiated, so we only trigger suspend. */

		/* Force low-power mode in the macrocell */
		USBD->CNTR |= USBD_FSUP;
		USBD->CNTR |= USBD_LPMODE;

		/* clear of the ISTR bit must be done after setting of CNTR_FSUSP */
		USBD->ISTR = (fsdev_bus_t)~USBD_SUSP;

	  	ctx->USBD_DevSleepStatus |= 0x02;
		if( ctx->USBD_DevSleepStatus == 0x03 )
		{
			/* Handling usb sleep here */
			//TODO: MCU_Sleep_Wakeup_Operate( );
		}
	}

	// if (int_status & USBD_ESOF) {
	//   if (remoteWakeCountdown == 1u) {
	// 	USBD->CNTR &= ~USBD_RESUME;
	//   }
	//   if (remoteWakeCountdown > 0u) {
	// 	remoteWakeCountdown--;
	//   }
	//   USBD->ISTR = (fsdev_bus_t)~USBD_ESOF;
	// }

	// loop to handle all pending CTR (correct transfer status) interrupts
	while (USBD->ISTR & USBD_CTR) {
	  // skip DIR bit, and use CTR TX/RX instead, since there is chance we have both TX/RX completed in one interrupt
	  uint32_t const ep_id = USBD->ISTR & USBD_EP_ID;
	  uint32_t const ep_reg = ep_read(ep_id);

	  if (ep_reg & USBD_CTR_RX) {
		if (ep_reg & USBD_SETUP) {	// SETUP PID - Host -> Device setup transfer
		  handle_ctr_setup(ep_id); 	// CTR will be clear after copied setup packet
		} else {					// OUT PID - Host -> Device Send data to device
		  ep_write_clear_ctr(ep_id, TUSB_DIR_OUT);
		  handle_ctr_rx(ep_id);
		}
	  }

	  if (ep_reg & USBD_CTR_TX) { // IN PID - Device -> Host Request data from device
		ep_write_clear_ctr(ep_id, TUSB_DIR_IN);
		handle_ctr_tx(ep_id);
	  }
	}

	if (int_status & USBD_PMAOVR) {
	  printf("USB: Packet memory Overflow!!\n");
	  USBD->ISTR = (fsdev_bus_t)~USBD_PMAOVR;
	}

}

int USBDSetup(void)
{
	printf("Setup USB Device\n\n");
    // USBPRE[1:0] = 10: Divided by 3 (when PLLCLK=144MHz);
	// Must be done before enabling clock to USB OTG tree.
    RCC->CFGR0 = (RCC->CFGR0 & ~(3<<22)) | (2<<22);

    /** Enable USB Clock **/
    RCC->APB2PCENR |= RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA;   // Enable Port A clock
    RCC->APB1PCENR |= RCC_USBEN;  // Enable USBD clock
	RCC->AHBPCENR |= RCC_AHBPeriph_DMA1; // USBFS === OTG_FSEN

    /** Configure GPIO for USB (PA11 = D-, PA12 = D+) **/
    // GPIOA->CFGHR &= ~(GPIO_CFGHR_MODE11 | GPIO_CFGHR_MODE12); // Clear mode bits
    // GPIOA->CFGHR |= (GPIO_CFGHR_CNF11_1 | GPIO_CFGHR_CNF12_1); // Set PA11, PA12 as USB

    /** Configure USB Registers **/
    USBD->CNTR = USBD_FRES | USBD_PDWN;  // Reset USB controller
	Delay_Us(200);
	USBD->CNTR &= ~USBD_PDWN;
	Delay_Us(200);
	USBD->CNTR = 0;  // Enable USB

	USBD->BTABLE = FSDEV_BTABLE_BASE;	// Set 0, which is start of dedicated packet memory

	USBD->ISTR = 0xFF;     // Clear all interrupts

	// Reset endpoints to disabled
	for (uint32_t i = 0; i < FSDEV_EP_COUNT; i++) {
	// This doesn't clear all bits since some bits are "toggle", but does set the type to DISABLED.
		USBD->EPR[i] = 0u;
	}

	USBD->CNTR |= USBD_RESETM | USBD_ESOFM | USBD_CTRM | USBD_SUSPM | USBD_WKUPM | USBD_PMAOVRM;
	handle_bus_reset();


    /** Enable USB Interrupts **/
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    //Chapter 33 of TRM
    EXTEN->EXTEN_CTR |= 0x02; // USBD internal pull-up resistor enable and Full-speed mode

    // Go on-bus.
	return 0;
}

static inline uint8_t * USBD_GetEPBufferIfAvailable( int endp )
{
	if( USBDCTX.USBD_Endp_Busy[ endp ] ) return 0;
	return USBDCTX.ENDPOINTS[ endp ];
}

static inline int USBD_SendEndpoint( int endp, int len )
{
	// if( USBDCTX.USBD_Endp_Busy[ endp ] ) return -1;
	// UEP_CTRL_LEN( endp ) = len;
	// UEP_CTRL_TX( endp ) = ( UEP_CTRL_TX( endp ) & ~USBOTG_UEP_T_RES_MASK ) | USBOTG_UEP_T_RES_ACK;
	// USBOTGCTX.USBOTG_Endp_Busy[ endp ] = 0x01;
	// return 0;
}

