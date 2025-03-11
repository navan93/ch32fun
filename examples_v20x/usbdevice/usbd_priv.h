#ifndef USBD_PRIV_H
#define USBD_PRIV_H

#include "stdint.h"
#include "ch32fun.h"
#include "usb_defines.h"

#define TU_ATTR_ALIGNED(n)       __attribute__((aligned(n)))


// Mask for the combined USBFSD->INT_FG + USBFSD->INT_ST
#define CRB_UIF_ISO_ACT   (1<<6)
#define CRB_UIF_SETUP_ACT (1<<5)
#define CRB_UIF_FIFO_OV   (1<<4)
#define CRB_UIF_HST_SOF   (1<<3)
#define CRB_UIF_SUSPEND   (1<<2)
#define CRB_UIF_TRANSFER  (1<<1)
#define CRB_UIF_BUS_RST   (1<<0)
#define CRB_UIS_IS_NAK    (1<<15)
#define CRB_UIS_TOG_OK    (1<<14)
#define CMASK_UIS_TOKEN   (3<<12)
#define CMASK_UIS_ENDP    (0xf<<8)

#define CUIS_TOKEN_OUT	   0x0
#define CUIS_TOKEN_SOF     0x1
#define CUIS_TOKEN_IN      0x2
#define CUIS_TOKEN_SETUP   0x3

/* EndPoint REGister MASK (no toggle fields) */
#define USBD_EPREG_MASK    (USBD_CTR_RX|USBD_SETUP|USBD_EPTYPE|USBD_EPKIND|USBD_CTR_TX|USBD_EA)

#define USB_EP_TYPE_MASK                     ((uint16_t)0x0600U)               /*!< EndPoint TYPE Mask */
#define USB_EP_BULK                          ((uint16_t)0x0000U)               /*!< EndPoint BULK */
#define USB_EP_CONTROL                       ((uint16_t)0x0200U)               /*!< EndPoint CONTROL */
#define USB_EP_ISOCHRONOUS                   ((uint16_t)0x0400U)               /*!< EndPoint ISOCHRONOUS */
#define USB_EP_INTERRUPT                     ((uint16_t)0x0600U)               /*!< EndPoint INTERRUPT */
#define USB_EP_T_MASK                        ((uint16_t) ~USB_EP_T_FIELD & USB_EPREG_MASK)

#define FSDEV_PMA_SIZE (512u)
#define FSDEV_PMA_BASE CAN_USBD_SHARED_BASE
#define FSDEV_PMA_STRIDE  2
#define pma_access_scheme TU_ATTR_ALIGNED(4)
#define CFG_TUD_ENDPOINT0_SIZE 64

// If sharing with CAN, one can set this to be non-zero to give CAN space where it wants it
// Both of these MUST be a multiple of 2, and are in byte units.
#ifndef FSDEV_BTABLE_BASE
#define FSDEV_BTABLE_BASE 0U
#endif

typedef uint16_t fsdev_bus_t;
#define fsdevbus_unaligned_read(_addr)           tu_unaligned_read16(_addr)
#define fsdevbus_unaligned_write(_addr, _value)  tu_unaligned_write16(_addr, _value)

typedef struct {
	volatile pma_access_scheme fsdev_bus_t value;
  } fsdev_pma_buf_t;

#define PMA_BUF_AT(_addr) ((fsdev_pma_buf_t*) (FSDEV_PMA_BASE + FSDEV_PMA_STRIDE*(_addr)))



/* Note on shared packet memory
Total size of 512 bytes
The register USBD->BTABLE points to the location of buffer descriptor table inside packet memory.
Each entry in buffer descriptor table contains 4 16-bit words
The packet memory is accesses in 2 bytes aligned addresses by the USB periph.
*/
// PMA allocation/access
static uint16_t ep_buf_ptr; ///< Points to first free memory location

typedef enum {
	EP_STAT_DISABLED = 0,
	EP_STAT_STALL = 1,
	EP_STAT_NAK = 2,
	EP_STAT_VALID = 3
  }ep_stat_t;

#define EP_STAT_MASK(_dir)  (3u << (4 + ((_dir) == TUSB_DIR_IN ? 0 : 8)))
#define EP_DTOG_MASK(_dir)  (1u << (6 + ((_dir) == TUSB_DIR_IN ? 0 : 8)))

//--------------------------------------------------------------------+
// BTable Typedef
//--------------------------------------------------------------------+
enum {
	BTABLE_BUF_TX = 0,
	BTABLE_BUF_RX = 1
};

// hardware limit endpoint
#define FSDEV_EP_COUNT 8

// Buffer Table is located in Packet Memory Area (PMA) and therefore its address access is forced to either
// 16-bit or 32-bit depending on FSDEV_BUS_32BIT.
// 0: TX (IN), 1: RX (OUT)
typedef union {
	struct {
		volatile pma_access_scheme uint16_t addr;
		volatile pma_access_scheme uint16_t count;
	} ep16[FSDEV_EP_COUNT][2];
} fsdev_btable_t;


#define FSDEV_BTABLE ((volatile fsdev_btable_t*) (FSDEV_PMA_BASE + FSDEV_PMA_STRIDE*(FSDEV_BTABLE_BASE)))

#define CEIL_DIV(x, y)  (((x) + (y) - 1) / (y))

#define PMA_BLOCK_THRESHOLD   62
#define PMA_BLOCK_LARGE_SIZE  32
#define PMA_BLOCK_SMALL_SIZE  2

#define PMA_BLSIZE(size)  (((size) > PMA_BLOCK_THRESHOLD) ? 1 : 0)

#define PMA_NUM_BLOCK(size)  \
  (((size) > PMA_BLOCK_THRESHOLD) ? \
   (CEIL_DIV((size), PMA_BLOCK_LARGE_SIZE)) : \
   (CEIL_DIV((size), PMA_BLOCK_SMALL_SIZE)))

#define PMA_ALIGN_BUFFER_SIZE(size, blsize, num_block)  \
  *(blsize) = PMA_BLSIZE(size),                        \
  *(num_block) = PMA_NUM_BLOCK(size)

#define PMA_TOTAL_BYTES(size)  \
	(PMA_NUM_BLOCK(size) * ((size) > PMA_BLOCK_THRESHOLD ? PMA_BLOCK_LARGE_SIZE : PMA_BLOCK_SMALL_SIZE))


//--------------------------------------------------------------------+
// BTable Helper
//--------------------------------------------------------------------+

static inline uint32_t btable_get_addr(uint32_t ep_id, uint8_t buf_id)
{
	return FSDEV_BTABLE->ep16[ep_id][buf_id].addr;
}

static inline void btable_set_addr(uint32_t ep_id, uint8_t buf_id, uint16_t addr)
{
	FSDEV_BTABLE->ep16[ep_id][buf_id].addr = addr;
}

static inline uint16_t btable_get_count(uint32_t ep_id, uint8_t buf_id)
{
	uint16_t count;
	count = FSDEV_BTABLE->ep16[ep_id][buf_id].count;
	return count & 0x3FFU;
}

// Used for setting TX count
static inline void btable_set_count(uint32_t ep_id, uint8_t buf_id, uint16_t byte_count)
{
	uint16_t cnt = FSDEV_BTABLE->ep16[ep_id][buf_id].count;
	cnt = (cnt & ~0x3FFU) | (byte_count & 0x3FFU);
	FSDEV_BTABLE->ep16[ep_id][buf_id].count = cnt;
}

// Used for setting expected bytes in RX
static inline void btable_set_rx_bufsize(uint32_t ep_id, uint8_t buf_id, uint16_t wCount)
{
	uint8_t blsize, num_block;
	PMA_ALIGN_BUFFER_SIZE(wCount, &blsize, &num_block);

	/* Encode into register. When BLSIZE==1, we need to subtract 1 block count */
	uint16_t bl_nb = (blsize << 15) | ((num_block - blsize) << 10);
	if (bl_nb == 0) {
	  // zlp but 0 is invalid value, set blsize to 1 (32 bytes)
	  // Note: lower value can cause PMAOVR on setup with ch32v203
	  bl_nb = 1 << 15;
	}

	FSDEV_BTABLE->ep16[ep_id][buf_id].count = bl_nb;
}

/***
 * Allocate a section of PMA
 * In case of double buffering, high 16bit is the address of 2nd buffer
 * During failure, TU_ASSERT is used. If this happens, rework/reallocate memory manually.
 * pass only aligned 2 bytes aligned len
 */
static uint32_t dcd_pma_alloc(uint16_t len)
{
  uint32_t addr = ep_buf_ptr;
  uint16_t aligned_len = PMA_TOTAL_BYTES(len);


  ep_buf_ptr = (uint16_t)(ep_buf_ptr + aligned_len); // increment buffer pointer

  printf("PMA alloc- addr: %ld, align_len: %d, ep_buf_ptr: %ld\n", addr, aligned_len, ep_buf_ptr);
  // Verify packet buffer is not overflowed
//   TU_ASSERT(ep_buf_ptr <= FSDEV_PMA_SIZE, 0xFFFF);

  return addr;
}

//--------------------------------------------------------------------+
// Endpoint Helper
// - CTR is write 0 to clear
// - DTOG and STAT are write 1 to toggle
//--------------------------------------------------------------------+

static inline uint32_t ep_read(uint32_t ep_id)
{
	return USBD->EPR[ep_id];
}

static inline void ep_write(uint32_t ep_id, uint32_t value)
{
	USBD->EPR[ep_id] = value;
}

static inline void ep_change_status(uint32_t* reg, tusb_dir_t dir, ep_stat_t state)
{
	*reg ^= (state << (4u + (dir == TUSB_DIR_IN ? 0 : 8)));
}

static inline void ep_write_clear_ctr(uint32_t ep_id, tusb_dir_t dir) {
	uint32_t reg = ep_read(ep_id);
	reg |= USBD_CTR_TX | USBD_CTR_RX;
	reg &= USBD_EPREG_MASK;
	reg &= ~(1 << (7u + (dir == TUSB_DIR_IN ? 0 : 8)));
	ep_write(ep_id, reg);
}

static inline void edpt0_prepare_setup(void) {
	btable_set_rx_bufsize(0, BTABLE_BUF_RX, 8);
}

static inline bool ep_is_iso(uint32_t reg) {
    return (reg & USB_EP_TYPE_MASK) == USB_EP_ISOCHRONOUS;
}

static inline void ep_change_dtog(uint32_t* reg, tusb_dir_t dir, uint8_t state) {
    *reg ^= (state << (6u + (dir == TUSB_DIR_IN ? 0 : 8)));
}

#endif //USBD_PRIV_H