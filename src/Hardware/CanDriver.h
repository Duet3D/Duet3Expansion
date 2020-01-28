/**
 * \file
 *
 * \brief Control Area Network(CAN) functionality declaration.
 *
 * Copyright (c) 2016-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#ifndef SRC_CAN_CANDRIVER_H_
#define SRC_CAN_CANDRIVER_H_

//#include <utils.h>
#include <hpl_irq.h>

typedef void (*FUNC_PTR)(void);

/**
 * \brief CAN Message Format
 */
enum can_format {
	CAN_FMT_STDID, /*!< Standard Format, 11 bits identifier */
	CAN_FMT_EXTID  /*!< Extended Format, 29 bits identifier */
};

/**
 * \brief CAN Message Type
 */
enum can_type {
	CAN_TYPE_DATA,  /*!< A DATA FRAME carries data from a transmitter to the
	                    receivers.*/
	CAN_TYPE_REMOTE /*!< A REMOTE FRAME is transmitted by a bus unit to request
	                     the transmission of the DATA FRAME with the same
	                     IDENTIFIER */
};

/**
 * \brief CAN Bus Mode
 */
enum can_mode {
	/** Normal operation Mode */
	CAN_MODE_NORMAL,
	/** In Bus Monitoring mode (see ISO11898-1, 10.12 Bus monitoring), the CAN
	 * is able to receive valid data frames and valid remote frames, but
	 * cannot start a transmission. In this mode, it sends only recessive bits
	 * on the CAN bus. If the CAN is required to send a dominant bit (ACK bit,
	 * overload flag, active error flag), the bit is rerouted internally so
	 * that the CAN monitors this dominant bit, although the CAN bus may
	 * remain in recessive state.
	 * The Bus Monitoring mode can be used to analyze the traffic on a CAN bus
	 * without affecting it by the transmission of dominant bits.
	 */
	CAN_MODE_MONITORING
};

/**
 * \brief CAN Message
 */
struct can_message {
	uint32_t        id;   /* Message identifier */
	enum can_type   type; /* Message Type */
	uint8_t *       data; /* Pointer to Message Data */
	uint8_t         len;  /* Message Length */
	enum can_format fmt;  /* Identifier format, CAN_STD, CAN_EXT */
};

/**
 * \brief CAN Filter
 */
struct can_filter {
	uint32_t id;   /* Message identifier */
	uint32_t mask; /* The mask applied to the id */
};

/**@}*/
/**
 * \brief CAN receive FIFO element.
 */
struct _can_rx_fifo_entry {
	__IO union {
		struct {
			uint32_t ID : 29; /*!< Identifier */
			uint32_t RTR : 1; /*!< Remote Transmission Request */
			uint32_t XTD : 1; /*!< Extended Identifier */
			uint32_t ESI : 1; /*!< Error State Indicator */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} R0;
	__IO union {
		struct {
			uint32_t RXTS : 16; /*!< Rx Timestamp */
			uint32_t DLC : 4;   /*!< Data Length Code */
			uint32_t BRS : 1;   /*!< Bit Rate Switch */
			uint32_t FDF : 1;   /*!< FD Format */
			uint32_t : 2;       /*!< Reserved */
			uint32_t FIDX : 7;  /*!< Filter Index */
			uint32_t ANMF : 1;  /*!< Accepted Non-matching Frame */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} R1;
	uint8_t data[];
};

/**
 * \brief CAN transmit FIFO element.
 */
struct _can_tx_fifo_entry {
	__IO union {
		struct {
			uint32_t ID : 29; /*!< Identifier */
			uint32_t RTR : 1; /*!< Remote Transmission Request */
			uint32_t XTD : 1; /*!< Extended Identifier */
			uint32_t ESI : 1; /*!< Error State Indicator */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} T0;
	__IO union {
		struct {
			uint32_t : 16;    /*!< Reserved */
			uint32_t DLC : 4; /*!< Data Length Code */
			uint32_t BRS : 1; /*!< Bit Rate Switch */
			uint32_t FDF : 1; /*!< FD Format */
			uint32_t : 1;     /*!< Reserved */
			uint32_t EFC : 1; /*!< Event FIFO Control */
			uint32_t MM : 8;  /*!< Message Marker */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} T1;
	uint8_t data[];
};

/**
 * \brief CAN transmit Event element.
 */
struct _can_tx_event_entry {
	__IO union {
		struct {
			uint32_t ID : 29; /*!< Identifier */
			uint32_t RTR : 1; /*!< Remote Transmission Request */
			uint32_t XTD : 1; /*!< Extended Identifier */
			uint32_t ESI : 1; /*!< Error State Indicator */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} R0;
	__IO union {
		struct {
			uint32_t TXTS : 16; /*!< Tx Timestamp */
			uint32_t DLC : 4;   /*!< Data Length Code */
			uint32_t BRS : 1;   /*!< Bit Rate Switch */
			uint32_t FDF : 1;   /*!< FD Format */
			uint32_t ET : 2;    /*!< Event Type */
			uint32_t MM : 8;    /*!< Message Marker */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} R1;
};

/**
 * \brief CAN standard message ID filter element structure.
 *
 *  Common element structure for standard message ID filter element.
 */
struct _can_standard_message_filter_element {
	__IO union {
		struct {
			uint32_t SFID2 : 11; /*!< Standard Filter ID 2 */
			uint32_t : 5;        /*!< Reserved */
			uint32_t SFID1 : 11; /*!< Standard Filter ID 1 */
			uint32_t SFEC : 3;   /*!< Standard Filter Configuration */
			uint32_t SFT : 2;    /*!< Standard Filter Type */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} S0;
};

#define _CAN_SFT_RANGE 0     /*!< Range filter from SFID1 to SFID2 */
#define _CAN_SFT_DUAL 1      /*!< Dual ID filter for SFID1 or SFID2 */
#define _CAN_SFT_CLASSIC 2   /*!< Classic filter: SFID1 = filter, SFID2 = mask */
#define _CAN_SFEC_DISABLE 0  /*!< Disable filter element */
#define _CAN_SFEC_STF0M 1    /*!< Store in Rx FIFO 0 if filter matches */
#define _CAN_SFEC_STF1M 2    /*!< Store in Rx FIFO 1 if filter matches */
#define _CAN_SFEC_REJECT 3   /*!< Reject ID if filter matches */
#define _CAN_SFEC_PRIORITY 4 /*!< Set priority if filter matches. */
#define _CAN_SFEC_PRIF0M 5   /*!< Set priority and store in FIFO 0 if filter matches */
#define _CAN_SFEC_PRIF1M 6   /*!< Set priority and store in FIFO 1 if filter matches. */
#define _CAN_SFEC_STRXBUF 7  /*!< Store into Rx Buffer or as debug message, configuration of SFT[1:0] ignored. */

#define _CAN_EFT_RANGE 0     /*!< Range filter from SFID1 to SFID2 */
#define _CAN_EFT_DUAL 1      /*!< Dual ID filter for SFID1 or SFID2 */
#define _CAN_EFT_CLASSIC 2   /*!< Classic filter: SFID1 = filter, SFID2 = mask */
#define _CAN_EFEC_DISABLE 0  /*!< Disable filter element */
#define _CAN_EFEC_STF0M 1    /*!< Store in Rx FIFO 0 if filter matches */
#define _CAN_EFEC_STF1M 2    /*!< Store in Rx FIFO 1 if filter matches */
#define _CAN_EFEC_REJECT 3   /*!< Reject ID if filter matches */
#define _CAN_EFEC_PRIORITY 4 /*!< Set priority if filter matches. */
#define _CAN_EFEC_PRIF0M 5   /*!< Set priority and store in FIFO 0 if filter matches */
#define _CAN_EFEC_PRIF1M 6   /*!< Set priority and store in FIFO 1 if filter matches. */
#define _CAN_EFEC_STRXBUF 7  /*!< Store into Rx Buffer or as debug message, configuration of SFT[1:0] ignored. */
/**
 * \brief CAN extended message ID filter element structure.
 *
 *  Common element structure for extended message ID filter element.
 */
struct _can_extended_message_filter_element {
	__IO union {
		struct {
			uint32_t EFID1 : 29; /*!< bit: Extended Filter ID 1 */
			uint32_t EFEC : 3;   /*!< bit: Extended Filter Configuration */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} F0;
	__IO union {
		struct {
			uint32_t EFID2 : 29; /*!< bit: Extended Filter ID 2 */
			uint32_t : 1;        /*!< bit: Reserved */
			uint32_t EFT : 2;    /*!< bit: Extended Filter Type */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} F1;
};

struct _can_context {
	uint8_t *                   rx_fifo;  /*!< receive message fifo */
	uint8_t *                   tx_fifo;  /*!< transfer message fifo */
	struct _can_tx_event_entry *tx_event; /*!< transfer event fifo */
	/* Standard filter List */
	struct _can_standard_message_filter_element *rx_std_filter;
	/* Extended filter List */
	struct _can_extended_message_filter_element *rx_ext_filter;
};

/**
 * \brief CAN device descriptor forward declaration
 */
struct _can_async_device;

/**
 * \brief CAN callback types
 */
enum can_async_callback_type {
	CAN_ASYNC_RX_CB, /*!< A new message arrived */
	CAN_ASYNC_TX_CB, /*!< A message transmitted */
	CAN_ASYNC_IRQ_CB /*!< Message error of some kind on the CAN bus IRQ */
};

enum can_async_interrupt_type {
	CAN_IRQ_EW, /*!< Error warning, Error counter has reached the
	              error warning limit of 96, An error count value
	              greater than about 96 indicates a heavily disturbed
	              bus. It may be of advantage to provide means to test
	              for this condition. Refer to ISO 11898-1 (Bosch CAN
	              specification 2.0 part A,B)
	              */
	CAN_IRQ_EA, /*!< Error Active State, The CAN node normally take
	               part in bus communication and sends an ACTIVE ERROR
	               FLAG when an error has been detected.
	               Refer to ISO 11898-1 (7)
	               */
	CAN_IRQ_EP, /*!< Error Passive State, The Can node goes into error
	              passive state if at least one of its error counters is
	              greater than 127. It still takes part in bus
	              activities, but it sends a passive error frame only,
	              on errors. Refer to ISO 11898-1 (7)
	              */
	CAN_IRQ_BO, /*!< Bus Off State, The CAN node is 'bus off' when the
	               TRANSMIT ERROR COUNT is greater than or equal to 256.
	               Refer to ISO 11898-1 (7)
	               */
	CAN_IRQ_DO  /*!< Data Overrun in receive queue. A message was lost
	               because the messages in the queue was not reading and
	               releasing fast enough. There is not enough space for
	               a new message in receive queue.
	               */
};
/**
 * \brief CAN interrupt handlers structure
 */
struct _can_async_callback {
	void (*tx_done)(struct _can_async_device *dev);
	void (*rx_done)(struct _can_async_device *dev);
	void (*irq_handler)(struct _can_async_device *dev, enum can_async_interrupt_type type);
};

/**
 * \brief CAN device descriptor
 */
struct _can_async_device {
	Can *                      hw;      /*!< CAN hardware pointer */
	struct _can_async_callback cb;      /*!< CAN interrupt handler */
	struct _irq_descriptor     irq;     /*!< Interrupt descriptor */
	void *                     context; /*!< CAN hardware context */
};

/**
 * \brief Initialize CAN.
 *
 * This function initializes the given CAN device descriptor.
 *
 * \param[in, out] dev   A CAN device descriptor to initialize
 * \param[in]      hw    The pointer to hardware instance
 *
 * \return Initialization status.
 */
int32_t _can_async_init(struct _can_async_device *const dev, Can *const hw);

/**
 * \brief Deinitialize CAN.
 *
 * This function deinitializes the given can device descriptor.
 *
 * \param[in] dev The CAN device descriptor to deinitialize
 *
 * \return De-initialization status.
 */
int32_t _can_async_deinit(struct _can_async_device *const dev);

/**
 * \brief Enable CAN
 *
 * This function enable CAN by the given can device descriptor.
 *
 * \param[in] dev The CAN device descriptor to enable
 *
 * \return Enabling status.
 */
int32_t _can_async_enable(struct _can_async_device *const dev);

/**
 * \brief Disable CAN
 *
 * This function disable CAN by the given can device descriptor.
 *
 * \param[in] dev The CAN descriptor to disable
 *
 * \return Disabling status.
 */
int32_t _can_async_disable(struct _can_async_device *const dev);

/**
 * \brief Read a CAN message
 *
 * \param[in] dev   The CAN device descriptor to read message.
 * \param[in] msg   The CAN message to read to.
 *
 * \return The status of read message.
 */
int32_t _can_async_read(struct _can_async_device *const dev, struct can_message *msg);

/**
 * \brief Write a CAN message
 *
 * \param[in] dev   The CAN device descriptor to write message.
 * \param[in] msg   The CAN message to write.
 *
 * \return The status of write message.
 */
int32_t _can_async_write(struct _can_async_device *const dev, struct can_message *msg);

/**
 * \brief Set CAN Interrupt State
 *
 * \param[in] dev   The CAN device descriptor
 * \param[in] type  Callback type
 * \param[in] state ture for enable or false for disable
 *
 */
void _can_async_set_irq_state(struct _can_async_device *const dev, enum can_async_callback_type type, bool state);

/**
 * \brief Return number of read errors
 *
 * This function return number of read errors
 *
 * \param[in] dev The CAN device descriptor pointer
 *
 * \return Number of read errors.
 */
uint8_t _can_async_get_rxerr(struct _can_async_device *const dev);

/**
 * \brief Return number of write errors
 *
 * This function return number of write errors
 *
 * \param[in] dev The CAN device descriptor pointer
 *
 * \return Number of write errors.
 */
uint8_t _can_async_get_txerr(struct _can_async_device *const dev);

/**
 * \brief Set CAN to the specified mode
 *
 * This function set CAN to a specified mode
 *
 * \param[in] dev The CAN device descriptor pointer
 * \param[in] mode  CAN operation mode
 *
 * \return Status of the operation
 */
int32_t _can_async_set_mode(struct _can_async_device *const dev, enum can_mode mode);

/**
 * \brief Set CAN to the specified mode
 *
 * This function set CAN to a specified mode
 *
 * \param[in] dev The CAN device descriptor pointer
 * \param[in] index   Index of Filter list
 * \param[in] filter  CAN Filter struct, NULL for clear filter
 *
 * \return Status of the operation
 */
int32_t _can_async_set_filter(struct _can_async_device *const dev, uint8_t index, enum can_format fmt,
                              struct can_filter *filter);

/**@}*/

/**
 * \brief CAN Asynchronous descriptor
 *
 * The CAN descriptor forward declaration.
 */
struct can_async_descriptor;

/**
 * Callback for CAN interrupt
 */
typedef void (*can_cb_t)(struct can_async_descriptor *const descr);

/**
 * \brief CAN callbacks
 */
struct can_callbacks {
	can_cb_t tx_done;
	can_cb_t rx_done;
	void (*irq_handler)(struct can_async_descriptor *const descr, enum can_async_interrupt_type type);
};

/**
 * \brief CAN descriptor
 */
struct can_async_descriptor {
	struct _can_async_device dev; /*!< CAN HPL device descriptor */
	struct can_callbacks     cb;  /*!< CAN Interrupt Callbacks handler */
};

/**
 * \brief Initialize CAN.
 *
 * This function initializes the given CAN descriptor.
 *
 * \param[in, out] descr A CAN descriptor to initialize.
 * \param[in]      hw    The pointer to hardware instance.
 *
 * \return Initialization status.
 */
int32_t can_async_init(struct can_async_descriptor *const descr, Can *const hw);

/**
 * \brief Deinitialize CAN.
 *
 * This function deinitializes the given CAN descriptor.
 *
 * \param[in, out] descr The CAN descriptor to deinitialize.
 *
 * \return De-initialization status.
 */
int32_t can_async_deinit(struct can_async_descriptor *const descr);

/**
 * \brief Enable CAN
 *
 * This function enables CAN by the given can descriptor.
 *
 * \param[in] descr The CAN descriptor to enable.
 *
 * \return Enabling status.
 */
int32_t can_async_enable(struct can_async_descriptor *const descr);

/**
 * \brief Disable CAN
 *
 * This function disables CAN by the given can descriptor.
 *
 * \param[in] descr The CAN descriptor to disable.
 *
 * \return Disabling status.
 */
int32_t can_async_disable(struct can_async_descriptor *const descr);

/**
 * \brief Read a CAN message
 *
 * \param[in] descr The CAN descriptor to read message.
 * \param[in] msg   The CAN message to read to.
 *
 * \return The status of read message.
 */
int32_t can_async_read(struct can_async_descriptor *const descr, struct can_message *msg);

/**
 * \brief Write a CAN message
 *
 * \param[in] descr The CAN descriptor to write message.
 * \param[in] msg   The CAN message to write.
 *
 * \return The status of write message.
 */
int32_t can_async_write(struct can_async_descriptor *const descr, struct can_message *msg);

/**
 * \brief Register CAN callback function to interrupt
 *
 * \param[in] descr The CAN descriptor
 * \param[in] type  Callback type
 * \param[in] cb    A callback function, passing NULL will de-register any
 *                  registered callback
 *
 * \return The status of callback assignment.
 */
int32_t can_async_register_callback(struct can_async_descriptor *const descr, enum can_async_callback_type type,
                                    FUNC_PTR cb);

/**
 * \brief Return number of read errors
 *
 * This function returns the number of read errors.
 *
 * \param[in] descr The CAN descriptor pointer
 *
 * \return The number of read errors.
 */
uint8_t can_async_get_rxerr(struct can_async_descriptor *const descr);

/**
 * \brief Return number of write errors
 *
 * This function returns the number of write errors.
 *
 * \param[in] descr The CAN descriptor pointer
 *
 * \return The number of write errors.
 */
uint8_t can_async_get_txerr(struct can_async_descriptor *const descr);

/**
 * \brief Set CAN to the specified mode
 *
 * This function sets CAN to a specified mode.
 *
 * \param[in] descr The CAN descriptor pointer
 * \param[in] mode  The CAN operation mode
 *
 * \return Status of the operation.
 */
int32_t can_async_set_mode(struct can_async_descriptor *const descr, enum can_mode mode);

/**
 * \brief Set CAN Filter
 *
 * This function sets CAN to a specified mode.
 *
 * \param[in] descr The CAN descriptor pointer
 * \param[in] index   Index of Filter list
 * \param[in] fmt     CAN Indentify Type
 * \param[in] filter  CAN Filter struct, NULL for clear filter
 *
 * \return Status of the operation.
 */
int32_t can_async_set_filter(struct can_async_descriptor *const descr, uint8_t index, enum can_format fmt,
                             struct can_filter *filter);

/**
 * \brief Retrieve the current driver version
 *
 * \return The current driver version.
 */
uint32_t can_async_get_version(void);

#endif /* SRC_CAN_CANDRIVER_H_ */
