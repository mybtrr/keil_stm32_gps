#ifndef __GPS_H
#define __GPS_H
#pragma once

#include "usart.h"
#include "string.h"
#include "delay.h"
#include "led.h"


/*********************gps配置过程宏定义************************************************************************************************************************/
//头标识符
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

/* Message Classes */
#define UBX_CLASS_NAV		0x01
#define UBX_CLASS_ACK		0x05
#define UBX_CLASS_CFG		0x06
#define UBX_CLASS_MON		0x0A

/* Message IDs */
#define UBX_ID_CFG_PRT		0x00
#define UBX_ID_CFG_RATE	  0x08
#define UBX_ID_CFG_MSG		0x01
#define UBX_ID_CFG_NAV5	0x24


#define UBX_ID_ACK_NAK		0x00
#define UBX_ID_ACK_ACK		0x01

#define UBX_ID_NAV_PVT		0x07


/* Message Classes & IDs */
#define UBX_MSG_CFG_PRT		((UBX_CLASS_CFG) | UBX_ID_CFG_PRT << 8)
#define UBX_MSG_CFG_RATE	((UBX_CLASS_CFG) | UBX_ID_CFG_RATE << 8)
#define UBX_MSG_CFG_MSG		((UBX_CLASS_CFG) | UBX_ID_CFG_MSG << 8)
#define UBX_MSG_CFG_NAV5	((UBX_CLASS_CFG) | UBX_ID_CFG_NAV5 << 8)

#define UBX_MSG_ACK_NAK		((UBX_CLASS_ACK) | UBX_ID_ACK_NAK << 8)
#define UBX_MSG_ACK_ACK		((UBX_CLASS_ACK) | UBX_ID_ACK_ACK << 8)

#define UBX_MSG_NAV_PVT		((UBX_CLASS_NAV) | UBX_ID_NAV_PVT << 8)


/* TX CFG-PRT message contents */

#define UBX_TX_CFG_PRT_PORTID		0x01		/**< UART1 */

#define UBX_TX_CFG_PRT_MODE		0x000008D0	/**< 0b0000100011010000: 8N1 */
#define UBX_TX_CFG_PRT_BAUDRATE 115200
#define UBX_TX_CFG_PRT_INPROTOMASK_GPS	(0x01)	/**< UBX in */
#define UBX_TX_CFG_PRT_OUTPROTOMASK_GPS	(0x01)			/**< UBX out */


/* TX CFG-RATE message contents */

#define UBX_TX_CFG_RATE_MEASINTERVAL		100		/**< 100ms for 10Hz 目前ublox8版本并不稳定，(ublox9* boards use 10Hz) */
#define UBX_TX_CFG_RATE_NAVRATE		1		/**< cannot be changed */
#define UBX_TX_CFG_RATE_TIMEREF		0		/**< 0: UTC格林尼治时间, 1: GPS time */

/* TX CFG-NAV5 message contents */
#define UBX_TX_CFG_NAV5_MASK		0x0005		/**< Only update dynamic model and fix mode */
#define UBX_TX_CFG_NAV5_DYNMODEL		7		/**Dynamic platform model:--7: airborne with <2g acceleration */
#define UBX_TX_CFG_NAV5_FIXMODE		2		/**< 1 2D only, 2 3D only, 3 Auto 2D/3D */




/********************通讯过程中使用的结构体变量***********************************************************************************************************************/
/* General: Header */
typedef struct {
	u8	  sync1;
	u8	  sync2;
	u16 	msg;
	u16 	length;
} ubx_header_t;

/* General: Checksum */
typedef struct {
	u8		ck_a;
	u8		ck_b;
} ubx_checksum_t ;

/* Rx NAV-PVT (ubx8) */
typedef struct {
	u32	  iTOW;		/**< GPS Time of Week [ms] */
	u16	  year; 		/**< Year (UTC)*/
	u8		month; 		/**< Month, range 1..12 (UTC) */
	u8		day; 		/**< Day of month, range 1..31 (UTC) */
	u8		hour; 		/**< Hour of day, range 0..23 (UTC) */
	u8		min; 		/**< Minute of hour, range 0..59 (UTC) */
	u8		sec;		/**< Seconds of minute, range 0..60 (UTC) */
	u8		valid; 		/**< Validity flags (see UBX_RX_NAV_PVT_VALID_...) */
	u32	  tAcc; 		/**< Time accuracy estimate (UTC) [ns] */
	int32_t		nano;		/**< Fraction of second (UTC) [-1e9...1e9 ns] */
	u8		fixType;	/**< GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix */
	u8		flags;		/**< Fix Status Flags (see UBX_RX_NAV_PVT_FLAGS_...) */
	u8		reserved1;
	u8		numSV;		/**< Number of SVs used in Nav Solution */
	int32_t		lon;		/**< Longitude [1e-7 deg] */
	int32_t		lat;		/**< Latitude [1e-7 deg] */
	int32_t		height;		/**< Height above ellipsoid [mm] */
	int32_t		hMSL;		/**< Height above mean sea level [mm] */
	u32	hAcc;  		/**< Horizontal accuracy estimate [mm] */
	u32	vAcc;  		/**< Vertical accuracy estimate [mm] */
	int32_t		velN;		/**< NED north velocity [mm/s]*/
	int32_t		velE;		/**< NED east velocity [mm/s]*/
	int32_t		velD;		/**< NED down velocity [mm/s]*/
	int32_t		gSpeed;		/**< Ground Speed (2-D) [mm/s] */
	int32_t		headMot;	/**< Heading of motion (2-D) [1e-5 deg] */
	u32	  sAcc;		/**< Speed accuracy estimate [mm/s] */
	u32  	headAcc;	/**< Heading accuracy estimate (motion and vehicle) [1e-5 deg] */
	u16	  pDOP;		/**< Position DOP [0.01] */
	u16	  reserved2;
	u32  	reserved3;
	int32_t		headVeh;	/**< (ubx8+ only) Heading of vehicle (2-D) [1e-5 deg] */
	u16	  magDec;		/**Magnetic declination deg */
	u16 	magAcc;   /*Magnetic declination accuracy deg */
	//uint32_t	reserved4;	/**< (ubx8+ only) */
} ubx_payload_rx_nav_pvt_t;

#define UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX8	(sizeof(ubx_payload_rx_nav_pvt_t))

/* 消息类与ID */
typedef struct {
			u8	msgClass;
			u8	msgID;
		}class_id;

/*class mag ID分与合*/
typedef union {
		u16	cmid_combine;
		class_id  cmid_separate;
	}cmid_sc;
		
/* Rx ACK-ACK */
typedef	union {
	u16	msg;
	class_id  cmid;
} ubx_payload_rx_ack_ack_t;

/* Rx ACK-NAK */
typedef	union {
	u16	msg;
	class_id  cmid;
} ubx_payload_rx_ack_nak_t;

/* Tx CFG-PRT */
typedef struct {
	u8		portID;
	u8		reserved0;
	u16  	txReady;
	u32 	mode;
	u32  	baudRate;
	u16 	inProtoMask;
	u16 	outProtoMask;
	u16 	flags;
	u16  	reserved5;
} ubx_payload_tx_cfg_prt_t;

/* Tx CFG-RATE */
typedef struct {
	u16  	measRate;	/**< Measurement Rate, GPS measurements are taken every measRate milliseconds */
	u16  	navRate;	/**< Navigation Rate, in number of measurement cycles. This parameter cannot be changed, and must be set to 1 */
	u16 	timeRef;	/**< Alignment to reference time: 0 = UTC time, 1 = GPS time */
} ubx_payload_tx_cfg_rate_t;

/* Tx CFG-NAV5 */
typedef struct {
	u16  	mask;
	u8		dynModel;	/**< Dynamic Platform model: 0 Portable, 2 Stationary, 3 Pedestrian, 4 Automotive, 5 Sea, 6 Airborne <1g, 7 Airborne <2g, 8 Airborne <4g */
	u8		fixMode;	/**< Position Fixing Mode: 1 2D only, 2 3D only, 3 Auto 2D/3D */
	int32_t		fixedAlt;
	u32 	fixedAltVar;
	int8_t		minElev;
	u8		drLimit;
	u16 	pDop;
	u16 	tDop;
	u16 	pAcc;
	u16 	tAcc;
	u8		staticHoldThresh;
	u8		dgpsTimeOut;
	u8		cnoThreshNumSVs;	/**< (ubx7+ only, else 0) */
	u8		cnoThresh;		/**< (ubx7+ only, else 0) */
	u16   reserved;
	u16 	staticHoldMaxDist;	/**< (ubx8+ only, else 0) */
	u8		utcStandard;		/**< (ubx8+ only, else 0) */
	u8		reserved3;
	u32 	reserved4;
} ubx_payload_tx_cfg_nav5_t;

/* Tx CFG-MSG */
typedef struct {
	cmid_sc cmid_s2c1;
	u8  port_iic_rate;
	u8  port_uart1_rate;
	u8  port_uart2_rate;
	u8  port_usb_rate;
	u8  port_spi_rate;
	u8  reserved;
} ubx_payload_tx_cfg_msg_t;

/* General message and payload buffer union */
typedef union {
	ubx_payload_rx_nav_pvt_t		payload_rx_nav_pvt;
	ubx_payload_rx_ack_ack_t		payload_rx_ack_ack;
	ubx_payload_rx_ack_nak_t		payload_rx_ack_nak;
	ubx_payload_tx_cfg_prt_t		payload_tx_cfg_prt;
	ubx_payload_tx_cfg_rate_t		payload_tx_cfg_rate;
	ubx_payload_tx_cfg_nav5_t		payload_tx_cfg_nav5;
	ubx_payload_tx_cfg_msg_t		payload_tx_cfg_msg;
} ubx_buf_t;


/* Decoder state gps数据解析状态机*/
typedef enum {
	UBX_DECODE_SYNC1 = 0,
	UBX_DECODE_SYNC2,
	UBX_DECODE_CLASS,
	UBX_DECODE_ID,
	UBX_DECODE_LENGTH1,
	UBX_DECODE_LENGTH2,
	UBX_DECODE_PAYLOAD,
	UBX_DECODE_CHKSUM1,
	UBX_DECODE_CHKSUM2
} ubx_decode_state_t;

/* Rx message state MCU对接收的不同class数据消息处理方式状态机*/
typedef enum {
	UBX_RXMSG_IGNORE = 0,
	UBX_RXMSG_HANDLE,
	UBX_RXMSG_DISABLE,
	UBX_RXMSG_ERROR_LENGTH
} ubx_rxmsg_state_t;


/* ACK state 配置后等待应答状态机*/
typedef enum {
	UBX_ACK_IDLE = 0,
	UBX_ACK_WAITING,
	UBX_ACK_GOT_ACK,
	UBX_ACK_GOT_NAK
} ubx_ack_state_t;


/************************给飞控的跟踪数据*****************************************************************************************************************/
#pragma pack(1)
/*follow数据*/
typedef struct
{
	u8  head[5];
	u16 buflen;
	u8  command;
	u8  command_re;
	double lat;
	double lon;
	float alt;
	float vy;
	float vx;
	float vz;
	u8  numSV;
	u16 mag_dec; 
	u8  flags;
  u8  fixType;
	u16 crc_check;
}EXYF_FOLLOW;
#pragma pack()



/*************************gps配置及通信函数****************************************************************************************************************/

//gps配置
void gps_config(void);

//发送信息给gps
void sendMessage(const u16 msg, const u8 *payload, const u16 length);

//校验和计算
void calcChecksum(const u8 *buffer, const u16 length, ubx_checksum_t *checksum);


//配置后等待gps返回应答
u8 waitForAck(const u16 msg, const u16 timeout);

//接收gps数据
u8 receive(u16 timeout);

//解析gps数据
u8 parseChar(const u8 b);

//解析初始化
void decodeInit(void);

//gps接收数据的校验
void addByteToChecksum(const u8 b);

//有效数据接收初始化
u8 payloadRxInit(void);

//有效数据添加到解析结构体中
u8 payloadRxAdd(const u8 b);

//解析有效数据
u8 payloadRxDone(void);


extern EXYF_FOLLOW follow_data;//跟随数据

#endif






