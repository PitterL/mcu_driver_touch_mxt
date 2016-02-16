#ifndef __QT_Controller_H__
#define __QT_Controller_H__

#include "Base.h"

#define QT_EXT extern

//*****************************************************************************
//		Common Defines
//*****************************************************************************
/*
typedef signed char             int8_t ;  //!< 8-bit signed integer.
typedef unsigned char           uint8_t ;  //!< 8-bit unsigned integer.
typedef signed short int        int16_t;  //!< 16-bit signed integer.
typedef unsigned short int      uint16_t;  //!< 16-bit unsigned integer.
typedef signed long int         int32_t;  //!< 32-bit signed integer.
typedef unsigned long int       uint32_t;  //!< 32-bit unsigned integer.
typedef signed long long int    int64_t;  //!< 64-bit signed integer.
typedef unsigned long long int  uint64_t;  //!< 64-bit unsigned integer.
typedef float                   float32_t;  //!< 32-bit floating-point number.
typedef double                  float64_t;  //!< 64-bit floating-point number.
*/

#define false   0
#define true    1

//#define NULL           0

/*
 * casts are necessary for constants, because we never know how for sure
 * how U/UL/ULL map to __u16, __u32, __u64. At least not in a portable way.
 */
#define BIG_ENDIAN

typedef  unsigned short __u16;
typedef  unsigned long  __u32;

#define ___constant_swab16(x) ((__u16)(				\
	(((__u16)(x) & (__u16)0x00ffU) << 8) |			\
	(((__u16)(x) & (__u16)0xff00U) >> 8)))

#define ___constant_swab32(x) ((__u32)(				\
	(((__u32)(x) & (__u32)0x000000ffUL) << 24) |		\
	(((__u32)(x) & (__u32)0x0000ff00UL) <<  8) |		\
	(((__u32)(x) & (__u32)0x00ff0000UL) >>  8) |		\
	(((__u32)(x) & (__u32)0xff000000UL) >> 24)))

/**
 * __swab16 - return a byteswapped 16-bit value
 * @x: value to byteswap
 */
#define __swab16(x)	___constant_swab16(x)

/**
 * __swab32 - return a byteswapped 32-bit value
 * @x: value to byteswap
 */
#define __swab32(x)	___constant_swab32(x)

#if defined(BIG_ENDIAN)
#define cpu_to_le32p(x) ___constant_swab32(x)
#define le32_to_cpup(x) ___constant_swab32(x)
#define cpu_to_le16p(x) ___constant_swab16(x)
#define le16_to_cpup(x) ___constant_swab16(x)
#define hi16_to_cpup(x) (x)
#else
#define cpu_to_le32p(x) (x)
#define le32_to_cpup(x) (x)
#define cpu_to_le16p(x) (x)
#define le16_to_cpup(x) (x)
#define hi16_to_cpup(x) ___constant_swab16(x)
#endif
//*****************************************************************************
//		info_block_driver
//*****************************************************************************

/*! \brief Object table element struct. */
typedef struct
{
	uint8_t object_type;     // Object type ID. */
	uint16_t i2c_address;    // Start address of the obj config structure. 
	uint8_t size;            // Byte length of the obj config structure -1.
	uint8_t instances;       // Number of objects of this obj. type -1. 
        uint8_t num_report_ids;  // The max number of touches in a screen,
                              //   max number of sliders in a slider array, etc.
} object_t;

/*! \brief Info ID struct. */
typedef struct
{
	uint8_t family_id;            /* address 0 */
	uint8_t variant_id;           /* address 1 */
	
	uint8_t version;              /* address 2 */
	uint8_t build;                /* address 3 */
	
	uint8_t matrix_x_size;        /* address 4 */
	uint8_t matrix_y_size;        /* address 5 */
                                     /*! Number of entries in the object table. The actual number of objects
    * can be different if any object has more than one instance. */
	uint8_t num_declared_objects; /* address 6 */
} info_id_t;

/*! \brief Info block struct holding ID and object table data and their CRC sum.
*
* Info block struct. Similar to one in info_block.h, but since
* the size of object table is not known beforehand, it's pointer to an
* array instead of an array. This is not defined in info_block.h unless
* we are compiling with IAR AVR or AVR32 compiler (__ICCAVR__ or __ICCAVR32__
* is defined). If this driver is compiled with those compilers, the
* info_block.h needs to be edited to not include that struct definition.
*
* CRC is 24 bits, consisting of CRC and CRC_hi; CRC is the lower 16 bytes and
* CRC_hi the upper 8.
*
*/

typedef struct
{
    /*! Info ID struct. */
    info_id_t info_id;
	
    /*! Pointer to an array of objects. */
    object_t *objects;
	
    /*! CRC field, low bytes. */
    uint16_t CRC;
	
    /*! CRC field, high byte. */
    uint8_t CRC_hi;
} info_block_t;

typedef struct
{
	uint8_t object_type;     // Object type. */
	uint8_t instance;        // Instance number. */
} report_id_map_t;

//*****************************************************************************
//		std_objects_driver
//*****************************************************************************

/*! ===Header File Version Number=== */
#define OBJECT_LIST__VERSION_NUMBER     0x11

/*! \defgroup OBJECT_LIST ===Object Type Number List===
 This list contains the enumeration of each of the object types used in the
 chip information table.

 The Structure of the name used is:
 <OBJECTPURPOSE>_<OBJECTDESCRIPTION>_T<TYPENUMBER>

 Possible Object Purposes include:
 DEBUG, GEN (General), TOUCH (Touch Sensitive Objects), PROCI
 (Processor - instance), PROCT (Processor - type), PROCG (Processor - global)

 Below is the actual list, reserved entries should not be used, spare entries
 are available for use. No entries should ever be re-used once they have been
 included in the list. Once an entry is added its configuration and/or status
 structures should be completed and commented below.
 */

#define RESERVED_T0                               0u
#define RESERVED_T1                               1u
#define GEN_MESSAGEPROCESSOR_T5                   5u
#define GEN_COMMANDPROCESSOR_T6                   6u
#define TOUCH_MULTITOUCHSCREEN_T9                 9u
#define TOUCH_SINGLETOUCHSCREEN_T10               10u
#define TOUCH_KEYARRAY_T15                        15u
#define PROCI_ONETOUCHGESTUREPROCESSOR_T24        24u
#define PROCI_TWOTOUCHGESTUREPROCESSOR_T27        27u
/*
 * All entries spare up to 255
*/
#define RESERVED_T255                             255u


/* MXT_TOUCH_MULTI_T9 status */
#define MXT_T9_UNGRIP		(1 << 0)
#define MXT_T9_SUPPRESS		(1 << 1)
#define MXT_T9_AMP		(1 << 2)
#define MXT_T9_VECTOR		(1 << 3)
#define MXT_T9_MOVE		(1 << 4)
#define MXT_T9_RELEASE		(1 << 5)
#define MXT_T9_PRESS		(1 << 6)
#define MXT_T9_DETECT		(1 << 7)


//*****************************************************************************
//		touch_driver
//*****************************************************************************

/*! \brief Touch driver version number. */
#define TOUCH_DRIVER_VERSION    0x02u

#define I2C_APPL_ADDR_0         0x4b    //0x4A,0x4B,0x24,0x25,
#define I2C_APPL_ADDR_1         0x4b
#define I2C_BOOT_ADDR_0         0x4b
#define I2C_BOOT_ADDR_1         0x4b

#define NUM_OF_I2C_ADDR         4

/* \brief Defines CHANGE line active mode. */
#define CHANGELINE_NEGATED          0u
#define CHANGELINE_ASSERTED         1u

#define CONNECT_OK                  1u
#define CONNECT_ERROR               2u

#define READ_MEM_OK                 1u
#define READ_MEM_FAILED             2u

#define MESSAGE_READ_OK             1u
#define MESSAGE_READ_FAILED         2u

#define WRITE_MEM_OK                1u
#define WRITE_MEM_FAILED            2u

#define I2C_INIT_OK                 1u
#define I2C_INIT_FAIL               2u

#define CRC_CALCULATION_OK          1u
#define CRC_CALCULATION_FAILED      2u

#define ID_MAPPING_OK               1u
#define ID_MAPPING_FAILED           2u

#define ID_DATA_OK                  1u
#define ID_DATA_NOT_AVAILABLE       2u

enum driver_setup_t {DRIVER_SETUP_OK, DRIVER_SETUP_INCOMPLETE};

/* Array of I2C addresses where we are trying to find the chip. */
extern uint8_t i2c_addresses[];

/*! \brief Returned by get_object_address() if object is not found. */
#define OBJECT_NOT_FOUND   0u

/*! Address where object table starts at touch IC memory map. */
#define OBJECT_TABLE_START_ADDRESS      7U

/*! Size of one object table element in touch IC memory map. */
#define OBJECT_TABLE_ELEMENT_SIZE       6U

/*! Offset to REPORTALL register from the beginning of command processor. */
#define REPORTATLL_OFFSET   3u

/*----------------------------------------------------------------------------
Function prototypes.
----------------------------------------------------------------------------*/

/* Initializes the touch driver: tries to connect to given address,
* sets the message handler pointer, reads the info block and object
* table, sets the message processor address etc. */

QT_EXT uint8_t init_touch_driver(uint8_t I2C_address, void (*handler)(uint8_t *, uint8_t));
QT_EXT uint8_t close_touch_driver();
QT_EXT uint8_t get_variant_id(uint8_t *variant);
QT_EXT uint8_t get_family_id(uint8_t *family_id);
QT_EXT uint8_t get_build_number(uint8_t *build);
QT_EXT uint8_t get_version(uint8_t *version);

QT_EXT uint8_t get_object_size(uint8_t object_type);
QT_EXT uint8_t type_to_report_id(uint8_t object_type, uint8_t instance);
QT_EXT uint8_t report_id_to_type(uint8_t report_id, uint8_t *instance);
QT_EXT uint8_t read_id_block(info_id_t *id);
QT_EXT uint8_t get_max_report_id();
QT_EXT uint16_t get_object_address(uint8_t object_type, uint8_t instance);
QT_EXT uint32_t get_stored_infoblock_crc(void);
QT_EXT uint8_t calculate_infoblock_crc(uint32_t *crc_pointer);
QT_EXT uint32_t CRC_24(uint32_t crc, uint8_t byte1, uint8_t byte2);

QT_EXT info_block_t *info_block;
QT_EXT report_id_map_t *report_id_map;
QT_EXT int max_report_id ;
QT_EXT uint8_t max_message_length;
QT_EXT uint16_t message_processor_address;
QT_EXT uint16_t command_processor_address;
QT_EXT enum driver_setup_t driver_setup;
QT_EXT uint8_t *msg;

/*! Touch device I2C Address */

QT_EXT uint8_t QT_i2c_address;
QT_EXT  uint8_t QT_i2c_boot_address;
QT_EXT uint8_t chip_detected_flag;

//*****************************************************************************
//		main
//*****************************************************************************
extern void *malloc(unsigned int num_bytes);
QT_EXT void get_message(void);

/*! Pointer to message handler provided by main app. */
QT_EXT void (*application_message_handler)(uint8_t *, uint8_t);
QT_EXT void message_handler(uint8_t *msg, uint8_t length);

QT_EXT uint8_t init_touch_app(void);

QT_EXT uint8_t write_mem( uint16_t Address, uint8_t ByteCount, uint8_t *Data );
QT_EXT uint8_t read_mem( uint16_t Address, uint8_t ByteCount, uint8_t *Data );
QT_EXT uint8_t read_uint16_t( uint16_t Address, uint16_t *Data );

QT_EXT uint8_t ChangeLineStatus( void );

QT_EXT unsigned char tmsg[];
QT_EXT unsigned short x_pos, y_pos, s_t9sts;

//*****************************************************************************
//  I2C Functions
//*****************************************************************************
QT_EXT void I2cStart();
QT_EXT void I2cStop();
QT_EXT uint8_t I2cTxByte( uint8_t TxData);
QT_EXT uint8_t I2cRxByte( uint8_t AckState );

QT_EXT unsigned char  TWI_Wait(void);
QT_EXT void Twi_SendStop(void);
QT_EXT void InitTwi(void);
QT_EXT uint8_t address_slave(void);

// use TWI (I2C) peripheral of mega32
#define TWI_START  0x08  //启动信号发送成功
#define TWI_REP_START  0x10  //重复启动信号发送成功
#define TWSR_STATUS_MASK  0xf8 //TWI状态寄存器
#define TWI_TWBR_200KHZ   72   //18,设置TWI的波特率200kbps，8MHz,TWPS=0,72=50hkz
//TWBR =(FOSC/SCLf -16)/2
#define TWI_ADR_BITS  1  //地址左移1位
#define TWI_READ_BIT  0  //在0位读出或者写入
#define TWI_MRX_ADR_ACK  0x40  //主机接收- SLA+R 应答
#define TWI_MTX_ADR_ACK  0x18  //主机发送- SLA+W 应答
#define TWI_MRX_DATA_ACK 0x50  //主机接收- data received 应答
#define TWI_MTX_DATA_ACK 0x28  //主机发送- data transmitted 应答
#define FOSC  8000000   //8MHz

QT_EXT unsigned char address_pointer;
QT_EXT unsigned char TwiWaitTimer;

#endif //__QT_Controller_H__

