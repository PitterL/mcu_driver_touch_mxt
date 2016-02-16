/*****************************************************************************
*       ATmel QT Controller   ---  tested by ATmega32
*       version: Het04
*       maintance by pitter liao : pitter.liao@atmel.com
*****************************************************************************/

// requirements to ATmega32 as host:
// MCU clock: 8MHz
// IAR:
// CLIB heap size: 0x100
// CSTACK=0x40  ,RSTACK=0x30

// I2C device address of QT Controller
// ADDR_SEL=0 ==> 0x4A
// ADDR_SEL=1 ==> 0x4B

// use TWI peripheral instead of I/O emulation

// THIERRY #define __TWI__   

#define __VER_1_4__
#define __BUILD_0xAA__  

#define QT_printf(...)   


#define SDA   PTCD_PTCD1
#define SDAD   PTCDD_PTCDD1
#define SCL   PTCD_PTCD0
#define SCLD   PTCDD_PTCDD0

/*****************************************************************************
*   Build Options
* ***************************************************************************/
//#define OPTION_WRITE_CONFIG     /* uncomment to force chip setup at startup */
//#define OPTION_PRINT_MESSAGES   /* uncomment to display touch messages via UART */

/*****************************************************************************
*   Include Files
* ***************************************************************************/
#include "MCF51AC256B.h"
#include "QT_Controller_h.h"
#include "I2C.h"
#include "io.h"
#include "Base.h"

// THIERRY #include <ioavr.h>
// THIERRY #include <intrinsics.h>
// THIERRY #include <stdio.h>
// THIERRY #include <stdlib.h>
// THIERRY #include <string.h>

//-------------------------------------------//
//	V A R I A B L E S THIERRY 
//-------------------------------------------//


/* Array of I2C addresses where we are trying to find the chip. */
uint8_t i2c_addresses[];

/*----------------------------------------------------------------------------
Function prototypes.
----------------------------------------------------------------------------*/

/* Initializes the touch driver: tries to connect to given address,
* sets the message handler pointer, reads the info block and object
* table, sets the message processor address etc. */

 uint8_t init_touch_driver(uint8_t I2C_address, void (*handler)(uint8_t *, uint8_t));
 uint8_t close_touch_driver();
 uint8_t get_variant_id(uint8_t *variant);
 uint8_t get_family_id(uint8_t *family_id);
 uint8_t get_build_number(uint8_t *build);
 uint8_t get_version(uint8_t *version);

 uint8_t get_object_size(uint8_t object_type);
 uint8_t type_to_report_id(uint8_t object_type, uint8_t instance);
 uint8_t report_id_to_type(uint8_t report_id, uint8_t *instance);
 uint8_t read_id_block(info_id_t *id);
 uint8_t get_max_report_id();
 uint16_t get_object_address(uint8_t object_type, uint8_t instance);
 uint32_t get_stored_infoblock_crc(void);
 uint8_t calculate_infoblock_crc(uint32_t *crc_pointer);
 uint32_t CRC_24(uint32_t crc, uint8_t byte1, uint8_t byte2);

 info_block_t *info_block;
 report_id_map_t *report_id_map;
 int max_report_id ;
 uint8_t max_message_length;
 uint16_t message_processor_address;
 uint16_t command_processor_address;
 enum driver_setup_t driver_setup;
 uint8_t *msg;

/*! Touch device I2C Address */

uint8_t QT_i2c_address;
uint8_t QT_i2c_boot_address;
uint8_t chip_detected_flag;

//*****************************************************************************
//		main
//*****************************************************************************
void *malloc(unsigned int num_bytes);
void get_message(void);

/*! Pointer to message handler provided by main app. */
void (*application_message_handler)(uint8_t *, uint8_t);
void message_handler(uint8_t *msg, uint8_t length);

uint8_t init_touch_app(void);

uint8_t write_mem( uint16_t Address, uint8_t ByteCount, uint8_t *Data );
uint8_t read_mem( uint16_t Address, uint8_t ByteCount, uint8_t *Data );
uint8_t read_uint16_t( uint16_t Address, uint16_t *Data );

uint8_t ChangeLineStatus( void );

unsigned char tmsg[20];
unsigned short x_pos, y_pos, s_t9sts;

//*****************************************************************************
//  I2C Functions
//*****************************************************************************
void I2cStart();
void I2cStop();
uint8_t I2cTxByte( uint8_t TxData);
uint8_t I2cRxByte( uint8_t AckState );

unsigned char  TWI_Wait(void);
void Twi_SendStop(void);
void InitTwi(void);
uint8_t address_slave(void);


unsigned char address_pointer;
unsigned char TwiWaitTimer;






//-------------------------------------------//
//	V A R I A B L E S END
//-------------------------------------------//

//外部中断0中断服务程序
// THIERRY #pragma vector=INT0_vect 
// THIERRY __interrupt static void INT0_ISR(void)
// THIERRY {
// THIERRY   asm("nop");  
// THIERRY  get_message();
// THIERRY }

//外部中断0初始化
// THIERRY void ExtIntConfigure(void)  //int1 low level interrupt,aaaaaaaaaaaaaaaaaaa
// THIERRY {
// THIERRY  MCUCR =0x02;   //the falling edge of int0 generates an interrupt
// THIERRY  GIFR  =0x40;   //clear int0 interrupt flag
// THIERRY  GICR  =0x40;   //enable int0 interrupt
// THIERRY }


/*****************************************************************************
*   Array of I2C addresses to try to find the touch chip from
* ***************************************************************************/

uint8_t i2c_addresses[] =
{
    I2C_APPL_ADDR_0,
    I2C_APPL_ADDR_1,
    I2C_BOOT_ADDR_0,
    I2C_BOOT_ADDR_1,
};


/******************************************************************************
*       QT Controller Object table init
* *****************************************************************************/

// THIERRY void delay_ms(unsigned short dly)
// THIERRY {
// THIERRY    while(dly--)
// THIERRY    {
// THIERRY        __delay_cycles(FOSC/1000);   //delay 1ms
// THIERRY    }
// THIERRY   asm("nop"); 
// THIERRY}



#define __QT_I2C_BLOCK___
/************************************************************************************************************
*       I2C Blocks
* ***************************************************************************/
/* I2c read/write flags */
#define I2C_WRITE_FLAG	0x00	/* bit 0 of slave-addres byte */
#define I2C_READ_FLAG	0x01

/* Retry times on NACK */
#define NACK_RETRY_MAX	10

typedef struct
{
    uint8_t slave_addr;
    uint8_t *txbuf;
    uint16_t txlen;
    uint8_t *rxbuf;
    uint16_t rxlen;

}i2c_package_t;


// THIERRY
/*typedef struct
{
  char chip;
  unsigned int addr;
  int addr_length;
  void *buffer;
  unsigned int length;
} twi_package_t;*/


/* Ports definitions               ***** MODIFY TO SUIT *****  */
/* Set these defines to specify used port bits  */
// THIERRY #define SCL_MASK            (1<<PC0)            /* SCL bit-mask e.g. bit1 */
// THIERRY #define SDA_MASK            (1<<PC1)            /* SDA bit-mask e.g. bit0 */

/* Port macros                     ***** MODIFY TO SUIT ***** */
/* Data-direction register for I2C pins */
// THIERRY #define I2CDDR      DDRC
/* Data pin register for I2C pins */
// THIERRY #define I2CPIN      PINC
/* Data port register for I2C pins */
// THIERRY #define I2CPORT     PORTC

/* bit manipulation macros - assume DDR bit is set for output */
/* wait for Scl pin to float high - it may be held low by slave */
// THIERRY #define wait_scl_high()     while (!(I2CPIN & SCL_MASK))
#define wait_scl_high()     while (!SCL)
#define wait_sda_high()     while (!SDA)
/* get state of Scl pin */
// THIERRY #define read_scl()          (I2CPORT & SCL_MASK)                            /* drives SCL pin (low) */
#define read_scl()          (SCL)
/* get state of Sda pin */
// THIERRY #define read_sda()          (I2CPIN & SDA_MASK)                         /* drives SCL pin (low) */
#define read_sda()          (SDA)
/* drive Scl pin low */
// THIERRY #define scl_low()           (I2CPORT &= ~SCL_MASK , I2CDDR |= SCL_MASK )        /* drives SCL pin (low) */
#define scl_low()           (SCL = 0 , SCLD = 1)
/* float Scl pin (set to input mode) */
// THIERRY #define scl_high()          (I2CPORT |= SCL_MASK , I2CDDR &= ~SCL_MASK) /* drives SCL pin (low) */
#define scl_high()          (SCLD = 0)
/* drive Sda pin low */
// THIERRY #define sda_low()           (I2CPORT &= ~SDA_MASK , I2CDDR |= SDA_MASK)     /* drives SDA pin (low) */
#define sda_low()           (SDA = 0 , SDAD = 1)
/* float Sda pin (set to input mode) */
// THIERRY #define sda_high()          (I2CPORT |= SDA_MASK, I2CDDR &= ~SDA_MASK ) /* drives SCL pin (low) */
#define sda_high()          (SDAD = 0)
#define sda_high1()          (SDA = 1 , SDAD = 1)

//int twi_master_read(const twi_package_t *package);
//int twi_master_write( const twi_package_t *package);
/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

void InitI2c ( void )
{
    uint8_t i;

    /************************ PLATFORM SPECIFIC *****************************/
    /* Configure I/O ports corresponding to SCL, SDA and CHANGE/ pins on target processor */
    /* SCL:     set to input mode, set to drive low if output */
    /* SDA:     set to input mode  set to drive low if output */
    /* CHANGE/: set to input mode */

    //I2CDDR |= SCL_MASK ;
    //I2CPORT &= ~SCL_MASK ;

    //I2CDDR |= SDA_MASK ;
    //I2CPORT &= ~SDA_MASK ;

    /************************ PLATFORM SPECIFIC *****************************/
    /* set SCL and SDA into correct idle state */
    scl_high();
    wait_scl_high();
    sda_high();

    /* ensure any connected device in read-mode is disconnected from the bus */
    for (i = 0; i < 10; i++)
    {
        scl_low();
        scl_high();
        wait_scl_high();
    }
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

void I2cStart()
{

    sda_high();
    scl_high();
    wait_scl_high();
    sda_low();
    scl_low();
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

void I2cStop()
{
    sda_low();
    scl_high();
    wait_scl_high();
    sda_high();
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint8_t I2cTxByte(uint8_t TxData)
{
    uint8_t i;
    uint8_t RetVal;
    uint8_t t = TxData;

    for (i= 0;i < 8;i++)
    {
        if (t & 0x80)
            sda_high();
        else
            sda_low();
        t <<= 1;
        scl_high();
        wait_scl_high();
        scl_low();
    }

    sda_high();
    scl_high();
    wait_scl_high();
    if (read_sda())
        RetVal = 0;
    else
        RetVal = 1;
    scl_low();
    //wait_sda_high();

    return RetVal;
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint8_t I2cRxByte( uint8_t AckState )
{
    uint8_t i;
    uint8_t r = 0;

    sda_high();
    for (i= 0;i < 8;i++)
    {
        r <<= 1;
        scl_high();
        wait_scl_high();
        if (read_sda())
            r |= 1;
        scl_low();
    }

    if (!AckState)
        sda_low();
    scl_high();
    wait_scl_high();
    scl_low();

    return r;
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint8_t i2c_master_write(i2c_package_t  *i2c_cmd)
{
    unsigned char i2c_addr;
    unsigned short i;

    uint8_t Status = WRITE_MEM_OK;


    i2c_addr = (i2c_cmd->slave_addr *2) + I2C_WRITE_FLAG;

    //I2C_START
    I2cStart();

    //SLA + W
    if ( !I2cTxByte(i2c_addr))
    {
        I2cStop();
        return WRITE_MEM_FAILED;
    }

    //TX Buffer
    for(i = 0; i < i2c_cmd->txlen ; i++)
    {
        if ( !I2cTxByte(*i2c_cmd->txbuf++))
        {
            I2cStop();
            return WRITE_MEM_FAILED;
        }
    }

    //I2C_STOP
    I2cStop();

    return (Status);
}

/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint8_t i2c_master_read(i2c_package_t  *i2c_cmd)
{
    unsigned char i2c_addr;
    unsigned short i;

    uint8_t Status = READ_MEM_OK;


    i2c_addr = (i2c_cmd->slave_addr *2) + I2C_READ_FLAG;

    //I2C_START
    I2cStart();

    //SLA + R
    if ( !I2cTxByte(i2c_addr))
    {
        I2cStop();
        return READ_MEM_FAILED;
    }

    //READ Buffer
    for(i = 0; i < i2c_cmd->rxlen ; i++)
    {
        *i2c_cmd->rxbuf++ = I2cRxByte(i == (i2c_cmd->rxlen - 1) ? 1 : 0);
    }

    //I2C_STOP
    I2cStop();

    return (Status);
}


/*****************************************************************************
*  FUNCTION  存储器写操作
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/
//存储器写操作

uint8_t write_mem( uint16_t Address, uint8_t ByteCount, uint8_t *Data )
{
    uint8_t i;
    uint8_t status;
    static uint8_t *txtmp;
    static uint8_t *rxtmp;

    i2c_package_t packet;

    address_pointer = Address;

    txtmp = malloc(2 + ByteCount);
    if (txtmp == 0)
    {
        return(WRITE_MEM_FAILED);
    }
    memset(txtmp,0,(2 + ByteCount));

    *txtmp = (uint8_t)(Address & 0xFF);
    *(txtmp + 1) = (uint8_t) (Address >> 8);

    memcpy((txtmp + 2), Data, ByteCount);

    packet.slave_addr = QT_i2c_address;
    packet.txbuf = (void*) txtmp;
    packet.txlen = (2 + ByteCount);
    packet.rxbuf = (void*) rxtmp;
    packet.rxlen = 0;

    status = i2c_master_write(&packet);

    /* Try the write several times if unsuccessful at first. */
    i = 0;
    while ((status == WRITE_MEM_FAILED) & (i < 10))
    {
        status = i2c_master_write(&packet);
        i++;
    }


    free(txtmp);

    return status;
}


/*****************************************************************************
*  FUNCTION 存储器读操作
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/
//存储器读操作

uint8_t read_mem( uint16_t Address, uint8_t ByteCount, uint8_t *Data )
{
    static unsigned char first_read = 1;
    uint8_t i;
    uint8_t status;
    static uint8_t *txtmp;
    static uint8_t *rxtmp;

    i2c_package_t packet;

    if (first_read)
    {
      /* Make sure that when making the first read, address pointer
       * is always written. */
        address_pointer = Address + 1;
        first_read = 0;
    }
   
    if ((address_pointer != Address) || (Address != message_processor_address))
    {

        address_pointer = Address;
        Address = le16_to_cpup(Address);

        packet.slave_addr = QT_i2c_address;
        packet.txbuf = (void*) &Address;
        packet.txlen = 2;
        packet.rxbuf = (void*) rxtmp;
        packet.rxlen = 0;

        status = i2c_master_write(&packet);

        
        if (status != WRITE_MEM_OK)
        {
          return(READ_MEM_FAILED);
        }

    }
        
    packet.slave_addr = QT_i2c_address;
    packet.txbuf = (void*) txtmp;
    packet.txlen = 0;
    packet.rxbuf = (void*) Data;
    packet.rxlen = ByteCount;

    status = i2c_master_read(&packet);
    i = 0;
    while ((status == READ_MEM_FAILED) & (i < 10))
    {
        status = i2c_master_read(&packet);
        i++;
    }

    I2cStop();
    return READ_MEM_OK;
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint8_t read_uint16_t( uint16_t Address, uint16_t *Data )
{
    uint8_t Status;

    Status = read_mem(Address, 2, (uint8_t *)Data); // try reading 2 bytes
    // format result
    *Data = le16_to_cpup(*Data);

    return Status;
}





/*****************************************************************************
*  FUNCTION 从机QT Controller 存储器的地址
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/
//从机QT Controller 存储器的地址

uint8_t address_slave(void)
{
    uint8_t i;
    uint8_t status;
    uint8_t txtmp;
    uint8_t rxtmp;

    i2c_package_t packet;

    packet.slave_addr = QT_i2c_address;
    packet.txbuf = (void*) &txtmp;
    packet.txlen = 0;
    packet.rxbuf = (void*) &rxtmp;
    packet.rxlen = 1;

    status = i2c_master_read(&packet);

    if(status == READ_MEM_OK)
    {
        return (CONNECT_OK);
    }
    else
    {
        i = 0;
        while ((status == READ_MEM_FAILED) & (i < 10))
        {
            status = i2c_master_read(&packet);
            if(status == READ_MEM_OK)
            {
                return (CONNECT_OK);
            }
            i++;
        }
    }
    return CONNECT_ERROR;
}




#define __QT_DRIVER_BLOCK___
/******************************************************************************
*       QT Controller Driver Block
*****************************************************************************/

// THIERRY #define CHANGELINE_MASK     (1<<PD2)            /* CHG/ bit-mask e.g. bit 0 */

/* Data port register for CHANGE/ pin */
// THIERRY #define CHGPORT     PORTD
/* Data port register for CHANGE/ pin */
// THIERRY #define CHGPIN      PIND
/* Data-direction register for CHANGE pins */
// THIERRY #define CHGDDR      DDRD

// THIERRY #define change_asserted()  ((CHGPIN & CHANGELINE_MASK) ? CHANGELINE_ASSERTED : CHANGELINE_NEGATED)

/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/
//QT Controller 信号输出判断
// THIERRY uint8_t ChangeLineStatus( void )
// THIERRY {
// THIERRY     return change_asserted();
// THIERRY }  


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/
// QT Controller 应用初始化驱动程序，information block 读出
// ID information,Object Table Element1-n, Object1-n
uint8_t init_touch_driver(uint8_t I2C_address, void (*handler)(uint8_t *, uint8_t))
{
    uint16_t i;
    uint8_t tmp;
    uint16_t current_address;
    uint16_t crc_address;

    info_id_t *id;
    object_t *object_table;
    uint32_t *CRC;
    uint8_t status;
    int current_report_id = 0;


    /* Set the message handler function pointer. */
//    application_message_handler = handler;


    /* save new slave address */
    QT_i2c_address = I2C_address;

    /* Poll device */
    status = address_slave();

    if(status == CONNECT_OK)
    {
//        QT_printf("[TSP] I2C Slave Address = 0x%x\n\r",QT_i2c_address);
      asm("nop");
    }
    else
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }


    driver_setup = DRIVER_SETUP_INCOMPLETE;

    /* Read the info block data. */

    id = (info_id_t *) malloc(sizeof(info_id_t));
    if (id == 0)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }

//num_declared_objects =0x11,is Number of Object Table Elements    
    if (read_id_block(id) != 1)  
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }


    /* Read object table. */
    object_table = (object_t *) malloc(id->num_declared_objects * sizeof(object_t));
    if (object_table == 0)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }     
    memset(object_table,0,sizeof(id->num_declared_objects * sizeof(object_t)));

    /* Reading the whole object table block to memory directly doesn't work cause sizeof object_t
    isn't necessarily the same on every compiler/platform due to alignment issues. Endianness
    can also cause trouble. */

    current_address = OBJECT_TABLE_START_ADDRESS;

    max_report_id = 0;
    for (i = 0; i < id->num_declared_objects; i++)  //num_declared_objects =0x11
    {
    	  object_t *obj =  &object_table[i];
        status = read_mem(current_address, 1, &object_table[i].object_type);
        if (status != READ_MEM_OK)
        {
            return(DRIVER_SETUP_INCOMPLETE);
        }
        current_address++;
        
        status = read_uint16_t(current_address, &object_table[i].i2c_address);
        if (status != READ_MEM_OK)
        {
            return(DRIVER_SETUP_INCOMPLETE);
        }
        current_address += 2;
        
        status = read_mem(current_address, 1, &object_table[i].size);
        if (status != READ_MEM_OK)
        {
            return(DRIVER_SETUP_INCOMPLETE);
        }
        current_address++;
        
        status = read_mem(current_address, 1, &object_table[i].instances);
        if (status != READ_MEM_OK)
        {
            return(DRIVER_SETUP_INCOMPLETE);
        }
        current_address++;

        status = read_mem(current_address, 1, &object_table[i].num_report_ids);
        if (status != READ_MEM_OK)
        {
            return(DRIVER_SETUP_INCOMPLETE);
        }
        current_address++;

        max_report_id += object_table[i].num_report_ids;

        /* Find out the maximum message length. */
        if (object_table[i].object_type == GEN_MESSAGEPROCESSOR_T5)
        {
            max_message_length = object_table[i].size;   //wwwwwwwwwwwwwwwwww
        }

    }

    /* Check that message processor was found. */
    if (max_message_length == 0)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }

    /* Read CRC. */

    CRC = (uint32_t *) malloc(sizeof(info_id_t));
    if (CRC == 0)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }  


    info_block = malloc(sizeof(info_block_t));
    if (info_block == 0)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }  

    info_block->info_id = *id;

    info_block->objects = object_table;

 //crc_address: Information block checksum address
    crc_address = OBJECT_TABLE_START_ADDRESS +
    id->num_declared_objects * OBJECT_TABLE_ELEMENT_SIZE;  

    status = read_mem(crc_address, 1u, &tmp);
    if (status != READ_MEM_OK)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }
    info_block->CRC = tmp;

    status = read_mem(crc_address + 1u, 1u, &tmp);
    if (status != READ_MEM_OK)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }
    info_block->CRC |= (tmp << 8u);

    status = read_mem(crc_address + 2, 1, &info_block->CRC_hi);
    if (status != READ_MEM_OK)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }

    /* Store message processor address, it is needed often on message reads. */
 // message_processor_address =0x00f2
    message_processor_address = get_object_address(GEN_MESSAGEPROCESSOR_T5, 0);
    if (message_processor_address == 0)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    } 

    // Store command processor address. 
    command_processor_address = get_object_address(GEN_COMMANDPROCESSOR_T6, 0);
    if (command_processor_address == 0)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }  
  
    msg = malloc(max_message_length);
    if (msg == 0)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }  

    /* Allocate memory for report id map now that the number of report id's
    * is known. */
     report_id_map = malloc(sizeof(report_id_map_t) * max_report_id + 1);
    if (report_id_map == 0)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }  

    /* Report ID 0 is reserved, so start from 1. */

    report_id_map[0].instance = 0;
    report_id_map[0].object_type = 0;
    current_report_id = 1;

    for (i = 0; i < id->num_declared_objects; i++)
    {
        if (object_table[i].num_report_ids != 0)
        {
            int instance;
            for (instance = 0; instance <= object_table[i].instances; instance++)
            {
                int start_report_id = current_report_id;
                for (; current_report_id < (start_report_id + object_table[i].num_report_ids); current_report_id++)
                {
                    report_id_map[current_report_id].instance = instance;
                    report_id_map[current_report_id].object_type =
                    object_table[i].object_type;
                }
            }
        }
    }

    driver_setup = DRIVER_SETUP_OK;

    return(DRIVER_SETUP_OK);

}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint8_t close_touch_driver()
{
    free(info_block);
    free(report_id_map);
    free(msg);

    /* Do platform specific clean-up, interrupt disable etc. */
    //   platform_close();

    return(0);
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint8_t get_version(uint8_t *version)
{

    if (info_block)
    {
        *version = info_block->info_id.version;
    }
    else
    {
        return(ID_DATA_NOT_AVAILABLE);
    }
    return (ID_DATA_OK);
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint8_t get_family_id(uint8_t *family_id)
{

    if (info_block)
    {
        *family_id = info_block->info_id.family_id;
    }
    else
    {
        return(ID_DATA_NOT_AVAILABLE);
    }
    return (ID_DATA_OK);
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint8_t get_build_number(uint8_t *build)
{

    if (info_block)
    {
        *build = info_block->info_id.build;
    }
    else
    {
        return(ID_DATA_NOT_AVAILABLE);
    }
    return (ID_DATA_OK);
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint8_t get_variant_id(uint8_t *variant)
{

    if (info_block)
    {
        *variant = info_block->info_id.variant_id;
    }
    else
    {
        return(ID_DATA_NOT_AVAILABLE);
    }
    return (ID_DATA_OK);
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint8_t get_object_size(uint8_t object_type)
{
    uint8_t object_table_index = 0;
    uint8_t object_found = 0;
    uint16_t size = OBJECT_NOT_FOUND;

    object_t *object_table;
    object_t obj;
    object_table = info_block->objects;
    while ((object_table_index < info_block->info_id.num_declared_objects) &&
        !object_found)
    {
        obj = object_table[object_table_index];
        /* Does object type match? */
        if (obj.object_type == object_type)
        {
            object_found = 1;
            size = obj.size + 1;
        }
        object_table_index++;
    }

    return(size);
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint8_t type_to_report_id(uint8_t object_type, uint8_t instance)
{

    uint8_t report_id = 1;
    int8_t report_id_found = 0;

    while((report_id <= max_report_id) && (report_id_found == 0))
    {
        if((report_id_map[report_id].object_type == object_type) &&
            (report_id_map[report_id].instance == instance))
        {
            report_id_found = 1;
        }
        else
        {
            report_id++;
        }
    }
    if (report_id_found)
    {
        return(report_id);
    }
    else
    {
        return(ID_MAPPING_FAILED);
    }
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint8_t report_id_to_type(uint8_t report_id, uint8_t *instance)
{

    if (report_id <= max_report_id)
    {
        *instance = report_id_map[report_id].instance;
        return(report_id_map[report_id].object_type);
    }
    else
    {
        return(ID_MAPPING_FAILED);
    }

}

/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/
//QT Controller运用初始化，读信息块
uint8_t read_id_block(info_id_t *id)
{
    uint8_t status;

    status = read_mem(0, 1, (void *) &id->family_id);
    if (status != READ_MEM_OK)
    {
        return(status);
    }

    status = read_mem(1, 1, (void *) &id->variant_id);
    if (status != READ_MEM_OK)
    {
        return(status);
    }

    status = read_mem(2, 1, (void *) &id->version);
    if (status != READ_MEM_OK)
    {
        return(status);
    }

    status = read_mem(3, 1, (void *) &id->build);
    if (status != READ_MEM_OK)
    {
        return(status);
    }

    status = read_mem(4, 1, (void *) &id->matrix_x_size);
    if (status != READ_MEM_OK)
    {
        return(status);
    }

    status = read_mem(5, 1, (void *) &id->matrix_y_size);
    if (status != READ_MEM_OK)
    {
        return(status);
    }

    status = read_mem(6, 1, (void *) &id->num_declared_objects);

    return(status);

}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint8_t get_max_report_id()
{
    return(max_report_id);
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint16_t get_object_address(uint8_t object_type, uint8_t instance)
{
    uint8_t object_table_index = 0;
    uint8_t address_found = 0;
    uint16_t address = OBJECT_NOT_FOUND;

    object_t *object_table;
    object_t obj;
    object_table = info_block->objects;
    while ((object_table_index < info_block->info_id.num_declared_objects) &&
        !address_found)
    {
        obj = object_table[object_table_index];
        /* Does object type match? */
        if (obj.object_type == object_type)
        {

            address_found = 1;

            /* Are there enough instances defined in the FW? */
            if (obj.instances >= instance)
            {
                address = obj.i2c_address + (obj.size + 1) * instance;
            }
        }
        object_table_index++;
    }

    return(address);
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint32_t get_stored_infoblock_crc()
{
    uint32_t crc;
    crc = (uint32_t) (((uint32_t) info_block->CRC_hi) << 16);
    crc = crc | info_block->CRC;
    return(crc);
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint8_t calculate_infoblock_crc(uint32_t *crc_pointer)
{

    uint32_t crc = 0;
    uint16_t crc_area_size;
    uint8_t *mem;

    uint8_t i;

    uint8_t status;

    /* 7 bytes of version data, 6 * NUM_OF_OBJECTS bytes of object table. */
    crc_area_size = 7 + info_block->info_id.num_declared_objects * 6;

    mem = (uint8_t *) malloc(crc_area_size);
    if (mem == 0)
    {
        return(CRC_CALCULATION_FAILED);
    }

    status = read_mem(0, crc_area_size, mem);

    if (status != READ_MEM_OK)
    {
        return(CRC_CALCULATION_FAILED);
    }

    i = 0;
    while (i < (crc_area_size - 1))
    {
        crc = CRC_24(crc, *(mem + i), *(mem + i + 1));
        i += 2;
    }

    crc = CRC_24(crc, *(mem + i), 0);

    free(mem);

    /* Return only 24 bit CRC. */
    *crc_pointer = (crc & 0x00FFFFFF);
    return(CRC_CALCULATION_OK);

}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

uint32_t CRC_24(uint32_t crc, uint8_t byte1, uint8_t byte2)
{
    static const uint32_t crcpoly = 0x80001B;
    uint32_t result;
    uint16_t data_word;

    data_word = (uint16_t) ((uint16_t) (byte2 << 8u) | byte1);
    result = ((crc << 1u) ^ (uint32_t) data_word);

    if (result & 0x1000000)
    {
        result ^= crcpoly;
    }

    return(result);

}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

void get_message(void)
{
//    unsigned char num_of_touch;
//    unsigned char max_touch;

//    max_touch = touchscreen_config.numtouch;

    x_pos = 0;
    y_pos = 0;


    if (driver_setup == DRIVER_SETUP_OK)
    {

        if(read_mem(message_processor_address, max_message_length, tmsg) == READ_MEM_OK)
        {
    
            if (report_id_map[tmsg[0]].object_type == TOUCH_MULTITOUCHSCREEN_T9)
            {
                x_pos = ((tmsg[2])<<2)+((tmsg[4] & 0xC0) >>6);
                y_pos = ((tmsg[3])<<2)+((tmsg[4] & 0x0C)>>2);
            }
            else if (report_id_map[tmsg[0]].object_type == TOUCH_KEYARRAY_T15)
            {
              asm("nop");
            }
            else if (report_id_map[tmsg[0]].object_type ==TOUCH_SINGLETOUCHSCREEN_T10 )
            {}
            else if(tmsg[0] == 0xFF)    // invalid message
            {
            }

        }
    }
}

int8_t get_t9_message()
{   
	  int8_t result = 0;
	  
    if(driver_setup != DRIVER_SETUP_OK)
        return -1;
    
    do{
        
        if(read_mem(message_processor_address, max_message_length, tmsg) == READ_MEM_OK)
        {
            if (report_id_map[tmsg[0]].object_type == TOUCH_MULTITOUCHSCREEN_T9)
            {
                uint8_t status;
                uint16_t x;
                uint16_t y;
            
                status = tmsg[1];
                x = (tmsg[2] << 4) | ((tmsg[4] >> 4) & 0xf);
                y = (tmsg[3] << 4) | ((tmsg[4] & 0xf));
        
                x >>= 2;
                y >>= 2;
        				s_t9sts = status;
        
                if (status & MXT_T9_DETECT) {
                    /* Multiple bits may be set if the host is slow to read the
                     * status messages, indicating all the events that have
                     * happened */
                     
                    if (status & MXT_T9_RELEASE) {
                        
                    }
                
                    /* Touch active */
                    x_pos = x;
                    y_pos = y;

                } else {
                    /* Touch no longer active, close out slot */
                    
                }
                result = 1;
            }
            else if (report_id_map[tmsg[0]].object_type == TOUCH_KEYARRAY_T15)
            {
                asm("nop");
            }
            else if (report_id_map[tmsg[0]].object_type ==TOUCH_SINGLETOUCHSCREEN_T10 )
            {
            	  asm("nop");
            }
            else if(tmsg[0] == 0xFF)    // invalid message
            {

            }else{

            }
        }
    }while(tmsg[0] != 0xff && !result);
    
    return result? 0 : -1;
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

void message_handler(uint8_t *msg, uint8_t length)
{
    if (report_id_map[tmsg[0]].object_type == TOUCH_MULTITOUCHSCREEN_T9)
    {
      x_pos = tmsg[2] << 2 + (tmsg[4] & 0xC0) >> 6;
      y_pos = tmsg[3] << 2 + (tmsg[4] & 0x0C) >> 2;
    }
    else if (report_id_map[tmsg[0]].object_type == TOUCH_KEYARRAY_T15)
    {}
    else if (report_id_map[tmsg[0]].object_type ==TOUCH_SINGLETOUCHSCREEN_T10 )
    {}
    else if (report_id_map[tmsg[0]].object_type ==PROCI_ONETOUCHGESTUREPROCESSOR_T24 )
    {}
    else if (report_id_map[tmsg[0]].object_type ==PROCI_TWOTOUCHGESTUREPROCESSOR_T27 )
    {}
    else if(tmsg[0] == 0xFF)    // invalid message
    {}
    
    /*    PlotTouch( msg );  */ /* add code to further process touch messages */
}


/*****************************************************************************
*  FUNCTION  对 QT Controller 应用进行初始化
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/
// 对 QT Controller 应用进行初始化，I2C连接并找到QT Controller
// 内存分配，读QT Controller的 Information Block,Object Table Element1-n,Object1-n
uint8_t init_touch_app(void)
{
    uint8_t return_val = true;
    uint8_t touch_chip_found = 0;
    uint8_t report_id;
    uint8_t max_report_id;
    uint8_t instance;
    uint8_t object_type;
    uint16_t i;
    uint8_t version, family_id, variant, build;
    
    // Try to initialize the driver. 
    for (i = 0; i < NUM_OF_I2C_ADDR; i++)
    {
        if((init_touch_driver(i2c_addresses[i],  &message_handler)) == DRIVER_SETUP_OK)
        {
            touch_chip_found = 1;
            break;
        }  
    }


    if (touch_chip_found == 0)  //I2C 通信正常并且有QT Controller
    {
        return_val = false;
    }
    else   
    {    // only continue here if chip is identified 

        // Get and show the version information. 
        get_family_id(&family_id);
        get_variant_id(&variant);
        get_version(&version);
        get_build_number(&build);

        /* Test the report id to object type / instance mapping: get the maximum
        * report id and print the report id map. */
        
        max_report_id = get_max_report_id();
        for (report_id = 1; report_id <= max_report_id; report_id++)
        {
            object_type = report_id_to_type(report_id, &instance);
        }

        // Calibrate the touch IC. 
    }   

    return return_val;
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
* ***************************************************************************/

/*void main( void )
{
//--------- 增加对 QT Controller 复位 --------------------
// THIERRY    asm("nop");
// THIERRY    delay_ms(30);     //delay 30ms 
// THIERRY    PORTD |=0x08;    //pd3 high 
// THIERRY    DDRD  |=0x08;   //pd3 output 
// THIERRY    PORTD &=~0x08;    //pd3 low
// THIERRY    delay_ms(1);      //delay 1ms
// THIERRY    PORTD |=0x08;    //pd3 high
// THIERRY    delay_ms(100);   //delay 100ms
      
//-------------------------------------------------
       
// THIERRY#if defined(__TWI__)
// THIERRY    InitTwi();  //I2C 接口初始化
// THIERRY#else
    InitI2c();
// THIERRY#endif
    
// THIERRY    CHGPORT |= CHANGELINE_MASK;
// THIERRY    CHGDDR &= ~CHANGELINE_MASK;
   
    chip_detected_flag = init_touch_app();  // find and initialise QT device 

    if(chip_detected_flag == true)
    {
        uint8_t version, family_id, variant, build;
        asm("nop");
      
        // Get and show the version information. 
        get_family_id(&family_id);
        get_variant_id(&variant);
        get_version(&version);
        get_build_number(&build);
 // THIERRY       while(ChangeLineStatus() == CHANGELINE_NEGATED)
        while(Touch_Int == 0) 
        {    
            get_message();
            asm("nop"); 
        }  
    }   


//------- 用CHG 中断的方式读取信号 --------------------          
// THIERRY    ExtIntConfigure();   //int1 low level interrupt, 中断方式接受

    // Global Interrupt Enable 
// THIERRY    __enable_interrupt();
//-----------------------------------------------------        

    for(;;)  // loop forever... 
    {
      asm("nop"); 
      asm("nop"); 
      asm("nop"); 

      
//------- 用查询CHG 的方式读取信号 --------------------      
        if(chip_detected_flag == true)
        {  
// THIERRY            if(ChangeLineStatus() == CHANGELINE_ASSERTED)
					if(Touch_Int == 0)
            {
                get_message();
            }  
        
        }  
//-----------------------------------------------------          
   
    }
}*/


