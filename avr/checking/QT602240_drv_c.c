/*****************************************************************************
*       ATmel mXT224 Driver
* ***************************************************************************/

// I2C device address of mXT224
// ADDR_SEL=0 ==> 0x4A
// ADDR_SEL=1 ==> 0x4B

//#define QT_GLOBALS

// use TWI module instead of I/O simulation
#define __TWI__

#define __VER_1_4__
#define __BUILD_0xAA__  

// requirements to mega32 as host:
// CLIB heap size =0x100
// System: CSTACK=0x40  ,RSTACK=0x30

/*****************************************************************************
*   Build Options
* ***************************************************************************/
// #define OPTION_WRITE_CONFIG     /* uncomment to force chip setup at startup */
// #define OPTION_PRINT_MESSAGES   /* uncomment to display touch messages via UART */

/*****************************************************************************
*   Include Files
* ***************************************************************************/
#include "QT602240_drv_h.h"
#include <ioavr.h>
#include <intrinsics.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define QT_printf(...)
//#define QT_printf printf
/*
// if need other ways to do printf, like UART, modify func of putchar
int putchar(int i)
{
  return 1;
}*/

// INT0 interrupt: read message
#pragma vector=INT0_vect
__interrupt static void INT0_ISR(void)
{
  get_message();
}

// INT0 initialization
void ExtIntConfigure(void)
{
  MCUCR = 0x02;   //the falling edge of int0 generates an interrupt
  GIFR  = 0x40;   //clear int0 interrupt flag
  GICR  = 0x40;   //enable int0 interrupt
}

//UART initialize
// UBRR0 =Fosc/(16*BAUD)-1,  //U2X0=0
// BAUD =Fosc/(16*UBRR0+1),  //U2X0=0
// desired baud rate: 38.4K
// actual: baud rate:38.4K(0.2%)
// char size: 8 bit
// parity: Disabled
void USART0_Init(uint8_t baud_set)
{
  UCSRA = 0x00;  
  UCSRC = (1<<URSEL)|(1<<USBS)|(1<<UCSZ1)|(1<<UCSZ0);
  UBRRH = 0x00;
  UBRRL = baud_set;
  UCSRB = (1<<TXEN);
}

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
*       QT602240 Object table init
* *****************************************************************************/
//General Object
gen_powerconfig_t7_config_t power_config = {0};                 //Power config settings.
gen_acquisitionconfig_t8_config_t acquisition_config = {0};     // Acquisition config. 

//Touch Object
touch_multitouchscreen_t9_config_t touchscreen_config = {0};    //Multitouch screen config.
touch_keyarray_t15_config_t keyarray_config = {0};              //Key array config.
touch_proximity_t23_config_t proximity_config = {0};        //Proximity config. 

//Signal Processing Objects
proci_gripfacesuppression_t20_config_t gripfacesuppression_config = {0};    //Grip / face suppression config.
procg_noisesuppression_t22_config_t noise_suppression_config = {0};         //Noise suppression config.
proci_onetouchgestureprocessor_t24_config_t onetouch_gesture_config = {0};  //One-touch gesture config. 
proci_twotouchgestureprocessor_t27_config_t twotouch_gesture_config = {0};  //Two-touch gesture config.

//Support Objects
spt_gpiopwm_t19_config_t  gpiopwm_config = {0};             //GPIO/PWM config
spt_selftest_t25_config_t selftest_config = {0};            //Selftest config.
spt_cteconfig_t28_config_t cte_config = {0};                //Capacitive touch engine config.

spt_comcconfig_t18_config_t   comc_config = {0};            //Communication config settings.

void delay_ms(unsigned short dly)
{
    while(dly--)
    {
        __delay_cycles(FOSC/1000);   //delay 1ms
    }
}

#define __QT_I2C_BLOCK___
/*****************************************************************************
*       I2C Blocks
* ***************************************************************************/

/* I2c read/write flags */
#define I2C_WRITE_FLAG	0x00	/* bit 0 of slave-addres byte */
#define I2C_READ_FLAG	0x01

/* Retry times on NACK */
#define NACK_RETRY_MAX	10

typedef struct
{
    uint8_t  slave_addr;
    uint8_t  *txbuf;
    uint16_t txlen;
    uint8_t  *rxbuf;
    uint16_t rxlen;
}i2c_package_t;

typedef struct
{
  char chip;
  unsigned int addr;
  int addr_length;
  void *buffer;
  unsigned int length;
} twi_package_t;

/* Ports definitions               ***** MODIFY TO SUIT *****  */
/* Set these defines to specify used port bits  */
#define SCL_MASK            (1<<PC0)            /* SCL bit-mask e.g. bit1 */
#define SDA_MASK            (1<<PC1)            /* SDA bit-mask e.g. bit0 */

/* Port macros                     ***** MODIFY TO SUIT ***** */
/* Data-direction register for I2C pins */
#define I2CDDR      DDRC
/* Data pin register for I2C pins */
#define I2CPIN      PINC
/* Data port register for I2C pins */
#define I2CPORT     PORTC

/* bit manipulation macros - assume DDR bit is set for output */
/* wait for Scl pin to float high - it may be held low by slave */
#define wait_scl_high()     while (!(I2CPIN & SCL_MASK))
/* get state of Scl pin */
#define read_scl()          (I2CPORT & SCL_MASK)                            /* drives SCL pin (low) */
/* get state of Sda pin */
#define read_sda()          (I2CPIN & SDA_MASK)                         /* drives SCL pin (low) */
/* drive Scl pin low */
#define scl_low()           (I2CPORT &= ~SCL_MASK , I2CDDR |= SCL_MASK )        /* drives SCL pin (low) */
/* float Scl pin (set to input mode) */
#define scl_high()          (I2CPORT |= SCL_MASK , I2CDDR &= ~SCL_MASK) /* drives SCL pin (low) */
/* drive Sda pin low */
#define sda_low()           (I2CPORT &= ~SDA_MASK , I2CDDR |= SDA_MASK)     /* drives SDA pin (low) */
/* float Sda pin (set to input mode) */
#define sda_high()          (I2CPORT |= SDA_MASK, I2CDDR &= ~SDA_MASK ) /* drives SCL pin (low) */


int twi_master_read(const twi_package_t *package);
int twi_master_write( const twi_package_t *package);

void InitI2c ( void )
{
    uint8_t i;

    /************************ PLATFORM SPECIFIC *****************************/
    /* Configure I/O ports corresponding to SCL, SDA and CHANGE/ pins on target processor */
    /* SCL:     set to input mode, set to drive low if output */
    /* SDA:     set to input mode  set to drive low if output */
    /* CHANGE/: set to input mode */

    I2CDDR |= SCL_MASK ;
    I2CPORT &= ~SCL_MASK ;

    I2CDDR |= SDA_MASK ;
    I2CPORT &= ~SDA_MASK ;

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

void I2cStart()
{
    sda_high();
    scl_high();
    wait_scl_high();
    sda_low();
    scl_low();
}

void I2cStop()
{
    sda_low();
    scl_high();
    wait_scl_high();
    sda_high();
}

uint8_t I2cTxByte( uint8_t TxData)
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

    return RetVal;
}

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

uint8_t i2c_master_write(i2c_package_t  *i2c_cmd)
{
    unsigned char i2c_addr;
    unsigned short i;

    uint8_t Status = WRITE_MEM_OK;
    i2c_addr = (i2c_cmd->slave_addr *2) + I2C_WRITE_FLAG;
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

    I2cStop();

    return (Status);
}

uint8_t i2c_master_read(i2c_package_t  *i2c_cmd)
{
    unsigned char i2c_addr;
    unsigned short i;

    uint8_t Status = READ_MEM_OK;
    i2c_addr = (i2c_cmd->slave_addr *2) + I2C_READ_FLAG;
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

    I2cStop();

    return (Status);
}


#if defined(__TWI__)
/*! \brief Maxtouch Memory write by I2C bus */
uint8_t write_mem( uint16_t Address, uint8_t ByteCount, uint8_t *Data )
{
   uint8_t status = 0;
   uint8_t i = 0;
   uint8_t *tmp;
   twi_package_t packet;

   address_pointer = Address;

   tmp = malloc(2 + ByteCount);
   if (tmp == NULL)
   {
      return(WRITE_MEM_FAILED);
   }
   /* Swap start address nibbles since MSB is first but touch IC wants LSB first. */
   *tmp = (uint8_t) (Address & 0xFF);
   *(tmp + 1) = (uint8_t) (Address >> 8);
   memcpy((tmp + 2), Data, ByteCount);

   /* TWI chip address to communicate with. */
   packet.chip = QT_i2c_address;
   /* TWI address/commands to issue to the other chip (node). */
   packet.addr = 0;
   /* Length of the TWI data address segment (1-3 bytes). */
   packet.addr_length = 0;
   /* Where to find the data to be written. */
   packet.buffer = (void*) tmp;
   /* How many bytes do we want to write. */
   packet.length = 2 + ByteCount;

   status = twi_master_write(&packet);

   /* Try the write several times if unsuccessful at first. */
   i = 0;
   while ((status != WRITE_MEM_OK) & (i < 10))
   {
      status = twi_master_write(&packet);
      i++;
   }

   free(tmp);

   if (status != WRITE_MEM_OK)
   {
      return(WRITE_MEM_FAILED);
   }

   return(WRITE_MEM_OK);
}

#else
uint8_t write_mem( uint16_t Address, uint8_t ByteCount, uint8_t *Data )
{
    uint8_t i;
    uint8_t status;
    static uint8_t *txtmp;
    static uint8_t *rxtmp;

    i2c_package_t packet;

    address_pointer = Address;

    txtmp = malloc(2 + ByteCount);
    if (txtmp == NULL)
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
#endif

#if defined(__TWI__)
uint8_t read_mem( uint16_t Address, uint8_t ByteCount, uint8_t *Data )
{
   static unsigned char first_read = 1;

   int status = 0;
   twi_package_t packet, packet_received;

   if (first_read){
      /* Make sure that when making the first read, address pointer
       * is always written. */
      address_pointer = Address + 1;
      first_read = 0;
   }
   
   /* Set the read address pointer, if needed - consecutive reads from
    * message processor address can be done without updating the pointer. */
   if ((address_pointer != Address) || (Address != message_processor_address))
   {
      address_pointer = Address;

      /* TWI chip address to communicate with. */
      packet.chip = QT_i2c_address;
      /* TWI address/commands to issue to the other chip (node). */
      packet.addr = 0;
      /* Length of the TWI data address segment (1-3 bytes). */
      packet.addr_length = 0;
      /* Where to find the data to be written */
      packet.buffer = (void*) &Address;

      /* How many bytes do we want to write. */
      packet.length = 2;

      status = twi_master_write(&packet);

      if (status != READ_MEM_OK)
      {
          return(READ_MEM_FAILED);
      }
   }

   packet_received.addr = 0;
   packet_received.addr_length = 0;
   packet_received.chip = QT_i2c_address;
   packet_received.buffer = Data;
   packet_received.length = ByteCount;

   status = twi_master_read(&packet_received);

   if (status != READ_MEM_OK)
   {
      return(READ_MEM_FAILED);
   }

   return(READ_MEM_OK);
}
#else
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
      /* Make sure that when making the first read, address pointer is always written. */
        address_pointer = Address + 1;
        first_read = 0;
    }
   
    if ((address_pointer != Address) || (Address != message_processor_address))
    {
        address_pointer = Address;

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
#endif

uint8_t read_uint16_t( uint16_t Address, uint16_t *Data )
{
    uint8_t Temp[2];
    uint8_t Status;

    Status = read_mem(Address, 2, Temp);
    // format result
    *Data = ((uint16_t)Temp[1] << 8) + (uint16_t)Temp[0];

    return Status;
}

void InitTwi(void)
{
    TWAR = 0xAA;                            // Set own TWI slave address. Accept TWI General Calls.
    TWBR = TWI_TWBR_200KHZ;                 // Set bit rate register (Baudrate). Defined in header file.
    //TWSR = TWI_TWPS;                      // Not used. Driver presumes prescaler to be 00.
    TWDR = 0xFF;                            // Default content = SDA released.
    TWCR = (1<<TWEN)|                       // Enable TWI-interface and release TWI pins.
           (0<<TWIE)|(1<<TWINT)|            // Disable TWI Interupt.
           (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)| // Do not ACK on any requests, yet.
           (0<<TWWC);
}

void Twi_SendStop(void)
{
    TWCR = (1<<TWEN)|                             // TWI Interface enabled.
           (0<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
           (0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|       // Initiate a START condition.
           (0<<TWWC);
}

unsigned char  TWI_Wait(void)
{
    uint8_t h;
//    TwiWaitTimer = 200;
//    while(!(TWCR& (1<<TWINT)));  //»áÏÝÈëËÀÑ­»·
//    return (TWSR & TWSR_STATUS_MASK);
    
    for(h = 0; h < 200; h++)
    {
      if(TWCR & (1<<TWINT))
      {
        return (TWSR & TWSR_STATUS_MASK);
      }
    }      
    return (TWSR & TWSR_STATUS_MASK);
}

int twi_master_read(const twi_package_t *package)
{
    unsigned char TwiStatus;
    unsigned char * TwiData;
    unsigned char sla;
    unsigned char r_reg;
    unsigned int len;
    
    delay_ms(1);   // delay 1ms
    sla = package->chip;
    r_reg = package->addr;
    TwiData = package->buffer;
    len = package->length;

    // check argument
    if (package->length == 0)
    {
        return READ_MEM_FAILED;
    }

    TWDR = 0x00;
    if(package->addr_length != 0)
    {
        //while(TESTBIT(TWCR,TWIE));

        TWCR = (1<<TWEN)|                             // TWI Interface enabled.
               (0<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
               (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
               (0<<TWWC);                             //
        TwiStatus = TWI_Wait();
        if(!((TwiStatus == TWI_START)||(TwiStatus == TWI_REP_START)))
        {
            Twi_SendStop();
            return READ_MEM_FAILED;
        }

        TWDR = (sla<<TWI_ADR_BITS)|(0<<TWI_READ_BIT);
        TWCR = (1<<TWEN)|                             // TWI Interface enabled.
               (0<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
               (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
               (0<<TWWC);                             //

        TwiStatus = TWI_Wait();

        if(TwiStatus != TWI_MTX_ADR_ACK)
        {
          Twi_SendStop();
          return READ_MEM_FAILED;
        }

        TWDR = r_reg;
        TWCR = (1<<TWEN)|                             // TWI Interface enabled.
               (0<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
               (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
               (0<<TWWC);                             //

        TwiStatus = TWI_Wait();
        if(TwiStatus != TWI_MTX_DATA_ACK)
        {
          Twi_SendStop();
          return READ_MEM_FAILED;
        }

        Twi_SendStop();
    }

    TWDR = 0x00;
    TWCR = (1<<TWEN)|                             // TWI Interface enabled.
           (0<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
           (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
           (0<<TWWC);                             //
    TwiStatus = TWI_Wait();

    if(!((TwiStatus == TWI_START)||(TwiStatus == TWI_REP_START)))
    {
      Twi_SendStop();
      return READ_MEM_FAILED;
    }

    TWDR = (sla<<TWI_ADR_BITS)|(1<<TWI_READ_BIT);
    TWCR = (1<<TWEN)|                             // TWI Interface enabled.
           (0<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
           (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
           (0<<TWWC);                             //

    TwiStatus = TWI_Wait();

    if(TwiStatus != TWI_MRX_ADR_ACK)
    {
      Twi_SendStop();
      return READ_MEM_FAILED;
    }
    
    do
    {
        TWCR = (1<<TWEN)|                             // TWI Interface enabled.
               (0<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
               (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
               (0<<TWWC);                             //

        TwiStatus = TWI_Wait();

        *TwiData = TWDR;
        TwiData++;

        if(TwiStatus != TWI_MRX_DATA_ACK)
        {
          Twi_SendStop();
          return READ_MEM_FAILED;
        }
    }while(--len > 0);

    TWCR = (1<<TWEN)|                             // TWI Interface enabled.
           (0<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
           (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
           (0<<TWWC);
    TwiStatus = TWI_Wait();

    Twi_SendStop();
    return READ_MEM_OK;
}


int twi_master_write( const twi_package_t *package)
{
    unsigned char   TwiStatus;
    unsigned char   * TwiData;
    unsigned char sla;
    unsigned char w_reg;
    unsigned int len;

    sla = package->chip;
    w_reg = package->addr;

    // get a pointer to applicative data
    TwiData = package->buffer;
    len = package->length;

    if (package->length == 0)
    {
        return WRITE_MEM_FAILED;
    }

    TWDR = 0x00;
    TWCR = (1<<TWEN)|                             // TWI Interface enabled.
           (0<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
           (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
           (0<<TWWC);                             //
    TwiStatus = TWI_Wait();

    if(!((TwiStatus == TWI_START)||(TwiStatus == TWI_REP_START)))
    {
        Twi_SendStop();
        return WRITE_MEM_FAILED;
    }

    TWDR = (sla<<TWI_ADR_BITS)|(0<<TWI_READ_BIT);
    TWCR = (1<<TWEN)|                             // TWI Interface enabled.
           (0<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
           (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
           (0<<TWWC);                             //

    TwiStatus = TWI_Wait();
    if(TwiStatus != TWI_MTX_ADR_ACK)
    {
      Twi_SendStop();
      return WRITE_MEM_FAILED;
    }

    if(package->addr_length != 0)
    {
        TWDR = w_reg;
        TWCR = (1<<TWEN)|                             // TWI Interface enabled.
               (0<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
               (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
               (0<<TWWC);                             //

        TwiStatus = TWI_Wait();
        if(TwiStatus != TWI_MTX_DATA_ACK)
        {
          Twi_SendStop();
          return WRITE_MEM_FAILED;
        }
    }

    do
    {
        TWDR = *TwiData++;
        TWCR = (1<<TWEN)|                             // TWI Interface enabled.
               (0<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
               (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
               (0<<TWWC);                             //

        TwiStatus = TWI_Wait();
        if(TwiStatus != TWI_MTX_DATA_ACK)
        {
          Twi_SendStop();
          return WRITE_MEM_FAILED;
        }
    }while(--len > 0);

    Twi_SendStop();
    return WRITE_MEM_OK;
}

// check if mXT224 is connected via I2C addressing operation
#if defined(__TWI__)
uint8_t address_slave(void)
{
    uint8_t i;
    uint8_t status;
    uint8_t rxtmp;
    twi_package_t packet;

    packet.chip = QT_i2c_address;
    packet.addr = 0;
    packet.addr_length = 0;
    packet.buffer = (void*) &rxtmp;
    packet.length = 1;

    status = twi_master_read(&packet);
    if(status == READ_MEM_OK)
    {
        return (CONNECT_OK);
    }
    else
    {
        i = 0;
        while ((status == READ_MEM_FAILED) & (i < 10))
        {
            status = twi_master_read(&packet);
            if(status == READ_MEM_OK)
            {
                return (CONNECT_OK);
            }
            i++;
        }
    }
    return CONNECT_ERROR;
}

#else
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
#endif

#define __QT_DRIVER_BLOCK___
/******************************************************************************
*       QT602240 Driver Block
*****************************************************************************/

#define CHANGELINE_MASK     (1<<PD2)            /* CHG/ bit-mask e.g. bit 0 */

/* Data port register for CHANGE/ pin */
#define CHGPORT     PORTD
/* Data port register for CHANGE/ pin */
#define CHGPIN      PIND
/* Data-direction register for CHANGE pins */
#define CHGDDR      DDRD

//#define change_asserted()  ((CHGPIN & CHANGELINE_MASK) ? CHANGELINE_NEGATED : CHANGELINE_ASSERTED)
#define change_asserted()  ((CHGPIN & CHANGELINE_MASK) ? CHANGELINE_ASSERTED : CHANGELINE_NEGATED)

uint8_t ChangeLineStatus( void )
{
    return change_asserted();
}  

// ID information,Object Table Element1-n, Object1-n
uint8_t init_touch_driver(uint8_t I2C_address, void (*handler)(uint8_t *, uint8_t))
{
    uint16_t i;
    uint8_t  tmp;
    uint16_t current_address;
    uint16_t crc_address;

    info_id_t *id;
    object_t  *object_table;
    uint32_t  *CRC;
    uint8_t   status;
    uint8_t   current_report_id = 0;

    /* Set the message handler function pointer. */
//    application_message_handler = handler;

    /* save new slave address */
    QT_i2c_address = I2C_address;

    /* Poll device */
    status = address_slave();

    if(status == CONNECT_OK)
    {
        QT_printf("[TSP] I2C Slave Address = 0x%x\n\r",QT_i2c_address);
    }
    else
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }

    driver_setup = DRIVER_SETUP_INCOMPLETE;

    /* Read the info block data. */
    id = (info_id_t *) malloc(sizeof(info_id_t));
    if (id == NULL)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }

    if (read_id_block(id) != 1)  
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }

    object_table = (object_t *) malloc(id->num_declared_objects * sizeof(object_t));
    if (object_table == NULL)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }     

    /* Reading the whole object table block to memory directly doesn't work cause sizeof object_t
    isn't necessarily the same on every compiler/platform due to alignment issues. Endianness
    can also cause trouble. */

    current_address = OBJECT_TABLE_START_ADDRESS;

    max_report_id = 0;
    for (i = 0; i < id->num_declared_objects; i++)
    {
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
            max_message_length = object_table[i].size;
        }
    }

    /* Check that message processor was found. */
    if (max_message_length == 0)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }

    /* Read CRC. */
    CRC = (uint32_t *) malloc(sizeof(info_id_t));
    if (CRC == NULL)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }  

    info_block = malloc(sizeof(info_block_t));
    if (info_block == NULL)
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
    if (msg == NULL)
    {
        return(DRIVER_SETUP_INCOMPLETE);
    }  

    /* Allocate memory for report id map now that the number of report id's is known. */
     report_id_map = malloc(sizeof(report_id_map_t) * max_report_id + 1);
    if (report_id_map == NULL)
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
                    report_id_map[current_report_id].object_type = object_table[i].object_type;
                }
            }
        }
    }

    if((id->version == 0x14))
    {
        if( id->build == 0x0B)
        {
            /* Store communication cfg processor address. */
            comc_cfg_address = get_object_address(SPT_COMCONFIG_T18, 0);
            if (command_processor_address == 0)
            {
                return(DRIVER_SETUP_INCOMPLETE);
            }
        }

        diagnostic_addr = get_object_address(DEBUG_DIAGNOSTIC_T37, 0);
        diagnostic_size = get_object_size(DEBUG_DIAGNOSTIC_T37);

        if (diagnostic_addr == 0)
        {
            return(DRIVER_SETUP_INCOMPLETE);
        }
    } 

    driver_setup = DRIVER_SETUP_OK;
    return(DRIVER_SETUP_OK);
}


uint8_t close_touch_driver()
{
    free(info_block);
    free(report_id_map);
    free(msg);

    /* Do platform specific clean-up, interrupt disable etc. */
    //   platform_close();

    return(0);
}

uint8_t reset_chip(void)
{
    uint8_t data = 1u;
    return(write_mem(command_processor_address + RESET_OFFSET, 1, &data));
}

uint8_t calibrate_chip(void)
{
    uint8_t data = 1u;
    return(write_mem(command_processor_address + CALIBRATE_OFFSET, 1, &data));
}

uint8_t diagnostic_chip(uint8_t mode)
{
    uint8_t status;
    status = write_mem(command_processor_address + DIAGNOSTIC_OFFSET, 1, &mode);
    return(status);
}

uint8_t backup_config(void)
{
    /* Write 0x55 to BACKUPNV register to initiate the backup. */
    uint8_t data = 0x55u;
    return(write_mem(command_processor_address + BACKUP_OFFSET, 1, &data));
}

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

uint8_t write_power_config(gen_powerconfig_t7_config_t cfg)
{
    return(write_simple_config(GEN_POWERCONFIG_T7, 0, (void *) &cfg));
}

uint8_t write_acquisition_config(gen_acquisitionconfig_t8_config_t cfg)
{
    return(write_simple_config(GEN_ACQUISITIONCONFIG_T8, 0, (void *) &cfg));
}

uint8_t write_multitouchscreen_config(uint8_t instance, touch_multitouchscreen_t9_config_t cfg)
{
    uint16_t object_address;
    uint8_t *tmp;
    uint8_t status;
    uint8_t object_size;

    object_size = get_object_size(TOUCH_MULTITOUCHSCREEN_T9);
    if (object_size == 0)
    {
        return(CFG_WRITE_FAILED);
    }
    tmp = (uint8_t *) malloc(object_size);
    if (tmp == NULL)
    {
        return(CFG_WRITE_FAILED);
    }

    memset(tmp,0,object_size);

    /* 18 elements at beginning are 1 byte. */
    memcpy(tmp, &cfg, 18);

    /* Next two are 2 bytes. */
    *(tmp + 18) = (uint8_t) (cfg.xrange &  0xFF);
    *(tmp + 19) = (uint8_t) (cfg.xrange >> 8);

    *(tmp + 20) = (uint8_t) (cfg.yrange &  0xFF);
    *(tmp + 21) = (uint8_t) (cfg.yrange >> 8);

    /* And the last 4(8) 1 bytes each again. */
    *(tmp + 22) = cfg.xloclip;
    *(tmp + 23) = cfg.xhiclip;
    *(tmp + 24) = cfg.yloclip;
    *(tmp + 25) = cfg.yhiclip;

#if defined(__VER_1_4__)
    *(tmp + 26) = cfg.xedgectrl;
    *(tmp + 27) = cfg.xedgedist;
    *(tmp + 28) = cfg.yedgectrl;
    *(tmp + 29) = cfg.yedgedist;
#endif
    object_address = get_object_address(TOUCH_MULTITOUCHSCREEN_T9, instance);

    if (object_address == 0)
    {
        return(CFG_WRITE_FAILED);
    }

    status = write_mem(object_address, object_size, tmp);

    free(tmp);
    return(status);
}

uint8_t write_keyarray_config(uint8_t instance, touch_keyarray_t15_config_t cfg)
{
    return(write_simple_config(TOUCH_KEYARRAY_T15, instance, (void *) &cfg));
}

uint8_t write_linearization_config(uint8_t instance, proci_linearizationtable_t17_config_t cfg)
{
    uint16_t object_address;
    uint8_t *tmp;
    uint8_t status;
    uint8_t object_size;

    object_size = get_object_size(PROCI_LINEARIZATIONTABLE_T17);
    if (object_size == 0)
    {
        return(CFG_WRITE_FAILED);
    }
    tmp = (uint8_t *) malloc(object_size);

    if (tmp == NULL)
    {
        return(CFG_WRITE_FAILED);
    }

    memset(tmp,0,object_size);

    *(tmp + 0) = cfg.ctrl;
    *(tmp + 1) = (uint8_t) (cfg.xoffset & 0x00FF);
    *(tmp + 2) = (uint8_t) (cfg.xoffset >> 8);

    memcpy((tmp+3), &cfg.xsegment, 16);

    *(tmp + 19) = (uint8_t) (cfg.yoffset & 0x00FF);
    *(tmp + 20) = (uint8_t) (cfg.yoffset >> 8);

    memcpy((tmp+21), &cfg.ysegment, 16);

    object_address = get_object_address(PROCI_LINEARIZATIONTABLE_T17, instance);

    if (object_address == 0)
    {
        return(CFG_WRITE_FAILED);
    }

    status = write_mem(object_address, object_size, tmp);

    free(tmp);
    return(status);
}

uint8_t write_comc_config(uint8_t instance, spt_comcconfig_t18_config_t cfg)
{
    return(write_simple_config(SPT_COMCONFIG_T18, instance, (void *) &cfg));
}

uint8_t write_gpio_config(uint8_t instance, spt_gpiopwm_t19_config_t cfg)
{
    return(write_simple_config(SPT_GPIOPWM_T19, instance, (void *) &cfg));
}

uint8_t write_gripsuppression_config(uint8_t instance, proci_gripfacesuppression_t20_config_t cfg)
{
    return(write_simple_config(PROCI_GRIPFACESUPPRESSION_T20, instance, (void *) &cfg));
}

uint8_t write_noisesuppression_config(uint8_t instance, procg_noisesuppression_t22_config_t cfg)
{
    return(write_simple_config(PROCG_NOISESUPPRESSION_T22, instance, (void *) &cfg));
}

uint8_t write_proximity_config(uint8_t instance, touch_proximity_t23_config_t cfg)
{
    uint16_t object_address;
    uint8_t *tmp;
    uint8_t status;
    uint8_t object_size;

    object_size = get_object_size(TOUCH_PROXIMITY_T23);
    if (object_size == 0)
    {
        return(CFG_WRITE_FAILED);
    }
    tmp = (uint8_t *) malloc(object_size);
    if (tmp == NULL)
    {
        return(CFG_WRITE_FAILED);
    }

    memset(tmp,0,object_size);

    *(tmp + 0) = cfg.ctrl;
    *(tmp + 1) = cfg.xorigin;
    *(tmp + 2) = cfg.yorigin;
    *(tmp + 3) = cfg.xsize;
    *(tmp + 4) = cfg.ysize;
    *(tmp + 5) = cfg.reserved_for_future_aks_usage;
    *(tmp + 6) = cfg.blen;

    *(tmp + 7) = (uint8_t) (cfg.tchthr & 0x00FF);
    *(tmp + 8) = (uint8_t) (cfg.tchthr >> 8);

    *(tmp + 9) = cfg.tchdi;
    *(tmp + 10) = cfg.average;

    *(tmp + 11) = (uint8_t) (cfg.rate & 0x00FF);
    *(tmp + 12) = (uint8_t) (cfg.rate >> 8);

    object_address = get_object_address(TOUCH_PROXIMITY_T23, instance);

    if (object_address == 0)
    {
        return(CFG_WRITE_FAILED);
    }

    status = write_mem(object_address, object_size, tmp);

    free(tmp);
    return(status);
}

uint8_t write_onetouchgesture_config(uint8_t instance, proci_onetouchgestureprocessor_t24_config_t cfg)
{
    uint16_t object_address;
    uint8_t *tmp;
    uint8_t status;
    uint8_t object_size;

    object_size = get_object_size(PROCI_ONETOUCHGESTUREPROCESSOR_T24);
    if (object_size == 0)
    {
        return(CFG_WRITE_FAILED);
    }
    tmp = (uint8_t *) malloc(object_size);
    if (tmp == NULL)
    {
        return(CFG_WRITE_FAILED);
    }

    memset(tmp,0,object_size);

    *(tmp + 0) = cfg.ctrl;
#if defined(__VER_1_2__)
    *(tmp + 1) = 0;
#else
    *(tmp + 1) = cfg.numgest;
#endif

    *(tmp + 2) = (uint8_t) (cfg.gesten & 0x00FF);
    *(tmp + 3) = (uint8_t) (cfg.gesten >> 8);

    memcpy((tmp+4), &cfg.pressproc, 7);

    *(tmp + 11) = (uint8_t) (cfg.flickthr & 0x00FF);
    *(tmp + 12) = (uint8_t) (cfg.flickthr >> 8);

    *(tmp + 13) = (uint8_t) (cfg.dragthr & 0x00FF);
    *(tmp + 14) = (uint8_t) (cfg.dragthr >> 8);

    *(tmp + 15) = (uint8_t) (cfg.tapthr & 0x00FF);
    *(tmp + 16) = (uint8_t) (cfg.tapthr >> 8);

    *(tmp + 17) = (uint8_t) (cfg.throwthr & 0x00FF);
    *(tmp + 18) = (uint8_t) (cfg.throwthr >> 8);

    object_address = get_object_address(PROCI_ONETOUCHGESTUREPROCESSOR_T24, instance);

    if (object_address == 0)
    {
        return(CFG_WRITE_FAILED);
    }

    status = write_mem(object_address, object_size, tmp);

    free(tmp);
    return(status);
}

uint8_t write_selftest_config(uint8_t instance, spt_selftest_t25_config_t cfg)
{
    uint16_t object_address;
    uint8_t *tmp;
    uint8_t status;
    uint8_t object_size;

    object_size = get_object_size(SPT_SELFTEST_T25);
    if (object_size == 0)
    {
        return(CFG_WRITE_FAILED);
    }
    tmp = (uint8_t *) malloc(object_size);


    if (tmp == NULL)
    {
        return(CFG_WRITE_FAILED);
    }

    memset(tmp,0,object_size);

    *(tmp + 0) = cfg.ctrl;
    *(tmp + 1) = cfg.cmd;
#if(NUM_OF_TOUCH_OBJECTS)
    *(tmp + 2) = (uint8_t) (cfg.upsiglim & 0x00FF);
    *(tmp + 3) = (uint8_t) (cfg.upsiglim >> 8);

    *(tmp + 2) = (uint8_t) (cfg.losiglim & 0x00FF);
    *(tmp + 3) = (uint8_t) (cfg.losiglim >> 8);
#endif
    object_address = get_object_address(SPT_SELFTEST_T25, instance);

    if (object_address == 0)
    {
        return(CFG_WRITE_FAILED);
    }

    status = write_mem(object_address, object_size, tmp);

    free(tmp);
    return(status);
}

uint8_t write_twotouchgesture_config(uint8_t instance, proci_twotouchgestureprocessor_t27_config_t cfg)
{
    uint16_t object_address;
    uint8_t *tmp;
    uint8_t status;
    uint8_t object_size;

    object_size = get_object_size(PROCI_TWOTOUCHGESTUREPROCESSOR_T27);
    if (object_size == 0)
    {
        return(CFG_WRITE_FAILED);
    }
    tmp = (uint8_t *) malloc(object_size);

    if (tmp == NULL)
    {
        return(CFG_WRITE_FAILED);
    }

    memset(tmp,0,object_size);

    *(tmp + 0) = cfg.ctrl;

#if defined(__VER_1_2__)
    *(tmp + 1) = 0;
#else
    
    *(tmp + 1) = cfg.numgest;
#endif

    *(tmp + 2) = 0;

    *(tmp + 3) = cfg.gesten;

    *(tmp + 4) = cfg.rotatethr;

    *(tmp + 5) = (uint8_t) (cfg.zoomthr & 0x00FF);
    *(tmp + 6) = (uint8_t) (cfg.zoomthr >> 8);

    object_address = get_object_address(PROCI_TWOTOUCHGESTUREPROCESSOR_T27, instance);

    if (object_address == 0)
    {
        return(CFG_WRITE_FAILED);
    }

    status = write_mem(object_address, object_size, tmp);

    free(tmp);
    return(status);
}

uint8_t write_CTE_config(spt_cteconfig_t28_config_t cfg)
{
    return(write_simple_config(SPT_CTECONFIG_T28, 0, (void *) &cfg));
}

uint8_t write_simple_config(uint8_t object_type, uint8_t instance, void *cfg)
{
    uint16_t object_address;
    uint8_t object_size;

    object_address = get_object_address(object_type, instance);
    object_size = get_object_size(object_type);

    if ((object_size == 0) || (object_address == 0))
    {
        return(CFG_WRITE_FAILED);
    }

    return (write_mem(object_address, object_size, cfg));
}

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

uint8_t get_max_report_id()
{
    return(max_report_id);
}

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

uint32_t get_stored_infoblock_crc()
{
    uint32_t crc;
    crc = (uint32_t) (((uint32_t) info_block->CRC_hi) << 16);
    crc = crc | info_block->CRC;
    return(crc);
}

uint8_t calculate_infoblock_crc(uint32_t *crc_pointer)
{

    uint32_t crc = 0;
    uint16_t crc_area_size;
    uint8_t  *mem;
    uint8_t  i;
    uint8_t  status;

    /* 7 bytes of version data, 6 * NUM_OF_OBJECTS bytes of object table. */
    crc_area_size = 7 + info_block->info_id.num_declared_objects * 6;

    mem = (uint8_t *) malloc(crc_area_size);
    if (mem == NULL)
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

#define __QT_CONFIG__
/*****************************************************************************
*       QT602240  Configuration Block
* ***************************************************************************/
void qt_Power_Config_Init(void)
{
    /* Set Idle Acquisition Interval to 32 ms. */
    power_config.idleacqint = 10;

    /* Set Active Acquisition Interval to 16 ms. */
    power_config.actvacqint = 255;

    /* Set Active to Idle Timeout to 4 s (one unit = 200ms). */
    power_config.actv2idleto = 0;

    /* Write power config to chip. */
    if (write_power_config(power_config) != CFG_WRITE_OK)
    {
        QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }
}

void qt_Acquisition_Config_Init(void)
{
    acquisition_config.chrgtime = 8;    // 2us
    acquisition_config.reserved = 0;
    acquisition_config.tchdrift = 20;   // 4s
    acquisition_config.driftst = 20;    // 4s
    acquisition_config.tchautocal = 0;  // infinite
    acquisition_config.sync = 0;        // disabled

#if defined(__VER_1_4__)
    acquisition_config.atchcalst = 0;
    acquisition_config.atchcalsthr = 0;
#endif

    if (write_acquisition_config(acquisition_config) != CFG_WRITE_OK)
    {
        QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }
}

void qt_Multitouchscreen_Init(void)
{
    touchscreen_config.ctrl = 143;    // enable + message-enable
    touchscreen_config.xorigin = 0;
    touchscreen_config.yorigin = 0;
    touchscreen_config.xsize = 16;
    touchscreen_config.ysize = 10;
    touchscreen_config.akscfg = 0;
    touchscreen_config.blen = 0x41;
    touchscreen_config.tchthr = 50;
    touchscreen_config.tchdi = 2;
    touchscreen_config.orient = 0;
    touchscreen_config.mrgtimeout = 0;
    touchscreen_config.movhysti = 3;
    touchscreen_config.movhystn = 1;
    touchscreen_config.movfilter = 0;
    touchscreen_config.numtouch= 1;
    touchscreen_config.mrghyst = 10;
    touchscreen_config.mrgthr = 10;
    touchscreen_config.amphyst = 10;
    touchscreen_config.xrange = 800;
    touchscreen_config.yrange = 480;
    touchscreen_config.xloclip = 0;
    touchscreen_config.xhiclip = 0;
    touchscreen_config.yloclip = 0;
    touchscreen_config.yhiclip = 0;
#if defined(__VER_1_4__)
    touchscreen_config.xedgectrl = 0;
    touchscreen_config.xedgedist = 0;
    touchscreen_config.yedgectrl = 0;
    touchscreen_config.yedgedist = 0;
#endif

    if (write_multitouchscreen_config(0, touchscreen_config) != CFG_WRITE_OK)
    {
        QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }
}

void qt_KeyArray_Init(void)
{
    keyarray_config.ctrl = 0;
    keyarray_config.xorigin = 0;
    keyarray_config.yorigin = 0;
    keyarray_config.xsize = 0;
    keyarray_config.ysize = 0;
    keyarray_config.akscfg = 0;
    keyarray_config.blen = 0;
    keyarray_config.tchthr = 0;
    keyarray_config.tchdi = 0;
    keyarray_config.reserved[0] = 0;
    keyarray_config.reserved[1] = 0;

    if (write_keyarray_config(0, keyarray_config) != CFG_WRITE_OK)
    {
        QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }
}

void qt_ComcConfig_Init(void)
{
    comc_config.ctrl = 0x01;
    comc_config.cmd = COMM_MODE1;

    if (get_object_address(SPT_COMCONFIG_T18, 0) != OBJECT_NOT_FOUND)
    {
        if (write_comc_config(0, comc_config) != CFG_WRITE_OK)
        {
            QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
        }
    }
}

void qt_Gpio_Pwm_Init(void)
{
    gpiopwm_config.ctrl = 0;
    gpiopwm_config.reportmask = 0;
    gpiopwm_config.dir = 0;
    gpiopwm_config.intpullup = 0;
    gpiopwm_config.out = 0;
    gpiopwm_config.wake = 0;
    gpiopwm_config.pwm = 0;
    gpiopwm_config.period = 0;
    gpiopwm_config.duty[0] = 0;
    gpiopwm_config.duty[1] = 0;
    gpiopwm_config.duty[2] = 0;
    gpiopwm_config.duty[3] = 0;

    if (write_gpio_config(0, gpiopwm_config) != CFG_WRITE_OK)
    {
        QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }
}

void qt_Grip_Face_Suppression_Config_Init(void)
{
    gripfacesuppression_config.ctrl = 0;
    gripfacesuppression_config.xlogrip = 0;
    gripfacesuppression_config.xhigrip = 0;
    gripfacesuppression_config.ylogrip = 0;
    gripfacesuppression_config.yhigrip = 0;
    gripfacesuppression_config.maxtchs = 0;
    gripfacesuppression_config.reserved = 0;
    gripfacesuppression_config.szthr1 = 0;
    gripfacesuppression_config.szthr2 = 0;
    gripfacesuppression_config.shpthr1 = 0;
    gripfacesuppression_config.shpthr2 = 0;

#if defined(__VER_1_4__)
    gripfacesuppression_config.supextto = 0;
#endif

    /* Write grip suppression config to chip. */
    if (get_object_address(PROCI_GRIPFACESUPPRESSION_T20, 0) != OBJECT_NOT_FOUND)
    {
        if (write_gripsuppression_config(0, gripfacesuppression_config) !=
            CFG_WRITE_OK)
        {
            QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
        }
    }
}

void qt_Noise_Suppression_Config_Init(void)
{
    noise_suppression_config.ctrl = 0;   noise_suppression_config.ctrl = 0;

#if defined(__VER_1_2__)
    noise_suppression_config.outflen = 0;
#elif defined(__VER_1_4__)
    noise_suppression_config.reserved = 0;
#endif

    noise_suppression_config.reserved1 = 0;
    noise_suppression_config.gcaful = 0;
    noise_suppression_config.gcafll = 0;


#if defined(__VER_1_2__)
    noise_suppression_config.gcaflcount = 0;
#elif defined(__VER_1_4__)
    noise_suppression_config.actvgcafvalid = 0;
#endif


#if defined(__VER_1_2__)
    noise_suppression_config.freq0 = 0;
    noise_suppression_config.freq1 = 0;
    noise_suppression_config.freq2 = 0;
#elif defined(__VER_1_4__)
    noise_suppression_config.freq[0] = 10;
    noise_suppression_config.freq[1] = 15;
    noise_suppression_config.freq[2] = 20;
    noise_suppression_config.freq[3] = 25;
    noise_suppression_config.freq[4] = 30;
    noise_suppression_config.idlegcafvalid = 0;
#endif

    /* Write Noise suppression config to chip. */
    if (get_object_address(PROCG_NOISESUPPRESSION_T22, 0) != OBJECT_NOT_FOUND)
    {
        if (write_noisesuppression_config(0,noise_suppression_config) != CFG_WRITE_OK)
        {
            QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
        }
    }
}

void qt_Proximity_Config_Init(void)
{
    proximity_config.ctrl = 0;
    proximity_config.xorigin = 0;
    proximity_config.xsize = 0;
    proximity_config.ysize = 0;
    proximity_config.reserved_for_future_aks_usage = 0;
    proximity_config.blen = 0;
    proximity_config.tchthr = 0;
    proximity_config.tchdi = 0;
    proximity_config.average = 0;
    proximity_config.rate = 0;

    if (get_object_address(TOUCH_PROXIMITY_T23, 0) != OBJECT_NOT_FOUND)
    {
        if (write_proximity_config(0, proximity_config) != CFG_WRITE_OK)
        {
            QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
        }
    }
}

void qt_One_Touch_Gesture_Config_Init(void)
{
    /* Disable one touch gestures. */
    onetouch_gesture_config.ctrl = 0;
#if defined(__VER_1_2__)
    onetouch_gesture_config.reserved_1 = 0;
#elif defined(__VER_1_4__)
    onetouch_gesture_config.numgest = 0;
#endif

    onetouch_gesture_config.gesten = 0;
    onetouch_gesture_config.pressproc = 0;
    onetouch_gesture_config.tapto = 0;
    onetouch_gesture_config.flickto = 0;
    onetouch_gesture_config.dragto = 0;
    onetouch_gesture_config.spressto = 0;
    onetouch_gesture_config.lpressto = 0;
    onetouch_gesture_config.reppressto = 0;
    onetouch_gesture_config.flickthr = 0;
    onetouch_gesture_config.dragthr = 0;
    onetouch_gesture_config.tapthr = 0;
    onetouch_gesture_config.throwthr = 0;

    if (get_object_address(PROCI_ONETOUCHGESTUREPROCESSOR_T24, 0) != OBJECT_NOT_FOUND)
    {
        if (write_onetouchgesture_config(0, onetouch_gesture_config) != CFG_WRITE_OK)
        {
            QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
        }
    }
}

void qt_Selftest_Init(void)
{
    selftest_config.ctrl = 0;
    selftest_config.cmd = 0;

#if(NUM_OF_TOUCH_OBJECTS)
    siglim.upsiglim[0] = 0;
    siglim.losiglim[0] = 0;
#endif
    if (get_object_address(SPT_SELFTEST_T25, 0) != OBJECT_NOT_FOUND)
    {
        if (write_selftest_config(0,selftest_config) != CFG_WRITE_OK)
        {
            QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
        }
    }
}

void qt_Two_touch_Gesture_Config_Init(void)
{
    /* Disable two touch gestures. */
    twotouch_gesture_config.ctrl = 0;
#if defined(__VER_1_2__)
    twotouch_gesture_config.reserved1 = 0;
#elif defined(__VER_1_4__)
    twotouch_gesture_config.numgest = 0;
#endif
    twotouch_gesture_config.reserved2 = 0;
    twotouch_gesture_config.gesten = 0;
    twotouch_gesture_config.rotatethr = 0;
    twotouch_gesture_config.zoomthr = 0;

    if (get_object_address(PROCI_TWOTOUCHGESTUREPROCESSOR_T27, 0) !=
        OBJECT_NOT_FOUND)
    {
        if (write_twotouchgesture_config(0, twotouch_gesture_config) !=
            CFG_WRITE_OK)
        {
            QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
        }
    }
}

void qt_CTE_Config_Init(void)
{
    /* Set CTE config */
    cte_config.ctrl = 1;
    cte_config.cmd = 0;
    cte_config.mode = 2;
    cte_config.idlegcafdepth = 4;
    cte_config.actvgcafdepth = 4;

    /* Write CTE config to chip. */
    if (get_object_address(SPT_CTECONFIG_T28, 0) != OBJECT_NOT_FOUND)
    {
        if (write_CTE_config(cte_config) != CFG_WRITE_OK)
        {
            QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
        }
    }
}

unsigned char Comm_Config_Process(unsigned char change_en)
{
    if(change_en == 1)
    {
        change_en = 0;

        if (get_object_address(SPT_COMCONFIG_T18, 0) != OBJECT_NOT_FOUND)
        {
            if((comc_config.cmd == COMM_MODE3))
            {
                if(PIND_Bit2 == 1)
                {
                    comc_config.cmd = COMM_MODE1;
                    return (change_en);
                }
            }

            if (write_comc_config(0, comc_config) != CFG_WRITE_OK)
            {
                return (change_en);
            }
        }
    }
    return (change_en);
}

#define __QT_APPLICATION__

void get_message(void)
{
    if (driver_setup == DRIVER_SETUP_OK)
    {
        if(read_mem(message_processor_address, max_message_length, tmsg) == READ_MEM_OK)
        {
          message_handler(tmsg, max_message_length);
        }
        else
        {
          QT_printf("Read Msg Processor ERROR");
        }
    }
}

void write_message_to_usart(uint8_t msg[], uint8_t length)
{
    int i;
    for (i = 0; i < length; i++)
    {
        QT_printf("0x%02x ", msg[i]);
    }
    putchar('\n');
    putchar('\r');
}

void message_handler(uint8_t *msg, uint8_t length)
{
#ifdef OPTION_PRINT_MESSAGES
    QT_printf("Touch IC message: ");
    write_message_to_usart(msg, length);
    QT_printf("\n\r");
#endif /* OPTION_PRINT_MESSAGES */

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

void Change_TouchNum(uint8_t touch_num)
{
    /* Multitouch screen config. */
    touch_multitouchscreen_t9_config_t touchscreen_config = {0};

    if(touchscreen_config.numtouch != touch_num)
    {

        touchscreen_config.numtouch = touch_num;


        /* Write touchscreen (1st instance) config to chip. */
        if (write_multitouchscreen_config(0, touchscreen_config) != CFG_WRITE_OK)
        {
            QT_printf("Multitouch screen config write failed!\n\r");
        }
        else
        {
            QT_printf("Multitouch screen config written successfully!\n\r");
        }

        /* Backup settings to NVM. */
        if (backup_config() != WRITE_MEM_OK)
        {
            QT_printf("Failed to backup, exiting...\n\r");
        }
        else
        {
            QT_printf("Backed up the config to non-volatile memory!\n\r");
        }

        /* Calibrate the touch IC. */
        if (calibrate_chip() != WRITE_MEM_OK)
        {
            QT_printf("Failed to calibrate, exiting...\n\r");
        }
        else
        {
            QT_printf("Chip calibrated!\n\r");
        }
    }
}

uint8_t init_touch_app(void)
{
    uint8_t return_val = true;
    uint8_t touch_chip_found = 0;
    uint8_t report_id;
    uint8_t max_report_id;
    uint8_t object_type, instance;
    uint8_t version, family_id, variant, build;
    uint32_t crc, stored_crc;
    uint16_t i;

    /* Try to initialize the driver. */
    for (i = 0; i < NUM_OF_I2C_ADDR; i++)
    {
        QT_printf("Checking address 0x%x...\n\r",i2c_addresses[i]);
        if((init_touch_driver(i2c_addresses[i],  &message_handler)) == DRIVER_SETUP_OK)
        {
            touch_chip_found = 1;
            break;
        }  
    }

    if (touch_chip_found == 0)
    {
        QT_printf("Connect / info block read failed!\n\r");
        return_val = false;
    }
    else
    {   // only continue here if chip is identified 
        // Get and show the version information. 
        get_family_id(&family_id);
        get_variant_id(&variant);
        get_version(&version);
        get_build_number(&build);

        QT_printf("Version:        0x%x\n\r", version);
        QT_printf("Family:         0x%x\n\r", family_id);
        QT_printf("Variant:        0x%x\n\r", variant);
        QT_printf("Build number:   0x%x\n\r", build);

        QT_printf("Matrix X size : %d\n\r", info_block->info_id.matrix_x_size);
        QT_printf("Matrix Y size : %d\n\r", info_block->info_id.matrix_y_size);

        if(calculate_infoblock_crc(&crc) != CRC_CALCULATION_OK)
        {
            QT_printf("Calculating CRC failed, skipping check!\n\r");
        }
        else
        {
            QT_printf("Calculated CRC:\t");
//            write_message_to_usart((uint8_t *) &crc, 4);
            QT_printf("\n\r");
        }

        stored_crc = get_stored_infoblock_crc();
        QT_printf("Stored CRC:\t");
//        write_message_to_usart((uint8_t *) &stored_crc, 4);
        QT_printf("\n\r");

        if (stored_crc != crc)
        {
            QT_printf("Warning: info block CRC value doesn't match the calculated!\n\r");
        }
        else
        {
            QT_printf("Info block CRC value OK.\n\n\r");

        }

        /* Test the report id to object type / instance mapping: get the maximum
        * report id and print the report id map. */

        QT_printf("Report ID to Object type / instance mapping:\n\r");
        max_report_id = get_max_report_id();
        for (report_id = 1; report_id <= max_report_id; report_id++)
        {
            object_type = report_id_to_type(report_id, &instance);
            QT_printf("[TSP] Report ID : %d, Object Type : T%d, Instance : %d\n\r",report_id ,object_type,instance);
        }

// following funcs are for configuration operation
#ifdef OPTION_WRITE_CONFIG
        qt_Power_Config_Init();
        qt_Acquisition_Config_Init();
        qt_Multitouchscreen_Init();
        qt_KeyArray_Init();
        qt_ComcConfig_Init();
        qt_Gpio_Pwm_Init();
        qt_Grip_Face_Suppression_Config_Init();
        qt_Noise_Suppression_Config_Init();
        qt_Proximity_Config_Init();
        qt_One_Touch_Gesture_Config_Init();
        qt_Selftest_Init();
        qt_Two_touch_Gesture_Config_Init();
        qt_CTE_Config_Init();

        // Backup settings to NVM. 
        if (backup_config() != WRITE_MEM_OK)
        {
            QT_printf("Failed to backup, exiting...\n\r");
            return_val = false;
        }
        else
        {
            QT_printf("Backed up the config to non-volatile memory!\n\r");
        }

#else
        QT_printf("Chip setup sequence was bypassed!\n\r");
#endif // OPTION_WRITE_CONFIG 

        // Calibrate the touch IC. 
        if (calibrate_chip() != WRITE_MEM_OK)
        {
            QT_printf("Failed to calibrate, exiting...\n\r");
            return_val = false;
        }
        else
        {
            QT_printf("Chip calibrated!\n\r");
        }

        QT_printf("\nWaiting for touch chip messages...\n\n\r");
    }   

    return return_val;
}

__task main( void )
{
    // reset mXT224
    DDRD  |= 0x08;    //pd3 output 
    PORTD &= ~0x08;   //pd3 low
    delay_ms(30);     //delay 30ms 
    PORTD |= 0x08;    //pd3 high
    delay_ms(100);    //delay 100ms
    
//    USART0_Init(BAUD_38400);
       
#if defined(__TWI__)
    InitTwi();
#else
    InitI2c();
#endif
    
    CHGPORT |= CHANGELINE_MASK;
    CHGDDR &= ~CHANGELINE_MASK;
    
    chip_detected_flag = init_touch_app();  // find and initialise QT device 
   
    if(chip_detected_flag == true)
    {
/*        uint8_t version, family_id, variant, build;
      
        // Get and show the version information. 
        // done at init_touch_app()
        get_family_id(&family_id);
        get_variant_id(&variant);
        get_version(&version);
        get_build_number(&build);
*/
        // clean messages except for normal touch
        while(ChangeLineStatus() == CHANGELINE_NEGATED) 
        {    
            get_message();
            // delay_ms(200);   // check for next one????
        }  
    }   
    else      // cannot find mXT224
    {
      while(1){}        
    }

   ExtIntConfigure();   // int1 low level interrupt
    __enable_interrupt();
    
    /* loop forever... */
    // for touch
    for(;;)
    {
      if(ChangeLineStatus() == CHANGELINE_NEGATED)
        get_message();
    }
}

#define __QT_DEBUG__

unsigned char read_diagnostic_debug(debug_diagnositc_t37_t *dbg, unsigned char mode, unsigned char page)
{
    unsigned char status;

    diagnostic_addr = get_object_address(DEBUG_DIAGNOSTIC_T37, 0);
    diagnostic_size = get_object_size(DEBUG_DIAGNOSTIC_T37);

    diagnostic_chip(mode);
    status = read_mem(diagnostic_addr,2, &dbg->mode);

    do
    {
        status = read_mem(diagnostic_addr,2, &dbg->mode);

        QT_printf("[TSP] DBG_PAGE = %d \n\r",dbg->page);

        if(status == READ_MEM_OK)
        {
            if(dbg->page > 128)
            {
                if(page > 128)
                {
                    if(dbg->page < page)
                    {
                        status = diagnostic_chip(QT_PAGE_UP);
                        delay_ms(1);
                    }
                    else if(dbg->page > page)
                    {
                        diagnostic_chip(QT_PAGE_DOWN);
                        delay_ms(1);
                    }
                }
                else
                {
                    if(dbg->page > page)
                    {
                        diagnostic_chip(QT_PAGE_UP);
                        delay_ms(1);
                    }
                    else if(dbg->page < page)
                    {
                        diagnostic_chip(QT_PAGE_DOWN);
                        delay_ms(1);
                    }
                }
            }
            else
            {
                if(page < 128)
                {
                    if(dbg->page < page)
                    {
                        diagnostic_chip(QT_PAGE_UP);
                        delay_ms(1);
                    }
                    else if(dbg->page > page)
                    {
                        diagnostic_chip(QT_PAGE_DOWN);
                        delay_ms(1);
                    }
                }
                else
                {
                    if(dbg->page > page)
                    {
                        diagnostic_chip(QT_PAGE_UP);
                        delay_ms(1);
                    }
                    else if(dbg->page < page)
                    {
                        diagnostic_chip(QT_PAGE_DOWN);
                        delay_ms(1);
                    }
                }
            }
        }
    }while(dbg->page != page);

    status = read_mem(diagnostic_addr,diagnostic_size, &dbg->mode);

    return status;
}

unsigned char read_diagnostic_delta(debug_diagnositc_t37_delta_t * dbg, unsigned char page)
{
    unsigned char status;

    diagnostic_chip(QT_DELTA_MODE);

    diagnostic_addr = get_object_address(DEBUG_DIAGNOSTIC_T37, 0);
    diagnostic_size = get_object_size(DEBUG_DIAGNOSTIC_T37);

    read_mem(diagnostic_addr,2, &dbg->mode);

    do
    {
        status = read_mem(diagnostic_addr,2, & dbg->mode);

        if(status == READ_MEM_OK)
        {
            if(dbg->page > 128)
            {
                if(page > 128)
                {
                    if(dbg->page < page)
                    {
                        diagnostic_chip(QT_PAGE_UP);
                    }
                    else if(dbg->page > page)
                    {
                        diagnostic_chip(QT_PAGE_DOWN);
                    }
                }
                else
                {
                    if(dbg->page > page)
                    {
                        diagnostic_chip(QT_PAGE_UP);
                    }
                    else if(dbg->page < page)
                    {
                        diagnostic_chip(QT_PAGE_DOWN);
                    }
                }
            }
            else
            {
                if(page < 128)
                {
                    if(dbg->page < page)
                    {
                        diagnostic_chip(QT_PAGE_UP);
                    }
                    else if(dbg->page > page)
                    {
                        diagnostic_chip(QT_PAGE_DOWN);
                    }
                }
                else
                {
                    if(dbg->page > page)
                    {
                        diagnostic_chip(QT_PAGE_UP);
                    }
                    else if(dbg->page < page)
                    {
                        diagnostic_chip(QT_PAGE_DOWN);
                    }
                }
            }
        }
        delay_ms(1);
    }while(dbg->page != page);

    status = read_mem(diagnostic_addr,diagnostic_size, &dbg->mode);

    return status;
}

unsigned char read_diagnostic_reference(debug_diagnositc_t37_reference_t * dbg,unsigned char page)
{
    unsigned char status;

    diagnostic_chip(QT_REFERENCE_MODE);

    diagnostic_addr = get_object_address(DEBUG_DIAGNOSTIC_T37, 0);
    diagnostic_size = get_object_size(DEBUG_DIAGNOSTIC_T37);

    read_mem(diagnostic_addr,2, &dbg->mode);

    do
    {
        status = read_mem(diagnostic_addr,2, & dbg->mode);

        if(status == READ_MEM_OK)
        {
            if(dbg->page > 128)
            {
                if(page > 128)
                {
                    if(dbg->page < page)
                    {
                        diagnostic_chip(QT_PAGE_UP);
                    }
                    else if(dbg->page > page)
                    {
                        diagnostic_chip(QT_PAGE_DOWN);
                    }
                }
                else
                {
                    if(dbg->page > page)
                    {
                        diagnostic_chip(QT_PAGE_UP);
                    }
                    else if(dbg->page < page)
                    {
                        diagnostic_chip(QT_PAGE_DOWN);
                    }
                }
            }
            else
            {
                if(page < 128)
                {
                    if(dbg->page < page)
                    {
                        diagnostic_chip(QT_PAGE_UP);
                    }
                    else if(dbg->page > page)
                    {
                        diagnostic_chip(QT_PAGE_DOWN);
                    }
                }
                else
                {
                    if(dbg->page > page)
                    {
                        diagnostic_chip(QT_PAGE_UP);
                    }
                    else if(dbg->page < page)
                    {
                        diagnostic_chip(QT_PAGE_DOWN);
                    }
                }
            }
        }
        delay_ms(1);
    }while(dbg->page != page);

    status = read_mem(diagnostic_addr,diagnostic_size, &dbg->mode);

    return status;
}

