/*
    (c) 2020 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
 */

#pragma config WDTE = OFF /* WDT operating mode->WDT Disabled */
#pragma config LVP = ON   /* Low-voltage programming enabled, RE3 pin is MCLR */

#define _XTAL_FREQ                      64000000UL

#include <pic18.h>
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define I2C_SLAVE_ADDRESS               0x20
#define MCP23008_REG_ADDR_IODIR         0x00
#define MCP23008_REG_ADDR_GPIO          0x09
#define I2C_RW_BIT                      0x01
#define MCP23008_DATA                   0x0F
#define PINS_DIGITAL_OUTPUT             0x00

/* SYSTEM initialization functions */
static void CLK_init(void);
static void PPS_init(void);
static void PORT_init(void);
static void I2C1_init(void);
static void INTERRUPT_init(void);

/* Driver functions */
static uint8_t I2C1_open(void);
static void I2C1_close(void);
static void I2C1_startCondition(void);
static void I2C1_stopCondition(void);
static uint8_t I2C1_getAckstatBit(void);
static void I2C1_sendNotAcknowledge(void);
static void I2C1_setReceiveMode(void);
static void I2C1_write1ByteRegister(uint8_t address, uint8_t reg, uint8_t data);
static uint8_t I2C1_read1ByteRegister(uint8_t address, uint8_t reg);

/* State functions for interrupt handler */
static void I2C_stateWriteStartComplete(void);
static void I2C_stateWriteAddressSent(void);
static void I2C_stateWriteRegisterSent(void);
static void I2C_stateWriteDataSent(void);
static void I2C_stateReadInit(void);
static void I2C_stateReadStartComplete(void);
static void I2C_stateReadAddressSent(void);
static void I2C_stateReadReceiveEnable(void);
static void I2C_stateReadDataComplete(void);
static void I2C_stateStopComplete(void);

static void MSSP1_interruptHandler(void);

typedef enum {
    I2C_WRITE_START = 0,
    I2C_WRITE_ADDRESS_SENT,
    I2C_REGISTER_SENT,
    I2C_WRITE_DATA_SENT,
    I2C_READ_INIT,
    I2C_READ_START_COMPLETE,
    I2C_READ_ADDRESS_SENT,
    I2C_RECEIVE_ENABLE,
    I2C_READ_DATA_COMPLETE,
    I2C_STOP,
    I2C_IDLE
} i2c1_states_t;

void (*I2C_stateFuncs[10])() = {
    I2C_stateWriteStartComplete,    /* I2C_WRITE_START         */
    I2C_stateWriteAddressSent,      /* I2C_WRITE_ADDRESS_SENT  */
    I2C_stateWriteRegisterSent,     /* I2C_REGISTER_SENT       */
    I2C_stateWriteDataSent,         /* I2C_WRITE_DATA_SENT     */
    I2C_stateReadInit,              /* I2C_READ_INIT           */
    I2C_stateReadStartComplete,     /* I2C_READ_START_COMPLETE */
    I2C_stateReadAddressSent,       /* I2C_READ_ADDRESS_SENT   */
    I2C_stateReadReceiveEnable,     /* I2C_RECEIVE_ENABLE      */
    I2C_stateReadDataComplete,      /* I2C_READ_DATA_COMPLETE  */
    I2C_stateStopComplete,          /* I2C_STOP                */
};

struct status {
    volatile i2c1_states_t state;
    uint8_t address;
    uint8_t reg;
    volatile uint8_t data;
    bool read;
} I2C1_status;

/*************************** SYSTEM INITIALIZATION ****************************/

static void CLK_init(void)
{
    /* Set Oscilator Source: HFINTOSC and Set Clock Divider: 1 */
    OSCCON1bits.NOSC = 0x6;

    /* Set Nominal Freq: 64 MHz */
    OSCFRQbits.FRQ3 = 1;
}

static void PPS_init(void)
{
    /* PPS setting for using RB1 as SCL */
    SSP1CLKPPS = 0x09;
    RB1PPS = 0x0F;

    /* PPS setting for using RB2 as SDA */
    SSP1DATPPS = 0x0A;
    RB2PPS = 0x10;
}

static void PORT_init(void)
{
    /* Set pins RB1 and RB2 as Digital */
    ANSELBbits.ANSELB1 = 0;
    ANSELBbits.ANSELB2 = 0;
    
    /* Set pull-up resistorsfor RB1 and RB2 */
    WPUBbits.WPUB1 = 1;
    WPUBbits.WPUB2 = 1;
}

static void I2C1_init(void)
{
    /* I2C Master Mode: Clock = F_OSC / (4 * (SSP1ADD + 1)) */
    SSP1CON1bits.SSPM3 = 1;
    
    /* Set the boud rate devider to obtain the I2C clock at 100000 Hz*/
    SSP1ADD  = 0x9F;
}

static void INTERRUPT_init(void)
{
    /* Enable the Global Interrupts */
    INTCONbits.GIE = 1;
    /* Enable the Peripheral Interrupts */
    INTCONbits.PEIE = 1;
}

/******************************** I2C Driver **********************************/

static uint8_t I2C1_open(void)
{
    if(I2C1_status.state != I2C_IDLE)  
    {
        /* Return 0 if failed opening I2C */
        return 0;
    }
    
    /* Clear IRQ */
    PIR3bits.SSP1IF = 0;

    /* Enable SSP1 Interrupts */
    PIE3bits.SSP1IE = 1;
    
    /* I2C Master Open */
    SSP1CON1bits.SSPEN = 1;

    /* Return 1 if successfully opened I2C */
    return 1;
}

static void I2C1_close(void)
{
    /* Disable I2C1 */
    SSP1CON1bits.SSPEN = 0;

    /* Disable SSP1 Interrupts */
    PIE3bits.SSP1IE = 0;
}

static void I2C1_startCondition(void)
{
    /* START Condition*/
    SSP1CON2bits.SEN = 1;
}

static void I2C1_stopCondition(void)
{
    /* STOP Condition */
    SSP1CON2bits.PEN = 1;
}

static uint8_t I2C1_getAckstatBit(void)
{
    /* Return ACKSTAT bit */
    return SSP1CON2bits.ACKSTAT;
}

static void I2C1_sendNotAcknowledge(void)
{
    /* Send NACK bit to slave */
    SSP1CON2bits.ACKDT = 1;
    SSP1CON2bits.ACKEN = 1;
}

static void I2C1_setReceiveMode(void)
{
    /* Start receiving mode */
    SSP1CON2bits.RCEN = 1;
}

static void I2C1_write1ByteRegister(uint8_t address, uint8_t reg, uint8_t data)
{
    while (!I2C1_open())
    {
        ;
    }

    I2C1_status.state = I2C_WRITE_START;
    I2C1_status.address = address;
    I2C1_status.reg = reg;
    I2C1_status.data = data;
    I2C1_status.read = false;

    I2C1_startCondition();
    
    while (I2C1_status.state != I2C_IDLE);
}

static uint8_t I2C1_read1ByteRegister(uint8_t address, uint8_t reg)
{
    while (!I2C1_open())
    {
        ;
    }

    I2C1_status.state = I2C_WRITE_START;
    I2C1_status.address = address;
    I2C1_status.reg = reg;
    I2C1_status.data = 0;
    I2C1_status.read = true;

    I2C1_startCondition();
    
    while (I2C1_status.state != I2C_IDLE);
    
    return I2C1_status.data;
}

/************************** I2C Interrupt States ******************************/

static void I2C_stateWriteStartComplete(void)
{
    I2C1_status.state = I2C_WRITE_ADDRESS_SENT;
    
    /* Shift the 7 bit address and add a 0 bit to indicate write operation */
    SSP1BUF = (I2C1_status.address << 1) & ~I2C_RW_BIT;
}

static void I2C_stateWriteAddressSent(void)
{
    /* Check the ACK bit and exit the function if its not acknowledge bit */
    if (I2C1_getAckstatBit())
    {
        I2C1_status.state = I2C_IDLE;
        return ;
    }
    
    if (I2C1_status.read)
    {
        I2C1_status.state = I2C_READ_INIT;
    }
    else
    {
        I2C1_status.state = I2C_REGISTER_SENT;
    }
    
    SSP1BUF = I2C1_status.reg;
}

static void I2C_stateWriteRegisterSent(void)
{
    /* Check the ACK bit and exit the function if its not acknowledge bit */
    if (I2C1_getAckstatBit())
    {
        I2C1_status.state = I2C_IDLE;
        return ;
    }
    
    I2C1_status.state = I2C_WRITE_DATA_SENT;
    
    SSP1BUF = I2C1_status.data;
}

static void I2C_stateWriteDataSent(void)
{
    /* Check the ACK bit and exit the function if its not acknowledge bit */
    if (I2C1_getAckstatBit())
    {
        I2C1_status.state = I2C_IDLE;
        return ;
    }
    
    I2C1_status.state = I2C_STOP;
    
    I2C1_stopCondition();
}

static void I2C_stateReadInit(void)
{
    I2C1_status.state = I2C_READ_START_COMPLETE;
    
    /* Send start condition for the read operation */
    I2C1_startCondition();
}

static void I2C_stateReadStartComplete(void)
{
    I2C1_status.state = I2C_READ_ADDRESS_SENT;
    
    /* Shift the 7 bit address and add a 1 bit to indicate read operation */
    SSP1BUF = (I2C1_status.address << 1) | I2C_RW_BIT;
}

static void I2C_stateReadAddressSent(void)
{
    /* Check the ACK bit and exit the function if its not acknowledge bit */
    if (I2C1_getAckstatBit())
    {
        I2C1_status.state = I2C_IDLE;
        return ;
    }
    
    I2C1_status.state = I2C_RECEIVE_ENABLE;
    I2C1_setReceiveMode();
}

static void I2C_stateReadReceiveEnable(void)
{
    I2C1_status.state = I2C_READ_DATA_COMPLETE;
    
    I2C1_status.data = SSP1BUF;
    
    /* Send NACK bit to stop receiving mode */
    I2C1_sendNotAcknowledge();
}

static void I2C_stateReadDataComplete(void)
{
    I2C1_status.state = I2C_STOP;
    I2C1_stopCondition();
}

static void I2C_stateStopComplete(void)
{
    I2C1_status.state = I2C_IDLE;
    I2C1_close();
}

static void MSSP1_interruptHandler(void)
{
    /* Call the function associated with the current state */
    I2C_stateFuncs[I2C1_status.state]();
    
    /* Clear Interrupt Flag */
    PIR3bits.SSP1IF = 0;
}

void __interrupt() INTERRUPT_InterruptManager (void)
{
    if(INTCONbits.PEIE == 1)
    {
        if(PIE3bits.SSP1IE == 1 && PIR3bits.SSP1IF == 1)
        {
            MSSP1_interruptHandler();
        }
    }
}

void main(void)
{
    CLK_init();
    PPS_init();
    PORT_init();
    I2C1_init();
    INTERRUPT_init();
    
    /* Set the initial state to IDLE */
    I2C1_status.state = I2C_IDLE;
    /* Set data to use in the I2C operations */
    uint8_t data = MCP23008_DATA;
    /* Set the extended pins as digital output */
    I2C1_write1ByteRegister(I2C_SLAVE_ADDRESS, MCP23008_REG_ADDR_IODIR, PINS_DIGITAL_OUTPUT);
    
    while (1)
    {
        /* Write data to the GPIO port */
        I2C1_write1ByteRegister(I2C_SLAVE_ADDRESS, MCP23008_REG_ADDR_GPIO, data);
        /* Read data from the GPIO port */
        data = I2C1_read1ByteRegister(I2C_SLAVE_ADDRESS, MCP23008_REG_ADDR_GPIO);
        /* Overwrite data with the inverted data read */
        data = ~data;

        __delay_ms(500);
	}
}
