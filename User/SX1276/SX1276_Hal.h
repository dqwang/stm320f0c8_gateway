#ifndef __SX1276_HAL_H__
#define __SX1276_HAL_H__

#include "main.h"



/*lora1*/
void init_lora1_cs(void);
void set_lora1_cs(void);
void reset_lora1_cs(void);

void init_lora1_rst(void);
void set_lora1_rst(void);
void reset_lora1_rst(void);



/*lora2*/
void init_lora2_cs(void);
void set_lora2_cs(void);
void reset_lora2_cs(void);

void init_lora2_rst(void);
void set_lora2_rst(void);
void reset_lora2_rst(void);


//===================================宏定义===================================================
// RXTX pin control see errata note
//#define RXTX( txEnable )                            SX1276WriteRxTx( txEnable );
/*!
 * DIO state read functions mapping
 */
//#define DIO0                                        SX1276ReadDio0( )
//#define DIO1                                        SX1276ReadDio1( )
//#define DIO2                                        SX1276ReadDio2( )
//#define DIO3                                        SX1276ReadDio3( )
//#define DIO4                                        SX1276ReadDio4( )
//#define DIO5                                        SX1276ReadDio5( )

#define RF_R_DIO0  		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)//GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)
#define RF_R_DIO1  		GPIO_ReadInputDataBit(GPIOb,GPIO_Pin_10)//GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)
//#define RF_R_DIO2  		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_)
#define RF_R_DIO3  		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)//GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8)

//0,1,3 needed

//#define SX1276ReadDio0( )   DIO0_IN
//#define SX1276ReadDio1( )   DIO1_IN
//#define SX1276ReadDio2( )   1         //未使用
//#define SX1276ReadDio3( )   DIO3_IN
//#define SX1276ReadDio4( )   1         //未使用

typedef enum
{
    RADIO_RESET_OFF,
    RADIO_RESET_ON,
}tRadioResetState;

//===================================SPI函数声明===================================================
void SX1276InitIo( void );
void SX1276SPISetup(void);
void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );
void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );
void SX1276Write( uint8_t addr, uint8_t data );
void SX1276Read( uint8_t addr, uint8_t *data );
void SX1276WriteFifo( uint8_t *buffer, uint8_t size );
void SX1276ReadFifo( uint8_t *buffer, uint8_t size );
void SX1276Reset(void);
//void SX1276WriteRxTx( bool txEnable );

void Init_LoraIO_DIO0(void);
void Init_LoraIO_DIO1(void);
void Init_LoraIO_DIO3(void);
void Init_LoraIO_DIO0_EXTIoff(void);
void Init_LoraIO_DIO1_EXTIoff(void);
void Init_LoraIO_DIO3_EXTIoff(void);


void SPI_SendData(UINT8 Data);
UINT8 SPI_ReceiveData(void);

#endif 
