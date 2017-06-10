////////////////////////////////////////////////////////////////////////////////
// ��Ȩ:     ������Ƽ����Źɷ����޹�˾
// �ļ���:   
// �汾��    v1.0
// ��������: IAR  v5.30
// ����:     �ں���
// ��������: 2013.12
// ����:       SPI
// ����ļ�: 
// �޸���־��
////////////////////////////////////////////////////////////////////////////////
#include "main.h"

#include "SX1276_Hal.h"

////////////////////////////////////////////////////////////////////////////////
// �������� : RF��ʼ��IO��
// ������� : ��
// ���ز��� : ��
// ˵��     : 
////////////////////////////////////////////////////////////////////////////////

//void Init_LoraIO_CS_LP(void)//PB5
//{	
//    GPIO_InitTypeDef g;    
//    //GPIOB
//    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);

//    g.GPIO_Pin = GPIO_Pin_5;
//    g.GPIO_Mode = GPIO_Mode_IPU;       
//    GPIO_Init(GPIOB, &g);
//}
//void Init_LoraIO_RST_LP(void)//PB1
//{	
//    GPIO_InitTypeDef g;    
//    //GPIOB
//    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);

//    g.GPIO_Pin = GPIO_Pin_1;
//    g.GPIO_Mode = GPIO_Mode_IPU; 
//    GPIO_Init(GPIOB, &g);
//}
//void Init_LoraIO_DIO1_LP(void)//PA12  �ж�
//{

//    GPIO_InitTypeDef g;    
//    //GPIOA
//    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);

//    g.GPIO_Pin = GPIO_Pin_12;	
//    g.GPIO_Mode = GPIO_Mode_IPU;       
//    GPIO_Init(GPIOA, &g);
//}
//void Init_LoraIO_DIO2_LP(void)//PA12  �ж�
//{

//    GPIO_InitTypeDef g;    
//    //GPIOA
//    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);

//    g.GPIO_Pin = GPIO_Pin_11;	
//    g.GPIO_Mode = GPIO_Mode_IPU;       
//    GPIO_Init(GPIOA, &g);
//}

//void Init_LoraIO_DIO3_LP(void)//PA8
//{	  
//    GPIO_InitTypeDef g;    
//    //GPIOA
//    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);

//    g.GPIO_Pin = GPIO_Pin_8;	
//    g.GPIO_Mode = GPIO_Mode_IPU;       
//    GPIO_Init(GPIOA, &g);
//}

//void Init_LoraIO_DIO0_LP(void)//PB12  �ж�
//{  
//    GPIO_InitTypeDef g;    
//    //GPIOB
//    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);

//    g.GPIO_Pin = GPIO_Pin_12;	
//    g.GPIO_Mode = GPIO_Mode_IPU;       
//    GPIO_Init(GPIOB, &g);
//}

//void Init_LoraIO_SPI_LP(void)//PB13-14-15
//{  
//    GPIO_InitTypeDef g;    
//    //GPIOB
//    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);

//    g.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;	
//    g.GPIO_Mode = GPIO_Mode_IPU;       
//    GPIO_Init(GPIOB, &g);
//}

//void Init_LoraIO_LP(void)//
//{  
//	Init_LoraIO_CS_LP();
//	Init_LoraIO_RST_LP();
//	Init_LoraIO_DIO1_LP();
//	//Init_LoraIO_DIO2_LP();
//	Init_LoraIO_DIO3_LP();
//	Init_LoraIO_DIO0_LP();
//	//Init_LoraIO_SPI_LP();
//}



void init_lora1_cs(void)//PA15
{	
	GPIO_InitTypeDef g;    
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);					  											 
	g.GPIO_Pin = GPIO_Pin_15;                 
	g.GPIO_Mode = GPIO_Mode_OUT; 
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &g);
}

void init_lora2_cs(void)//PF6
{	
    GPIO_InitTypeDef g;    
    
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOF, ENABLE);					  											 
	g.GPIO_Pin = GPIO_Pin_6;                 
	g.GPIO_Mode = GPIO_Mode_OUT; 
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &g);

	set_lora2_cs();
}

void set_lora1_cs(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_15);
}

void reset_lora1_cs(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_15);
}

void set_lora2_cs(void)
{
	GPIO_SetBits(GPIOF,GPIO_Pin_6);
}

void reset_lora2_cs(void)
{
	GPIO_ResetBits(GPIOF,GPIO_Pin_6);
}


void init_lora1_rst(void)//PA11
{	
	GPIO_InitTypeDef g;    

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);					  											 
	g.GPIO_Pin = GPIO_Pin_11;              
	g.GPIO_Mode = GPIO_Mode_OUT; 
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &g);
}

void init_lora2_rst(void)//PA12
{	
	GPIO_InitTypeDef g;    

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);					  											 
	g.GPIO_Pin = GPIO_Pin_12;              
	g.GPIO_Mode = GPIO_Mode_OUT; 
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &g);
}


void set_lora1_rst(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_11);
}

void reset_lora1_rst(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_11);
}

void set_lora2_rst(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_12);
}

void reset_lora2_rst(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_12);
}



void Init_LoraIO_DIO1(void)//PB10
{
//    NVIC_InitTypeDef NVIC_InitStructure;  
//    EXTI_InitTypeDef EXTI_InitStructure;  
    GPIO_InitTypeDef g;    
    //GPIOA
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);					  											 
	g.GPIO_Pin = GPIO_Pin_10;                 
	g.GPIO_Mode = GPIO_Mode_IN; 
	g.GPIO_PuPd = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &g);


//	EXTI_ClearITPendingBit(EXTI_Line12);  
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource12);//PC12  ΪGPIOA��PIN12  
//	EXTI_InitStructure.EXTI_Line= EXTI_Line12; //PC12��Ϊ��EXTI_Line12	
//	EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;	 
//	EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Rising;	//�жϷ�ʽΪ�������½���  
//	EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
//	EXTI_Init(&EXTI_InitStructure);  
//		  
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   //NVIC  
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;  
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;  
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2;		   
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;	  
//	NVIC_Init(&NVIC_InitStructure);   
	
}


void Init_LoraIO_DIO1_EXTIoff(void)//PB10  ���ж�
{
//    NVIC_InitTypeDef NVIC_InitStructure;  
//    EXTI_InitTypeDef EXTI_InitStructure;  
    GPIO_InitTypeDef g;    
    //GPIOA
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);					  											 
	g.GPIO_Pin = GPIO_Pin_10;                 
	g.GPIO_Mode = GPIO_Mode_IN; 
	g.GPIO_PuPd = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &g);


//	EXTI_ClearITPendingBit(EXTI_Line12);  
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource12);//PC12  ΪGPIOA��PIN12  
//	EXTI_InitStructure.EXTI_Line= EXTI_Line12; //PC12��Ϊ��EXTI_Line12	
//	EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;	 
//	EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Rising;	//�жϷ�ʽΪ�������½���  
//	EXTI_InitStructure.EXTI_LineCmd=DISABLE;  
//	EXTI_Init(&EXTI_InitStructure);  
//		  
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   //NVIC  
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;  
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;  
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2;		   
//	NVIC_InitStructure.NVIC_IRQChannelCmd=DISABLE;	  
//	NVIC_Init(&NVIC_InitStructure);   
	
}

void Init_LoraIO_DIO3(void)//PB2
{
	GPIO_InitTypeDef g; //GPIO ����
	EXTI_InitTypeDef EXTI_InitStructure; //�ⲿ�ж϶���
	NVIC_InitTypeDef NVIC_InitStructure; //Ƕ���ж϶���
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);					  											 
	g.GPIO_Pin = GPIO_Pin_2;                 
	g.GPIO_Mode = GPIO_Mode_IN; 
	g.GPIO_PuPd = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &g);

	#if 0

    //��EXTI0ָ��PB2  
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource2);  
    //EXTI0�ж�������  
//    EXTI_InitTypeDef EXTI_InitStructure;  
    EXTI_InitStructure.EXTI_Line=EXTI_Line2;  
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;  //����ָ�ߵ�ƽ
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
    EXTI_Init(&EXTI_InitStructure);  

    //EXTI0�ж���������  
//    NVIC_InitTypeDef NVIC_InitStructure;  
    NVIC_InitStructure.NVIC_IRQChannel=EXTI2_3_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPriority=0x01;  
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  
    NVIC_Init(&NVIC_InitStructure);  
	#endif
}


void Init_LoraIO_DIO3_EXTIoff(void)//PB2
{
	GPIO_InitTypeDef g; //GPIO ����
	EXTI_InitTypeDef EXTI_InitStructure; //�ⲿ�ж϶���
	NVIC_InitTypeDef NVIC_InitStructure; //Ƕ���ж϶���
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);																 
	g.GPIO_Pin = GPIO_Pin_2;				  
	g.GPIO_Mode = GPIO_Mode_IN; 
	g.GPIO_PuPd = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &g);


#if 0
	//��EXTI0ָ��PB2  
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource2);  
	//EXTI0�ж�������  
//	  EXTI_InitTypeDef EXTI_InitStructure;	
	EXTI_InitStructure.EXTI_Line=EXTI_Line2;  
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;  //����ָ�ߵ�ƽ
	EXTI_InitStructure.EXTI_LineCmd=DISABLE;  
	EXTI_Init(&EXTI_InitStructure);  

	//EXTI0�ж���������  
//	  NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel=EXTI2_3_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPriority=0x01;  
	NVIC_InitStructure.NVIC_IRQChannelCmd=DISABLE;  
	NVIC_Init(&NVIC_InitStructure);  
#endif
}


void Init_LoraIO_DIO0(void)//PB11  �ж�
{
	GPIO_InitTypeDef g; //GPIO ����
	EXTI_InitTypeDef EXTI_InitStructure; //�ⲿ�ж϶���
	NVIC_InitTypeDef NVIC_InitStructure; //Ƕ���ж϶���
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);																 
	g.GPIO_Pin = GPIO_Pin_11;				  
	g.GPIO_Mode = GPIO_Mode_IN; 
	g.GPIO_PuPd = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &g);

	//��EXTI0ָ��PB11
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource11);  
	//EXTI0�ж�������  
//	  EXTI_InitTypeDef EXTI_InitStructure;	
	EXTI_InitStructure.EXTI_Line=EXTI_Line11;  
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;  //����ָ�ߵ�ƽ
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
	EXTI_Init(&EXTI_InitStructure);  

	//EXTI0�ж���������  
//	  NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel=EXTI4_15_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPriority=0x01;  
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  
	NVIC_Init(&NVIC_InitStructure);  
}


void Init_LoraIO_DIO0_EXTIoff(void)//PB11  �ж�
{
	GPIO_InitTypeDef g; //GPIO ����
	EXTI_InitTypeDef EXTI_InitStructure; //�ⲿ�ж϶���
	NVIC_InitTypeDef NVIC_InitStructure; //Ƕ���ж϶���
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);																 
	g.GPIO_Pin = GPIO_Pin_11;				  
	g.GPIO_Mode = GPIO_Mode_IN; 
	g.GPIO_PuPd = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &g);

	//��EXTI0ָ��PB11  
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource11);  
	//EXTI0�ж�������  
//	  EXTI_InitTypeDef EXTI_InitStructure;	
	EXTI_InitStructure.EXTI_Line=EXTI_Line11;  
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;  //����ָ�ߵ�ƽ
	EXTI_InitStructure.EXTI_LineCmd=DISABLE;  
	EXTI_Init(&EXTI_InitStructure);  

	//EXTI0�ж���������  
//	  NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel=EXTI4_15_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPriority=0x01;  
	NVIC_InitStructure.NVIC_IRQChannelCmd=DISABLE;	
	NVIC_Init(&NVIC_InitStructure);  
}

void SX1276InitIo( void )
{
 
	init_lora1_cs();
	init_lora2_cs();
	Init_LoraIO_DIO1();
	Init_LoraIO_DIO3();
	Init_LoraIO_DIO0();

	init_lora1_rst();

}


////////////////////////////////////////////////////////////////////////////////
// �������� : RF  SPI���ó�ʼ��
// ������� : ��
// ���ز��� : ��
// ˵��     : �˳�ʼ��Ϊ430Ӳ��SPI��ʼ��
////////////////////////////////////////////////////////////////////////////////
void SX1276SPISetup(void)
{
    //SPI_NSS_SET = 1;v1.0
//    SPI_NSS_DIR = 1;
//    SPI_NSS_OUT = 1;         // /CS disable
    //��̬ΪLOW   ��һ���仯�ز���
    set_lora1_cs();
}
////////////////////////////////////////////////////////////////////////////////
// �������� : RF   ��λ
// ������� : ��
// ���ز��� : ��
// ˵��     :
////////////////////////////////////////////////////////////////////////////////

void SX1276Reset(void)
{
	reset_lora1_rst();
	delay_ms(100);  
	set_lora1_rst();
	delay_ms(20); 	
}
////////////////////////////////////////////////////////////////////////////////
// �������� : RF  ��Ĵ�����ַ������������
// ������� : uint8_t addr,�Ĵ�����ַ uint8_t *buffer,��������ָ�� uint8_t sizeָ�볤��
// ���ز��� : ��
// ˵��     :
////////////////////////////////////////////////////////////////////////////////
void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;  

    //SPI_NSS_OUT = 0;
    reset_lora1_cs();
    
     SPI_SendData(addr | 0x80);
    for( i = 0; i < size; i++ )
        SPI_SendData(buffer[i]);

    //SPI_NSS_OUT = 1;
    delay_us(50);
    set_lora1_cs();
}
////////////////////////////////////////////////////////////////////////////////
// �������� : RF  ��Ĵ�����ַ����������
// ������� : uint8_t addr,�Ĵ�����ַ uint8_t *buffer,�洢����ָ�� uint8_t sizeҪ���ĳ���
// ���ز��� : ���ݷ��ص�*buffer��
// ˵��     :
////////////////////////////////////////////////////////////////////////////////
void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
    
    //SPI_NSS_OUT = 0;
    reset_lora1_cs();
    
    SPI_SendData(addr & 0x7F);
    for( i = 0; i < size; i++ )
        buffer[i] = SPI_ReceiveData();

    //SPI_NSS_OUT = 1;
    delay_us(50);
    set_lora1_cs();
}
////////////////////////////////////////////////////////////////////////////////
// �������� : RF  ��Ĵ�����ַд1�ֽ�����
// ������� : uint8_t addr,�Ĵ�����ַ uint8_t data����
// ���ز��� : 
// ˵��     :
////////////////////////////////////////////////////////////////////////////////
void SX1276Write( uint8_t addr, uint8_t data )
{
    SX1276WriteBuffer( addr, &data, 1 );
}
////////////////////////////////////////////////////////////////////////////////
// �������� : RF  ��Ĵ�����ַ��1�ֽ�����
// ������� : uint8_t addr,�Ĵ�����ַ uint8_t *data�����ݴ洢��ַ
// ���ز��� : 
// ˵��     :
////////////////////////////////////////////////////////////////////////////////
void SX1276Read( uint8_t addr, uint8_t *data )
{
    SX1276ReadBuffer( addr, data, 1 );
}
////////////////////////////////////////////////////////////////////////////////
// �������� : RF  ��FIFOд����
// ������� : uint8_t *buffer,����ָ�� uint8_t size����
// ���ز��� : 
// ˵��     :
////////////////////////////////////////////////////////////////////////////////
void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1276WriteBuffer( 0, buffer, size );
}
////////////////////////////////////////////////////////////////////////////////
// �������� : RF  ��FIFO������
// ������� : uint8_t *buffer,����ָ�� uint8_t size����
// ���ز��� : uint8_t *buffer �洢��ȡ����
// ˵��     :
////////////////////////////////////////////////////////////////////////////////
void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1276ReadBuffer( 0, buffer, size );
}
////////////////////////////////////////////////////////////////////////////////
// �������� : RF  TX/RX��PA�л�
// ������� :  bool txEnable  �л��߼�
// ���ز��� : ��
// ˵��     :�棺��ΪTX���٣���ΪRX   ΪӲ������PA����IO��
////////////////////////////////////////////////////////////////////////////////
/*void SX1276WriteRxTx( bool txEnable )
{
    if( txEnable != 0 )       //���Ϊ�棬ΪTX
    {
        PA_RX_OUT = 0;        //PA_RXΪ0
        PA_TX_OUT = 1;        //PA_TXΪ1
    }
    else  //Ϊ�٣�ΪRX
    {
        PA_RX_OUT = 1;        //PA_RXΪ1
        PA_TX_OUT = 0;         //PA_TXΪ0
    }
}*/
////////////////////////////////////////////////////////////////////////////////

void SPI_SendData(UINT8 Data)
{
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
    SPI_SendData8(SPI2,Data);
}
UINT8 SPI_ReceiveData(void)
{
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
    SPI_SendData8(SPI2,0);
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)==RESET);
	return (SPI_ReceiveData8(SPI2));
}



