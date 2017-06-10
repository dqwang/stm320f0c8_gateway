////////////////////////////////////////////////////////////////////////////////
// 版权:     利尔达科技集团股份有限公司
// 文件名:   
// 版本：    v1.0
// 工作环境: IAR  v5.30
// 作者:     于海波
// 生成日期: 2013.12
// 功能:       SPI
// 相关文件: 
// 修改日志：
////////////////////////////////////////////////////////////////////////////////
#include "main.h"

#include "SX1276_Hal.h"

////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF初始化IO口
// 输入参数 : 无
// 返回参数 : 无
// 说明     : 
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
//void Init_LoraIO_DIO1_LP(void)//PA12  中断
//{

//    GPIO_InitTypeDef g;    
//    //GPIOA
//    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);

//    g.GPIO_Pin = GPIO_Pin_12;	
//    g.GPIO_Mode = GPIO_Mode_IPU;       
//    GPIO_Init(GPIOA, &g);
//}
//void Init_LoraIO_DIO2_LP(void)//PA12  中断
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

//void Init_LoraIO_DIO0_LP(void)//PB12  中断
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
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource12);//PC12  为GPIOA的PIN12  
//	EXTI_InitStructure.EXTI_Line= EXTI_Line12; //PC12，为：EXTI_Line12	
//	EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;	 
//	EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Rising;	//中断方式为上升与下降沿  
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


void Init_LoraIO_DIO1_EXTIoff(void)//PB10  关中断
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
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource12);//PC12  为GPIOA的PIN12  
//	EXTI_InitStructure.EXTI_Line= EXTI_Line12; //PC12，为：EXTI_Line12	
//	EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;	 
//	EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Rising;	//中断方式为上升与下降沿  
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
	GPIO_InitTypeDef g; //GPIO 定义
	EXTI_InitTypeDef EXTI_InitStructure; //外部中断定义
	NVIC_InitTypeDef NVIC_InitStructure; //嵌套中断定义
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);					  											 
	g.GPIO_Pin = GPIO_Pin_2;                 
	g.GPIO_Mode = GPIO_Mode_IN; 
	g.GPIO_PuPd = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &g);

	#if 0

    //将EXTI0指向PB2  
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource2);  
    //EXTI0中断线配置  
//    EXTI_InitTypeDef EXTI_InitStructure;  
    EXTI_InitStructure.EXTI_Line=EXTI_Line2;  
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;  //有手指高电平
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
    EXTI_Init(&EXTI_InitStructure);  

    //EXTI0中断向量配置  
//    NVIC_InitTypeDef NVIC_InitStructure;  
    NVIC_InitStructure.NVIC_IRQChannel=EXTI2_3_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPriority=0x01;  
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  
    NVIC_Init(&NVIC_InitStructure);  
	#endif
}


void Init_LoraIO_DIO3_EXTIoff(void)//PB2
{
	GPIO_InitTypeDef g; //GPIO 定义
	EXTI_InitTypeDef EXTI_InitStructure; //外部中断定义
	NVIC_InitTypeDef NVIC_InitStructure; //嵌套中断定义
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);																 
	g.GPIO_Pin = GPIO_Pin_2;				  
	g.GPIO_Mode = GPIO_Mode_IN; 
	g.GPIO_PuPd = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &g);


#if 0
	//将EXTI0指向PB2  
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource2);  
	//EXTI0中断线配置  
//	  EXTI_InitTypeDef EXTI_InitStructure;	
	EXTI_InitStructure.EXTI_Line=EXTI_Line2;  
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;  //有手指高电平
	EXTI_InitStructure.EXTI_LineCmd=DISABLE;  
	EXTI_Init(&EXTI_InitStructure);  

	//EXTI0中断向量配置  
//	  NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel=EXTI2_3_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPriority=0x01;  
	NVIC_InitStructure.NVIC_IRQChannelCmd=DISABLE;  
	NVIC_Init(&NVIC_InitStructure);  
#endif
}


void Init_LoraIO_DIO0(void)//PB11  中断
{
	GPIO_InitTypeDef g; //GPIO 定义
	EXTI_InitTypeDef EXTI_InitStructure; //外部中断定义
	NVIC_InitTypeDef NVIC_InitStructure; //嵌套中断定义
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);																 
	g.GPIO_Pin = GPIO_Pin_11;				  
	g.GPIO_Mode = GPIO_Mode_IN; 
	g.GPIO_PuPd = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &g);

	//将EXTI0指向PB11
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource11);  
	//EXTI0中断线配置  
//	  EXTI_InitTypeDef EXTI_InitStructure;	
	EXTI_InitStructure.EXTI_Line=EXTI_Line11;  
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;  //有手指高电平
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
	EXTI_Init(&EXTI_InitStructure);  

	//EXTI0中断向量配置  
//	  NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel=EXTI4_15_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPriority=0x01;  
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  
	NVIC_Init(&NVIC_InitStructure);  
}


void Init_LoraIO_DIO0_EXTIoff(void)//PB11  中断
{
	GPIO_InitTypeDef g; //GPIO 定义
	EXTI_InitTypeDef EXTI_InitStructure; //外部中断定义
	NVIC_InitTypeDef NVIC_InitStructure; //嵌套中断定义
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);																 
	g.GPIO_Pin = GPIO_Pin_11;				  
	g.GPIO_Mode = GPIO_Mode_IN; 
	g.GPIO_PuPd = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &g);

	//将EXTI0指向PB11  
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource11);  
	//EXTI0中断线配置  
//	  EXTI_InitTypeDef EXTI_InitStructure;	
	EXTI_InitStructure.EXTI_Line=EXTI_Line11;  
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;  //有手指高电平
	EXTI_InitStructure.EXTI_LineCmd=DISABLE;  
	EXTI_Init(&EXTI_InitStructure);  

	//EXTI0中断向量配置  
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
// 功能描述 : RF  SPI设置初始化
// 输入参数 : 无
// 返回参数 : 无
// 说明     : 此初始化为430硬件SPI初始化
////////////////////////////////////////////////////////////////////////////////
void SX1276SPISetup(void)
{
    //SPI_NSS_SET = 1;v1.0
//    SPI_NSS_DIR = 1;
//    SPI_NSS_OUT = 1;         // /CS disable
    //常态为LOW   第一个变化沿采样
    set_lora1_cs();
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF   复位
// 输入参数 : 无
// 返回参数 : 无
// 说明     :
////////////////////////////////////////////////////////////////////////////////

void SX1276Reset(void)
{
	reset_lora1_rst();
	delay_ms(100);  
	set_lora1_rst();
	delay_ms(20); 	
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF  向寄存器地址连续发送数据
// 输入参数 : uint8_t addr,寄存器地址 uint8_t *buffer,发送数组指针 uint8_t size指针长度
// 返回参数 : 无
// 说明     :
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
// 功能描述 : RF  向寄存器地址连续读数据
// 输入参数 : uint8_t addr,寄存器地址 uint8_t *buffer,存储数组指针 uint8_t size要读的长度
// 返回参数 : 数据返回到*buffer中
// 说明     :
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
// 功能描述 : RF  向寄存器地址写1字节数据
// 输入参数 : uint8_t addr,寄存器地址 uint8_t data数据
// 返回参数 : 
// 说明     :
////////////////////////////////////////////////////////////////////////////////
void SX1276Write( uint8_t addr, uint8_t data )
{
    SX1276WriteBuffer( addr, &data, 1 );
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF  向寄存器地址读1字节数据
// 输入参数 : uint8_t addr,寄存器地址 uint8_t *data读数据存储地址
// 返回参数 : 
// 说明     :
////////////////////////////////////////////////////////////////////////////////
void SX1276Read( uint8_t addr, uint8_t *data )
{
    SX1276ReadBuffer( addr, data, 1 );
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF  向FIFO写数据
// 输入参数 : uint8_t *buffer,数组指针 uint8_t size长度
// 返回参数 : 
// 说明     :
////////////////////////////////////////////////////////////////////////////////
void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1276WriteBuffer( 0, buffer, size );
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF  向FIFO读数据
// 输入参数 : uint8_t *buffer,数组指针 uint8_t size长度
// 返回参数 : uint8_t *buffer 存储读取内容
// 说明     :
////////////////////////////////////////////////////////////////////////////////
void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1276ReadBuffer( 0, buffer, size );
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF  TX/RX的PA切换
// 输入参数 :  bool txEnable  切换逻辑
// 返回参数 : 无
// 说明     :真：作为TX。假：作为RX   为硬件两个PA控制IO口
////////////////////////////////////////////////////////////////////////////////
/*void SX1276WriteRxTx( bool txEnable )
{
    if( txEnable != 0 )       //如果为真，为TX
    {
        PA_RX_OUT = 0;        //PA_RX为0
        PA_TX_OUT = 1;        //PA_TX为1
    }
    else  //为假，为RX
    {
        PA_RX_OUT = 1;        //PA_RX为1
        PA_TX_OUT = 0;         //PA_TX为0
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



