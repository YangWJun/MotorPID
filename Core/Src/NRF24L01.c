#include "NRF24L01.h"

uint8_t T_ADDR[5]={0xF0,0xF0,0xF0,0xF0,0xF0}; //发送地址
 uint8_t R_ADDR[5]={0xF0,0xF0,0xF0,0xF0,0xF0};//接收地址
 
 
 
 /*一些基本的操作函数*/
void W_CSN(uint8_t Value){
     HAL_GPIO_WritePin(CSN_PORT,CSN_Pin,Value);
   //GPIO_WriteBit(CSN_PORT,CSN_Pin,(BitAction)Value);
}  //对CSN的写操作

 void W_CE(uint8_t Value){
     HAL_GPIO_WritePin(CE_PORT,CE_Pin,Value);
//GPIO_WriteBit(CE_PORT,CE_Pin,(BitAction)Value);
}//对Ce的写操作
 

void W_MOSI(uint8_t Value){
    HAL_GPIO_WritePin(MOSI_PORT,MOSI_Pin,Value);
//GPIO_WriteBit(MOSI_PORT,MOSI_Pin,(BitAction)Value);
}
 //对MOSI写操作
 void W_MISO(uint8_t Value){
     HAL_GPIO_WritePin(MISO_PORT,MISO_Pin,Value);
//GPIO_WriteBit(MISO_PORT,MISO_Pin,(BitAction)Value);
}//对Miso写操作
void W_SCK(uint8_t Value){
    HAL_GPIO_WritePin(SCK_PORT,SCK_Pin,Value);
//GPIO_WriteBit(SCK_PORT,SCK_Pin,(BitAction)Value);
} 
//对SCK的写操作
uint8_t R_IRQ(void){
    return HAL_GPIO_ReadPin(IRQ_PORT,IRQ_Pin);
//return GPIO_ReadInputDataBit(IRQ_PORT,IRQ_Pin);

}
//对IRQ的读操作
uint8_t R_MISO(void){
    return HAL_GPIO_ReadPin(MISO_PORT,MISO_Pin);
//return GPIO_ReadInputDataBit(MISO_PORT,MISO_Pin);
}
//对MISO的读操作

 uint8_t SPI_SwapByte(uint8_t Byte){
  
	 uint8_t i,GetByte=0x00;
for(i=0;i<8;i++)	 
{
   W_MOSI(Byte&(0x80>>i));//先取Byte的高位
	 W_SCK(1); //拉高sck（使数据移出）
	 if(R_MISO()==1)
	 {
	  GetByte =GetByte|(0x80>>i);
	 }
    W_SCK(0);//拉低sck（使数据移入）
 
}
  return GetByte;//返回从机交换的数据
 
 }
  /*主机与从机交换一个字节，SPI通信的基本过程*/
 
 void W_Reg(uint8_t Reg,uint8_t Value)
 {
  W_CSN(0); //选中NRF24L01
  SPI_SwapByte(Reg); //指令
	 SPI_SwapByte(Value); //要写入的值
 W_CSN(1); //取消选中NRF24L01
 }
 /*对对应寄存器的写操作*/
 uint8_t R_Reg(uint8_t Reg)
 {uint8_t Value;
  W_CSN(0); //选中NRF24L01
  SPI_SwapByte(Reg);   //指令
	 Value=SPI_SwapByte(NOP);  //读寄存器的值
    W_CSN(1); //取消选中NRF24L01
	 return Value;
 }
  /*对对应寄存器的读操作*/

  void W_Buf(uint8_t Reg,uint8_t  *Buf,uint8_t len){
 uint8_t i;
	 W_CSN(0); //选中NRF24L01
	  SPI_SwapByte(Reg);
	 for(i=0;i<len;i++)
     {
		 SPI_SwapByte(Buf[i]);
	 }
   W_CSN(1); //取消选中NRF24L01
 
 }//写多个数据
 

 void R_Buf(uint8_t Reg,uint8_t  *Buf,uint8_t len){
  uint8_t i;
	 W_CSN(0); //选中NRF24L01
	 SPI_SwapByte(Reg);
	 for(i=0;i<len;i++)
	 {
		 
		Buf[i]=SPI_SwapByte(NOP);
	 }
   W_CSN(1); //取消选中NRF24L01
 
 }//读多个数据
 
 void NRF24L01_Init(void){
// NRF24L01_Pin_Init();
	W_CE(0); 
 W_Buf(NRF_WRITE_REG+TX_ADDR,T_ADDR,5);//配置发送通道，写入TX的地址
 W_Buf( NRF_WRITE_REG+ RX_ADDR_P0,R_ADDR,5);  //配置接收通道0
 W_Reg( NRF_WRITE_REG+CONFIG,0x0F); //配置成接受模式
	 W_Reg( NRF_WRITE_REG+EN_AA,0x01);//开启自动应答
  W_Reg( NRF_WRITE_REG+RF_CH,0x00);//配置通信频率2.4G
	  W_Reg( NRF_WRITE_REG+RX_PW_P0,2);//配置接受通道0接收的数据宽度3字节
	  W_Reg( NRF_WRITE_REG+EN_RXADDR,0x01);//使能接收通道0
	  W_Reg( NRF_WRITE_REG+SETUP_RETR,0x1A);//配置自动重发(580us重发间隔，重复10次)
	 W_Reg(FLUSH_RX,NOP);//清除接收数据缓存器
     W_CE(1);
 }
 
 
 void ReceiveData(uint8_t *Buf){
 uint8_t Status;
	 Status=R_Reg( NRF_READ_REG+STATUS); 
	 if(Status&0x40)
	 {   R_Buf(RD_RX_PLOAD,Buf,2); //读到的数据存进了Buf
       W_Reg(FLUSH_RX,NOP); //清除接收数据缓存器
		 W_Reg(NRF_WRITE_REG+STATUS,Status);
		 HAL_Delay(150);
	 }
 
 }
 
 uint8_t SendData(uint8_t *Buf){
	 uint8_t Status;
	 W_Buf(WR_TX_PLOAD,Buf,2);
	  W_CE(0); //改变了发送模式，要先待机
  W_Reg( NRF_WRITE_REG+CONFIG,0x0E); //配置为发送模式
	  W_CE(1);
	 	 
	 while(R_IRQ()==1);//等待中断
     Status =R_Reg(NRF_READ_REG+STATUS);
   if(Status&0x10) //当发送达到最大次数
	 { W_Reg(FLUSH_TX,NOP);//清除发送数据缓存器
	    W_Reg(NRF_WRITE_REG+STATUS,Status); //清除中断
		 return 3;
	 }
	 if(Status&0x20) //当发送成功,接收到应答信号
	 {W_Reg(NRF_WRITE_REG+STATUS,Status); //清除中断
	 return 2;
	 }
 }
 
 
 
