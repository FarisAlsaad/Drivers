/*
 * SX1262.c
 *
 *      Author: Faris Alsaad
 */

#include "SX1262.h"

void LoRa_Setup()
{
	uint8_t status = 0;
	RF_Chip_Power(OFF);
	HAL_Delay(1);	// STM32 delay 1 ms function
	RF_Chip_Power(ON);
	// Wait for SX1262 to Power up
	while(RF_Chip_isBusy());
	if(Get_Mode() != GetStatus_STBY_RC)
	{
		// Set in Standby Mode
		status = Set_Standby_Mode(Standby_STDBY_RC);
		status += status;			// Increment error count
	}
	// Set packet type as LoRa
	status = Set_Packet_Type(PACKET_TYPE_LORA);
	status += status;
	while(RF_Chip_isBusy());
	// Set RF Frequency of 868MHz
	Set_RF_Frequency(Freq_868MHz);
	//Retention register
	while(RF_Chip_isBusy());
	// Set TX,RX buffers' starting addresses
	status = Set_Buff_Addr(TXBufferAddressBase,RXBufferAddressBase);
	status += status;
	while(RF_Chip_isBusy());
	// Set LoRa Modulation Parameters
	Set_Modulation_Params_LoRa(SF8,LORA_BW_62KHz,LORA_CR_4_8,LowDataRateOptimizeOFF);
	while(RF_Chip_isBusy());
	// Set LoRa Packet Parameters
	Set_Packet_Params(_32Symbols,VariableLengthPacket,_4Bytes,CRC_ON,STD_IQ);
	while(RF_Chip_isBusy());
	// Set IRQ to GPIO Mapping Parameters
	Set_Dio_Irq_Params((IRQ_ALL), (IRQ_TxDone | IRQ_RxDone | IRQ_Timeout), IRQ_None, IRQ_None);
	while(RF_Chip_isBusy());
	// Set Fallback Mode as FS
	Set_Fallback_Mode(FallbackMode_STDBY_RC);
	// BW 250KHz (8), SF6, BaudRate = kb/s,  Symbol Rate = BW/(2^SF)

	// Set PA configuration
	Set_Power_Amplifier(PA_22dBm);
	while(RF_Chip_isBusy());
	// Set TX configuration
	Set_Tx_Parameters(TxParamsPwr_22dBm,SET_RAMP_200U);
	while(RF_Chip_isBusy());

	Set_DIO2_AsRfSwitchCtrl(DIO2_RF_SWITCH_DISABLE);
	while(RF_Chip_isBusy());
	Set_Regulator_Mode(BUCKandLDO);
	while(RF_Chip_isBusy());
	Tx_Clamp_Config();
	Set_DIO3_AsTCXOCtrl(0x07);
	while(RF_Chip_isBusy());
//	Set_LoRa_SymbNum_Timeout(_12Symbols);
	Inv_IQ_Solution();
	while(RF_Chip_isBusy());
	Set_RX_Gain(HIGH);
	while(RF_Chip_isBusy());
	OCP_Config();
	while(RF_Chip_isBusy());
	Clear_TX_RX_Buffer();
	while(RF_Chip_isBusy());
	Clear_Device_Errors();
	while(RF_Chip_isBusy());
	Clear_IRQ_Status(IRQ_ALL);
}

void FSK_Setup()
{
	uint8_t status = 0;
	RF_Chip_Power(OFF);
	HAL_Delay(1);
	RF_Chip_Power(ON);
	// Wait for SX1262 to Power up
	while(RF_Chip_isBusy());
	if(Get_Mode() != GetStatus_STBY_RC)
	{
		// Set in Standby Mode
		status = Set_Standby_Mode(Standby_STDBY_RC);
		status += status;			// Increment error count
	}

	// Set packet type as LoRa
	status = Set_Packet_Type(PACKET_TYPE_GFSK);
	status += status;
	while(RF_Chip_isBusy());
	// Set FSK Modulation Parameters
	Set_Modulation_Params_GFSK(0x002580,0x00,0x19,0x00CCCC);
	while(RF_Chip_isBusy());
	// Set FSK Packet Parameters
	Set_Packet_Params_GFSK(0x0008,0x00,0x20,0x00,0x00,0x04,0x01,0x00);
	while(RF_Chip_isBusy());
	// Set Fallback Mode as FS
	//	Set_Fallback_Mode(FallbackMode_STDBY_RC);
	// BW 250KHz (8), SF6, BaudRate = kb/s,  Symbol Rate = BW/(2^SF)
	// Set RF Frequency of 915MHz
	Set_RF_Frequency(Freq_868MHz);//Freq_800MHz
	//Retention register
	while(RF_Chip_isBusy());
	// Set PA configuration
	Set_Power_Amplifier(PA_14dBm);
	while(RF_Chip_isBusy());
	// Set TX configuration
	Set_Tx_Parameters(TxParamsPwr_14dBm,SET_RAMP_20U);
	while(RF_Chip_isBusy());
	// Set TX,RX buffers' starting addresses
	status = Set_Buff_Addr(TXBufferAddressBase,RXBufferAddressBase);
	status += status;
	while(RF_Chip_isBusy());
	// Set IRQ to GPIO Mapping Parameters
	Set_Dio_Irq_Params((IRQ_ALL), (IRQ_TxDone | IRQ_RxDone | IRQ_Timeout | IRQ_SyncWordValid), IRQ_None, IRQ_None);
	while(RF_Chip_isBusy());
	//	Define Sync Word value
	uint8_t SyncWord = 0xD9;
	Write_Reg(0x06C0, &SyncWord, 1);
	SyncWord = 0xA4;
	Write_Reg(0x06C1, &SyncWord, 1);
	SyncWord = 0x95;
	Write_Reg(0x06C2, &SyncWord, 1);
	SyncWord = 0x35;
	Write_Reg(0x06C3, &SyncWord, 1);
	SyncWord = 0xC6;
	Write_Reg(0x06C4, &SyncWord, 1);
	SyncWord = 0x54;
	Write_Reg(0x06C5, &SyncWord, 1);
	SyncWord = 0x99;
	Write_Reg(0x06C6, &SyncWord, 1);
	SyncWord = 0x54;
	Write_Reg(0x06C7, &SyncWord, 1);
	Set_DIO2_AsRfSwitchCtrl(DIO2_RF_SWITCH_DISABLE);
	while(RF_Chip_isBusy());
	Tx_Clamp_Config();
	Set_DIO3_AsTCXOCtrl(0x07);
	while(RF_Chip_isBusy());
	Set_Regulator_Mode(BUCKandLDO);

	//	Set_LoRa_SymbNum_Timeout(_12Symbols);
	Clear_TX_RX_Buffer();
	Clear_Device_Errors();
	while(RF_Chip_isBusy());
	Clear_IRQ_Status(IRQ_ALL);
}

void Set_Regulator_Mode(uint8_t PowerSource)
{
	uint8_t sendData[2] = {0};
	sendData[0] = SetRegulatorMode;
	sendData[1] = PowerSource;
	SPI1Send(sendData, 2);
}

uint8_t Read_Buffer(uint8_t offset, uint8_t* data, uint16_t bytes)
{
	uint16_t check_overflow = offset+bytes;
	if(check_overflow > 256)
		return 1;
	uint8_t sendData[256+3] = {0},ReceiveData[256+3] = {0};
	uint16_t i;
	sendData[0] = ReadBuffer;
	sendData[1] = offset;
	sendData[2] = NOP;
	SPI1XFR(sendData, bytes+3, ReceiveData, bytes+3);
	for(i = 0;i<bytes;i++)
	{
		data[i] = ReceiveData[i+3];
	}
	while(RF_Chip_isBusy());
	return 0;
}
uint8_t Write_Buffer(uint8_t offset, uint8_t* data, uint16_t bytes)
{
	uint16_t check_overflow = offset+bytes;
	if(check_overflow > 256)
		return 1;
	uint8_t sendData[256+2] = {0};
	uint16_t i;
	sendData[0] = WriteBuffer;
	sendData[1] = offset;
	for(i = 0;i<bytes;i++)
	{
		sendData[i+2] = (uint8_t)(data[i]);
	}
	while(RF_Chip_isBusy());
	SPI1Send(sendData, bytes+2);
	while(RF_Chip_isBusy());
	return 0;
}
void Clear_TX_Buffer()
{
	uint8_t data_buff[128] = {0};
	Write_Buffer(TXBufferAddressBase, data_buff, 128);
}
void Clear_RX_Buffer()
{
	uint8_t data_buff[128] = {0};
	Write_Buffer(RXBufferAddressBase, data_buff, 128);
}
void Clear_TX_RX_Buffer()
{
	uint8_t data_buff[256] = {0};
	Write_Buffer(RXBufferAddressBase, data_buff, 256);
}
void Write_Reg(uint16_t Address, uint8_t* data, uint8_t bytes)
{
	uint8_t sendData[128],i;
	sendData[0] = WriteReg;
	sendData[1] = (Address >> 8) & 0xFF;
	sendData[2] = Address & 0xFF;
	for(i = 0;i<bytes;i++)
	{
		sendData[i+3] = (uint8_t)(data[i]);
	}
	SPI1Send(sendData, bytes+3);
	while(RF_Chip_isBusy());
}
void Read_Reg(uint16_t Address, uint8_t* data, uint8_t bytes)
{
	uint8_t sendData[132] = {0},ReceiveData[132] = {0},i;
	sendData[0] = ReadReg;
	sendData[1] = (Address >> 8) & 0xFF;
	sendData[2] = Address & 0xFF;
	sendData[3] = NOP;
	SPI1XFR(sendData, bytes+4, ReceiveData, bytes+4);
	for(i = 0;i<bytes;i++)
	{
		data[i] = ReceiveData[i+4];
	}
	while(RF_Chip_isBusy());
}
void Tx_Clamp_Config()
{
	uint8_t value[1] = {0};
	Read_Reg(TxClampConfig,value,1);
	value[0] |= 0x1E;
	Write_Reg(TxClampConfig,value,1);
}
void Tx_Modulation(uint16_t frequency_KHz)
{
	uint8_t value[1] = {0};
	Read_Reg(TxModulation,value,1);
	if(frequency_KHz >= 500)
		value[0] &= 0xFB;
	else
		value[0] |= 0x04;
	Write_Reg(TxModulation,value,1);
}
void Inv_IQ_Solution()
{
	uint8_t value[1] = {0};
	Read_Reg(0x0736,value,1);
	value[0] |= 0x04;
	Write_Reg(0x0736,value,1);
}
void Set_RX_Gain(uint8_t Level)
{
	uint8_t value[1];
	if(Level == LOW)
		value[0] = 0x94;
	else
		value[0] = 0x96;
	Write_Reg(0x08AC,value,1);
}
void OCP_Config()
{
	uint8_t value[1];
	value[0] = 0x40;
	Write_Reg(0x08E7,value,1);
}
void Set_DIO2_AsRfSwitchCtrl(uint8_t config)
{
	uint8_t sendData[2] = {0};
	sendData[0] = SetDIO2AsRfSwitchCtrl;
	sendData[1] = config;
	SPI1Send(sendData, 2);
}
void Set_DIO3_AsTCXOCtrl(uint8_t tcxoVoltage)
{
	uint8_t sendData[5] = {0};
	sendData[0] = SetDIO3AsTCXOCtrl;
	sendData[1] = tcxoVoltage;
	sendData[2] = 0x00;
	sendData[3] = 0x03;
	sendData[4] = 0xE8;
	SPI1Send(sendData, 5);
}
void Set_Dio_Irq_Params(uint16_t IRQ_Mask, uint16_t DIO1_Mask, uint16_t DIO2_Mask, uint16_t DIO3_Mask)
{
	uint8_t sendData[9] = {0};
	sendData[0] = SetDioIrqParams;
	sendData[1] = (IRQ_Mask >> 8) & 0xFF;
	sendData[2] = IRQ_Mask & 0xFF;
	sendData[3] = (DIO1_Mask >> 8) & 0xFF;
	sendData[4] = DIO1_Mask & 0xFF;
	sendData[5] = (DIO2_Mask >> 8) & 0xFF;
	sendData[6] = DIO2_Mask & 0xFF;
	sendData[7] = (DIO3_Mask >> 8) & 0xFF;
	sendData[8] = DIO3_Mask & 0xFF;
	SPI1Send(sendData, 9);
}
void Set_Packet_Params(uint16_t PreambleLength,uint8_t HeaderType,uint8_t PayloadLength, uint8_t CRCType, uint8_t InvertIQ)
{
	uint8_t sendData[7] = {0};
	sendData[0] = SetPacketParams;
	sendData[1] = (PreambleLength >> 8) & 0xFF;
	sendData[2] = PreambleLength & 0xFF;
	sendData[3] = HeaderType;
	sendData[4] = PayloadLength;
	sendData[5] = CRCType;
	sendData[6] = InvertIQ;
	SPI1Send(sendData, 7);
}
void Set_Packet_Params_GFSK(uint16_t PreambleLength,uint8_t PreambleDetectorLength,uint8_t SyncWordLength,uint8_t AddrComp,uint8_t PacketType,uint8_t PayloadLength,uint8_t CRCType,uint8_t Whitening)
{
	uint8_t sendData[10] = {0};
	sendData[0] = SetPacketParams;
	sendData[1] = (PreambleLength >> 8) & 0xFF;
	sendData[2] = PreambleLength & 0xFF;
	sendData[3] = PreambleDetectorLength;
	sendData[4] = SyncWordLength;
	sendData[5] = AddrComp;
	sendData[6] = PacketType;
	sendData[7] = PayloadLength;
	sendData[8] = CRCType;
	sendData[9] = Whitening;
	SPI1Send(sendData, 10);
}
void Set_Modulation_Params_LoRa(uint8_t SpreadFactor,uint8_t Bandwidth,uint8_t CodingRate,uint8_t LowDataRateOptimization)
{
	uint8_t sendData[5] = {0};
	sendData[0] = SetModulationParams;
	sendData[1] = SpreadFactor;
	sendData[2] = Bandwidth;
	sendData[3] = CodingRate;
	sendData[4] = LowDataRateOptimization;
	SPI1Send(sendData, 5);
}
void Set_Modulation_Params_GFSK(uint32_t BaudRate,uint8_t PulseShape,uint8_t Bandwidth,uint32_t FreqDeviation)
{
	uint8_t sendData[9] = {0};
	sendData[0] = SetModulationParams;
	sendData[1] = (BaudRate >> 16) & 0xFF;
	sendData[2] = (BaudRate >> 8) & 0xFF;
	sendData[3] = BaudRate & 0xFF;
	sendData[4] = PulseShape;
	sendData[5] = Bandwidth;
	sendData[6] = (FreqDeviation >> 16) & 0xFF;
	sendData[7] = (FreqDeviation >> 8) & 0xFF;
	sendData[8] = FreqDeviation & 0xFF;
	SPI1Send(sendData, 9);
}
void Clear_IRQ_Status(uint16_t IRQ_Mask)
{
	uint8_t sendData[3] = {0};
	sendData[0] = ClearIrqStatus;
	sendData[1] = (IRQ_Mask >> 8) & 0xFF;
	sendData[2] = IRQ_Mask & 0xFF;
	SPI1Send(sendData, 3);
}
uint16_t Get_IRQ_Status()
{
	uint8_t sendData[4] = {0};
	uint8_t ReceiveData[4] = {0};
	uint16_t IrqStatus = 0;
	sendData[0] = GetIrqStatus;
	sendData[1] = NOP;
	sendData[2] = NOP;
	sendData[3] = NOP;
	SPI1XFR(sendData, 4, ReceiveData, 4);
	IrqStatus = ReceiveData[2];
	IrqStatus = (IrqStatus << 8) | ReceiveData[3];
	return IrqStatus;
}
// RX Packet Length + Base address in FIFO
uint16_t Get_Rx_Buffer_Status()
{
	uint8_t sendData[4] = {0};
	uint8_t ReceiveData[4] = {0};
	uint16_t retVal;
	sendData[0] = GetRxBufferStatus;
	sendData[1] = NOP;
	sendData[2] = NOP;
	sendData[3] = NOP;
	SPI1XFR(sendData, 4, ReceiveData, 4);
	retVal = ReceiveData[2];
	retVal = (retVal << 8) | ReceiveData[3];
	return retVal;
}
uint8_t Get_RX_Data_Offset()
{
	uint8_t sendData[4] = {0};
	uint8_t ReceiveData[4] = {0};
	sendData[0] = GetRxBufferStatus;
	sendData[1] = NOP;
	sendData[2] = NOP;
	sendData[3] = NOP;
	SPI1XFR(sendData, 4, ReceiveData, 4);
	return (ReceiveData[2] + ReceiveData[3]);
}
uint8_t Set_Buff_Addr(uint8_t TXBaseAddr,uint8_t RXBaseAddr)
{
	uint8_t sendData[4] = {0};
	uint8_t ReceiveData[4] = {0};
	sendData[0] = SetBufferBaseAddress;
	sendData[1] = TXBaseAddr;
	sendData[2] = RXBaseAddr;
	SPI1Send(sendData, 3);
	HAL_Delay(1);
	// Check if addresses are set correctly by reading them back
	sendData[0] = GetRxBufferStatus;
	sendData[1] = NOP;
	sendData[2] = NOP;
	sendData[3] = NOP;
	SPI1XFR(sendData, 4, ReceiveData, 4);
	if(ReceiveData[3] == RXBaseAddr)
		return 0;
	return 1;
}
void Set_Tx_Parameters(uint8_t Power, uint8_t RampTime)
{
	uint8_t sendData[3] = {0};
	sendData[0] = SetTxParams;
	sendData[1] = Power;
	sendData[2] = RampTime;
	SPI1Send(sendData, 3);
}
void Set_Power_Amplifier(uint8_t paDutyCycle,uint8_t hpMax)
{
	uint8_t sendData[4] = {0};
	sendData[0] = paDutyCycle;
	sendData[1] = hpMax;
	sendData[2] = 0x00;
	sendData[3] = 0x01;
	SPI1Send(sendData, 4);
}
void Set_RF_Frequency(uint16_t Freq_MHz)
{
	uint8_t sendData[5] = {0};
	//(Freq_MHz/32MHz)*(2^25);
	uint32_t Rf_Freq_Setting = Freq_MHz*1048576;
	sendData[0] = SetRfFrequency;
	sendData[1] = (Rf_Freq_Setting >> 24) & 0xFF;
	sendData[2] = (Rf_Freq_Setting >> 16) & 0xFF;
	sendData[3] = (Rf_Freq_Setting >> 8) & 0xFF;
	sendData[4] = (Rf_Freq_Setting) & 0xFF;

	SPI1Send(sendData, 5);
}
void Set_Fallback_Mode(uint8_t FallbackMode)
{
	uint8_t sendData[2] = {0};
	sendData[0] = SetRxTxFallbackMode;
	sendData[1] = FallbackMode;
	SPI1Send(sendData, 2);
}
uint8_t Set_Standby_Mode(uint8_t clk)
{
	uint8_t sendData[2] = {0};
	uint8_t CurrentMode = 0;
	sendData[0] = SetStandby;
	sendData[1] = clk;
	SPI1Send(sendData, 2);
	while(RF_Chip_isBusy());
//	HAL_Delay(1);
	CurrentMode = Get_Mode();
	if((clk == Standby_STDBY_RC) && (CurrentMode == GetStatus_STBY_RC))
		return 0;
	else if((clk == Standby_STDBY_XOSC) && (CurrentMode == GetStatus_STBY_XOSC))
		return 0;

	return 1;
}
uint8_t Set_FS_Mode()
{
	uint8_t sendData[1];
	sendData[0] = SetFS;
	SPI1Send(sendData, 1);
	while(RF_Chip_isBusy());
	if(Get_Mode() == GetStatus_FS)
		return 0;
	return 1;
}
void Set_TX_Inf()
{
	uint8_t sendData[1];
	Tx_Modulation(250);
	sendData[0] = 0xD2;
	SPI1Send(sendData, 1);
	while(RF_Chip_isBusy());
}
void Set_LoRa_SymbNum_Timeout(uint8_t NumberOfSymbols)
{
	uint8_t sendData[1];
	sendData[0] = SetLoRaSymbNumTimeout;
	sendData[1] = NumberOfSymbols;
	SPI1Send(sendData, 2);
	while(RF_Chip_isBusy());
}
void Set_CAD()
{
	uint8_t sendData[1];
	sendData[0] = SetCAD;
	SPI1Send(sendData, 1);
	while(RF_Chip_isBusy());
}
void Set_Cad_Params(uint8_t cadSymbolNum, uint8_t cadExitMode)
{
	uint8_t sendData[8];
	sendData[0] = SetCadParams;
	sendData[1] = cadSymbolNum;
	sendData[2] = 0x16;
	sendData[3] = 0x0A;
	sendData[4] = cadExitMode;
	sendData[5] = 0x00;
	sendData[6] = 0x00;
	sendData[7] = 0x80;
	SPI1Send(sendData, 8);
	while(RF_Chip_isBusy());
}
void Calibrate_Blocks(uint8_t calibParam)
{
	uint8_t sendData[2];
	sendData[0] = Calibrate;
	sendData[1] = calibParam;
	SPI1Send(sendData, 2);
	while(RF_Chip_isBusy());
}
void Calibrate_Image()
{
	uint8_t sendData[3];
	sendData[0] = CalibrateImage;
	sendData[1] = 0xD7;	// 863MHz
	sendData[2] = 0xDB; // 870MHz
	SPI1Send(sendData, 3);
	while(RF_Chip_isBusy());
}
uint8_t Set_TX_Mode(uint32_t timeout)
{
	uint8_t sendData[4];
	while(RF_Chip_isBusy());
	Tx_Modulation(250);
	while(RF_Chip_isBusy());
	HAL_GPIO_WritePin(RXEN_PORT, RXEN_PIN, LOW);
	while(RF_Chip_isBusy());
	HAL_GPIO_WritePin(TXEN_PORT, TXEN_PIN, HIGH);
	while(RF_Chip_isBusy());
	sendData[0] = SetTx;
	sendData[1] = (timeout >> 16) & 0xFF;
	sendData[2] = (timeout >> 8) & 0xFF;
	sendData[3] = timeout & 0xFF;
	SPI1Send(sendData, 4);
	while(RF_Chip_isBusy());
//	HAL_Delay(1);
//	if(Get_Mode() == GetStatus_FS)
//		return 0;
//	return 1;
	return 0;
}
uint8_t Set_RX_Mode(uint32_t timeout)
{
	uint8_t sendData[4];
	while(RF_Chip_isBusy());
	HAL_GPIO_WritePin(RXEN_PORT, RXEN_PIN, HIGH);
	while(RF_Chip_isBusy());
	HAL_GPIO_WritePin(TXEN_PORT, TXEN_PIN, LOW);
	while(RF_Chip_isBusy());
	sendData[0] = SetRx;
	sendData[1] = (timeout >> 16) & 0xFF;
	sendData[2] = (timeout >> 8) & 0xFF;
	sendData[3] = timeout & 0xFF;
	SPI1Send(sendData, 4);
	while(RF_Chip_isBusy());
//	HAL_Delay(1);
//	if(Get_Mode() == GetStatus_FS)
//		return 0;
//	return 1;
	return 0;
}
uint8_t Get_Status()
{
	uint8_t sendData[2] = {0};
	uint8_t ReceiveData[2] = {0};
	sendData[0] = GetStatus;
	sendData[1] = NOP;
	SPI1XFR(sendData, 2, ReceiveData, 2);
	while(RF_Chip_isBusy());
	return ReceiveData[1];
}
uint32_t Get_Packet_Status()
{
	uint8_t sendData[5] = {0};
	uint8_t ReceiveData[5] = {0};
	uint32_t retStatus = 0;
	sendData[0] = GetPacketStatus;
	sendData[1] = NOP;
	sendData[2] = NOP;
	sendData[3] = NOP;
	sendData[4] = NOP;
	SPI1XFR(sendData, 5, ReceiveData, 5);
	while(RF_Chip_isBusy());
	retStatus = ReceiveData[1];
	retStatus <<= 8;
	retStatus |= ReceiveData[2];
	retStatus <<= 8;
	retStatus |= ReceiveData[3];
	retStatus <<= 8;
	retStatus |= ReceiveData[4];
	return retStatus;
}
uint8_t Get_Mode()
{
	uint8_t Mode = Get_Status();
	Mode &= 0x70;
	Mode >>= 4;
	return Mode;
}
uint8_t Set_Packet_Type(uint8_t PacketType)
{
	uint8_t sendData[3] = {0};
	uint8_t ReceiveData[3] = {0};
	sendData[0] = SetPacketType;
	sendData[1] = PacketType;
	SPI1Send(sendData, 2);
	HAL_Delay(1);
	// Check packet type is set correctly
	sendData[0] = GetPacketType;
	sendData[1] = NOP;
	sendData[2] = NOP;
	SPI1XFR(sendData, 3, ReceiveData, 3);

	if(ReceiveData[2] == PacketType)
		return 0;
	return 1;
}
uint8_t RF_Chip_isBusy()
{
	return (HAL_GPIO_ReadPin(BUSY_PORT, BUSY_PIN) == HIGH);
}
uint8_t RF_Chip_isON()
{
	return RF_Power_State;
}
uint16_t Get_Device_Errors()
{
	uint8_t sendData[4] = {0};
	uint8_t ReceiveData[4] = {0};
	uint16_t errors = 0;
	sendData[0] = GetDeviceErrors;
	sendData[1] = NOP;
	sendData[2] = NOP;
	sendData[3] = NOP;
	SPI1XFR(sendData, 4, ReceiveData, 4);
	while(RF_Chip_isBusy());
	errors = ReceiveData[2];
	errors = (errors << 8) | ReceiveData[3];
	return errors;
}
uint16_t Clear_Device_Errors()
{
	uint8_t sendData[3] = {0};
	uint8_t ReceiveData[3] = {0};
	uint16_t errors = 0;
	sendData[0] = ClearDeviceErrors;
	sendData[1] = NOP;
	sendData[2] = NOP;
	SPI1XFR(sendData, 3, ReceiveData, 3);
	while(RF_Chip_isBusy());
	errors = ReceiveData[1];
	errors = (errors << 8) | ReceiveData[2];
	return errors;
}

void RF_Chip_Power(uint8_t State)
{
	RF_Power_State = State;
	HAL_GPIO_WritePin(NRST_PORT, NRST_PIN, State);
}
