/*
 * SX1262.h
 *
 *      Author: Faris Alsaad
 */

#ifndef INC_SX1262_H_
#define INC_SX1262_H_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <main.h>


uint8_t RF_Power_State;

#define LoRa_CS_PORT GPIOA
#define LoRa_CS_PIN  GPIO_PIN_4
#define BUSY_PORT GPIOC
#define BUSY_PIN  GPIO_PIN_4
#define RXEN_PORT GPIOA
#define RXEN_PIN  GPIO_PIN_3
#define TXEN_PORT GPIOA
#define TXEN_PIN  GPIO_PIN_2
#define DIO1_PORT GPIOB
#define DIO1_PIN  GPIO_PIN_0
#define NRST_PORT GPIOC
#define NRST_PIN  GPIO_PIN_5

#define LOW   	 0
#define HIGH  	 1
#define ON  	 HIGH
#define OFF  	 LOW

#define SetStandby 					0x80
#define Standby_STDBY_RC			0x00
#define Standby_STDBY_XOSC			0x01
#define SetFS 						0xC1
#define SetTx 						0x83
// 5 Seconds Timeout
#define TX_TIMEOUT_5_Sec 			0x04E200
#define TX_NO_TIMEOUT 				0x000000
#define SetRx						0x82
#define RX_NO_TIMEOUT 				0x000000
#define RX_NO_TIMEOUT_CONTINUOUS 	0xFFFFFF
#define RX_TIMEOUT_5_Sec 			0x04E200
#define CalibrateImage				0x98
#define SetCAD						0xC5
#define SetLoRaSymbNumTimeout		0xA0
#define SetRxTxFallbackMode 		0x93
#define FallbackMode_STDBY_RC		0x20 // Select for Drone
#define FallbackMode_STDBY_XOSC		0x30 // Select for Transmitter
#define FallbackMode_FS				0x40
#define SetDioIrqParams				0x08
#define GetIrqStatus				0x12
#define ClearIrqStatus				0x02
#define SetDIO2AsRfSwitchCtrl		0x9D
#define SetRfFrequency				0x86
#define SetPacketType				0x8A
#define GetPacketType				0x11
#define SetTxParams					0x8E
#define SetModulationParams			0x8B
#define SetPacketParams				0x8C
#define SetCadParams				0x88
#define SetBufferBaseAddress		0x8F
#define SetLoRaSymbNumTimeout		0xA0
#define SetDIO3AsTCXOCtrl			0x97
#define GetPacketStatus				0x14
#define GetDeviceErrors				0x17
#define PACKET_TYPE_GFSK			0x00
#define PACKET_TYPE_LORA			0x01
#define PA_22dBm					0x04,0x07
#define PA_20dBm					0x03,0x05
#define PA_17dBm					0x02,0x03
#define PA_14dBm					0x02,0x02
#define Freq_915MHz					915
#define Freq_900MHz					900
#define Freq_868MHz					868
#define Freq_800MHz					800
#define GetStatus					0xC0
#define GetRssiInst					0x15
#define GetRxBufferStatus			0x13
#define GetPacketStatus				0x14
#define GetDeviceErrors				0x17
#define ClearDeviceErrors			0x07
#define GetStats					0x10
#define ResetStats					0x00
#define Calibrate					0x89
#define WriteRegister				0x0D
#define ReadRegister				0x1D
#define WriteBuffer					0x0E
#define ReadBuffer					0x1E
#define RXBufferAddressBase			0x00
#define TXBufferAddressBase			0x80
#define WriteReg					0x0D
#define ReadReg						0x1D
#define TxClampConfig				0x08D8
#define TxModulation				0x0889
#define NOP							0x00
#define SetRegulatorMode			0x96

enum
{
	TxParamsPwr_0dBm = 0x00,
	TxParamsPwr_1dBm,
	TxParamsPwr_2dBm,
	TxParamsPwr_14dBm = 0x0E,
	TxParamsPwr_22dBm = 0x16
};

enum
{
	GetStatus_STBY_RC = 2,
	GetStatus_STBY_XOSC,
	GetStatus_FS,
	GetStatus_RX,
	GetStatus_TX
};
enum
{
	SET_RAMP_10U = 0,	// 10U = 10 microseconds
	SET_RAMP_20U,
	SET_RAMP_40U,
	SET_RAMP_80U,
	SET_RAMP_200U,
	SET_RAMP_800U,
	SET_RAMP_1700U,
	SET_RAMP_3400U
};

enum SpreadFactor_LoRa
{
	SF5 = 0x05,
	SF6,
	SF7,
	SF8,
	SF9,
	SF10,
	SF11,
	SF12
};

enum Bandwidth_LoRa
{
	LORA_BW_7KHz = 0x00,
	LORA_BW_15KHz,
	LORA_BW_31KHz,
	LORA_BW_62KHz,
	LORA_BW_125KHz,
	LORA_BW_250KHz,
	LORA_BW_500KHz,
	LORA_BW_10KHz = 0x08,
	LORA_BW_20KHz,
	LORA_BW_41KHz
};

enum CodingRate_LoRa
{
	LORA_CR_4_5 = 0x01,
	LORA_CR_4_6,
	LORA_CR_4_7,
	LORA_CR_4_8
};

enum LowDataRateOptimization_LoRa
{
	LowDataRateOptimizeOFF = 0x00,
	LowDataRateOptimizeON
};

enum PreambleLength_LoRa
{
	_1Symbol = 0x0001,
	_2Symbols,
	_3Symbols,
	_4Symbols,
	_5Symbols,
	_8Symbols = 0x0008,
	_12Symbols = 0x000C,
	_16Symbols = 0x0010,
	_32Symbols = 0x0010
};

enum HeaderType_LoRa
{
	VariableLengthPacket = 0x00,
	FixedLengthPacket
};

enum PayloadLength_LoRa
{
	_1Byte = 0x01,
	_2Bytes,
	_3Bytes,
	_4Bytes,
	_5Bytes,
};

enum CRCType_LoRa
{
	CRC_OFF = 0x00,
	CRC_ON
};

enum IQSetup_LoRa
{
	STD_IQ = 0x00,
	Inverted_IQ
};

enum IRQ_Masks
{
	IRQ_None = 0x0000,
	IRQ_TxDone = 0x0001,
	IRQ_RxDone = 0x0002,
	IRQ_PreambleDetected = 0x0004,
	IRQ_SyncWordValid = 0x0008,
	IRQ_HeaderValid = 0x0010,
	IRQ_HeaderErr = 0x0020,
	IRQ_CrcErr = 0x0040,
	IRQ_CadDone = 0x0080,
	IRQ_CadDetected = 0x0100,
	IRQ_Timeout = 0x0200,
	IRQ_ALL = 0x03FF
};
enum DIO2_RF_Switch
{
	DIO2_RF_SWITCH_DISABLE = 0x00,
	DIO2_RF_SWITCH_ENABLE
};

enum Power_Source
{
	LDO = 0x00,
	BUCKandLDO
};
enum CAD_Num_Of_Symbols
{
	CAD_ON_1_SYMB = 0x00,
	CAD_ON_2_SYMB,
	CAD_ON_4_SYMB,
	CAD_ON_8_SYMB,
	CAD_ON_16_SYMB,
};
enum CAD_Exit_Mode
{
	CAD_ONLY = 0x00,
	CAD_RX
};

HAL_StatusTypeDef SPI1Send(uint8_t* TxData, uint16_t bytes);
HAL_StatusTypeDef SPI1Receive(uint8_t* RxData, uint16_t bytes);
HAL_StatusTypeDef SPI1XFR(uint8_t* TxData, uint16_t TXbytes, uint8_t* RxData, uint16_t RXbytes);
uint8_t Write_Buffer(uint8_t offset, uint8_t* data, uint16_t bytes);
uint8_t Read_Buffer(uint8_t offset, uint8_t* data, uint16_t bytes);
void Clear_TX_Buffer();
void Clear_RX_Buffer();
void Clear_TX_RX_Buffer();
uint16_t Get_Rx_Buffer_Status();
uint8_t Set_Standby_Mode(uint8_t clk);
void Set_LoRa_SymbNum_Timeout(uint8_t NumberOfSymbols);
void Set_TX_Inf();
void Set_CAD();
void Set_Cad_Params(uint8_t cadSymbolNum, uint8_t cadExitMode);
void Calibrate_Image();
uint8_t Set_TX_Mode(uint32_t timeout);
uint8_t Set_RX_Mode(uint32_t timeout);
uint8_t Set_FS_Mode();
void Set_Sleep_Mode();
void Set_Fallback_Mode(uint8_t FallbackMode);
uint8_t Set_Packet_Type(uint8_t PacketType);
void Set_RF_Frequency(uint16_t Freq_MHz);
void Set_Power_Amplifier(uint8_t paDutyCycle,uint8_t hpMax);
void Set_Tx_Parameters(uint8_t Power, uint8_t RampTime);
uint8_t Set_Buff_Addr(uint8_t TXBaseAddr,uint8_t RXBaseAddr);
void Set_Modulation_Params_LoRa(uint8_t SpreadFactor,uint8_t Bandwidth,uint8_t CodingRate,uint8_t LowDataRateOptimization);
void Set_Modulation_Params_GFSK(uint32_t BaudRate,uint8_t PulseShape,uint8_t Bandwidth,uint32_t FreqDeviation);
void Set_Packet_Params(uint16_t PreambleLength,uint8_t HeaderType,uint8_t PayloadLength, uint8_t CRCType, uint8_t InvertIQ);
void Set_Packet_Params_GFSK(uint16_t PreambleLength,uint8_t PreambleDetectorLength,uint8_t SyncWordLength,uint8_t AddrComp,uint8_t PacketType,uint8_t PayloadLength,uint8_t CRCType,uint8_t Whitening);
void Set_Dio_Irq_Params(uint16_t IRQ_Mask, uint16_t DIO1_Mask, uint16_t DIO2_Mask, uint16_t DIO3_Mask);
void Clear_IRQ_Status(uint16_t IRQ_Mask);
void Set_DIO2_AsRfSwitchCtrl(uint8_t config);
void Set_DIO3_AsTCXOCtrl(uint8_t tcxoVoltage);
void Set_Regulator_Mode(uint8_t PowerSource);
uint8_t Get_Status();
uint32_t Get_Packet_Status();
uint8_t Get_Mode();
uint8_t Get_RX_Data_Offset();
uint16_t Get_IRQ_Status();
uint8_t RF_Chip_isBusy();
uint8_t RF_Chip_isON();
void RF_Chip_Power(uint8_t State);
uint16_t Get_Device_Errors();
uint16_t Clear_Device_Errors();
void Calibrate_Blocks(uint8_t calibParam);
void Write_Reg(uint16_t Address, uint8_t* data, uint8_t bytes);
void Read_Reg(uint16_t Address, uint8_t* data, uint8_t bytes);
void Tx_Clamp_Config();
void Tx_Modulation(uint16_t frequency_KHz);
void OCP_Config();
void Set_RX_Gain(uint8_t Level);
void Inv_IQ_Solution();
void LoRa_Setup();
void FSK_Setup();

#endif /* INC_SX1262_H_ */
