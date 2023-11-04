/*
 *      Comunicação via CAN - CAN.h
 *
 *      Data: 13 de junho, 2023
 *      Autor: Gabriel Luiz
 *      Contato: (31) 97136-4334 || gabrielluiz.eletro@gmail.com
 */

#ifndef CAN_LOG_H_
#define CAN_LOG_H_

#include "stm32h7xx.h"

#define CAN_IDS_NUMBER 400

/* USADO PARA REGISTRAR UM BUFFER DA CAN */
typedef enum
{
	CAN_POSITIVE, CAN_NEGATIVE, CAN_FLOAT, CAN_DOUBLE
} Data_type_t;

typedef struct
{
	uint8_t *Data_buf[CAN_IDS_NUMBER];
	uint8_t Size_buf[CAN_IDS_NUMBER];
	Data_type_t Type_buf[CAN_IDS_NUMBER];

} CAN_Buffer_t;

/**
 * @brief  Função chamada quando detectado uma mensagem no barramento da CAN
 * @param  hfdcan: Handle da CAN || normalmente "hfdcan1"
 * @param  RxFifo0ITs: FIFO de interrupção utilizado
 * @retval ***NONE***
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

/**
 * @brief  Configura a CAN, overwrite do .IOC
 * @param  ***NONE***
 * @retval ***NONE***
 */
void CAN_Configure_Init(void);

/**
 * @brief  Inicialização do vetor de dados da CAN
 * @param  ***NONE***
 * @retval ***NONE***
 */
void CAN_Clean_Buffers(void);

/**
 * @brief  Inicialização da comunicação via CAN
 * @param  ***NONE***
 * @retval ***NONE***
 */
void CAN_Init(void);

/**
 * @brief  Função de tratamento das mensagens recebidas
 * @param  ID: Identificador da mensagem
 * @param  DATA: Buffer de dados da mensagem
 * @retval ***NONE***
 */
void CAN_Stream_ReceiveCallback(FDCAN_RxHeaderTypeDef *hRxFDCAN, uint8_t *DATA);

void CAN_Storage_POSITIVE(uint8_t Identifier, uint8_t Size, uint8_t *Data);

void CAN_Storage_NEGATIVE(uint8_t Identifier, uint8_t Size, uint8_t *Data);

void CAN_Storage_FLOAT(uint8_t Identifier, uint8_t Size, uint8_t *Data);

void CAN_Storage_DOUBLE(uint8_t Identifier, uint8_t Size, uint8_t *Data);

int64_t CAN_Get_value(uint16_t Identifier);

float CAN_Get_value_FLOAT(uint16_t Identifier);

double CAN_Get_value_DOUBLE(uint16_t Identifier);

/**
 * @brief  Envio de mensagem pelo barramento CAN
 * @param  Identifier: Identificador da mensagem
 * @param  data: Buffer de dados da mensagem
 * @retval ***NONE***
 */
void CAN_TxData(uint16_t Identifier, uint64_t Data);

void CAN_Send(uint16_t Identifier, int64_t Data);

void CAN_Send_Float(uint16_t Identifier, float Data, uint8_t Precision);

void CAN_Send_Double(uint16_t Identifier, double Data, uint8_t Precision);

#endif /* CAN_LOG_H_ */
