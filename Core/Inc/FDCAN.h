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
	FDCAN_POSITIVE,
	FDCAN_NEGATIVE,
	FDCAN_FLOAT,
	FDCAN_DOUBLE,
	FDCAN_FREE
} Data_type_t;

typedef enum
{
	FDCAN_OK,
	FDCAN_ERROR,
	FDCAN_TIMEOUT
} FDCAN_StatusTypedef;

typedef struct
{
	uint8_t *Data_buf[CAN_IDS_NUMBER];
	uint8_t Size_buf[CAN_IDS_NUMBER];
	Data_type_t Type_buf[CAN_IDS_NUMBER];

} CAN_Buffer_t;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- CALLBACK DE RECEBIMENTO DA CAN --------------------------------------------------------------------------------*/

/**
 * @brief  Função chamada quando detectado uma mensagem no barramento da CAN
 * @param  hfdcan: Handle da CAN || normalmente "hfdcan1"
 * @param  RxFifo0ITs: FIFO em que foi detectado a mensagem
 * @retval ***NONE***
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- TRATAMENTO DE MENSAGENS RECEBIDAS -----------------------------------------------------------------------------*/

/**
 * @brief  Função de tratamento das mensagens recebidas
 * @param  hRxFDCAN: Handler com as innformações do flame recebido
 * @param  Buffer: Buffer com os dados e informações da mensagem
 * @retval Status de execução da função
 */
FDCAN_StatusTypedef CAN_Stream_ReceiveCallback(FDCAN_RxHeaderTypeDef *hRxFDCAN, uint8_t *Buffer);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- PARAMETROS DE CONFIGURAÇÃO DA CAN --------------------------------------------------------------*/

/**
 * @brief  Configura a CAN, overwrite das configurações do .IOC
 * @param  ***NONE***
 * @retval Status de execução da função
 */
FDCAN_StatusTypedef CAN_Configure_Init(void);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- LIMPEZA DOS BUFFERS DE ARMAZENAMENTO --------------------------------------------------------------*/

/**
 * @brief  Inicialização dos buffers de armazenamento das mensagens da CAN
 * @param  ***NONE***
 * @retval ***NONE***
 */
void CAN_Clean_Buffers(void);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- INÍCIO DA COMUNICAÇÃO VIA FDCAN --------------------------------------------------------------*/

/**
 * @brief  Inicialização da comunicação via FDCAN
 * @param  ***NONE***
 * @retval Status de execução da função
 */
FDCAN_StatusTypedef CAN_Init(void);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- ARMAZENAMENTO DE VALORES INTEIROS POSITIVOS --------------------------------------------------------------*/

/**
 * @brief  Função para armazenamento de valores inteiros positivos, assim como informação de tipo e tamanho
 * @param  Identifier: Identificador da mensagem
 * @param  Size: Espaço necessário para armazenamento da mensagem
 * @param  Buffer: Ponteiro para os buffer que contém os dados e as informações para seu armazenamento
 * @retval ***NONE***
 */
void CAN_Storage_POSITIVE(uint8_t Identifier, uint8_t Size, uint8_t *Buffer);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- ARMAZENAMENTO DE VALORES INTEIROS NEGATIVOS --------------------------------------------------------------*/

/**
 * @brief  Função para armazenamento de valores inteiros negativos, assim como informação de tipo e tamanho
 * @param  Identifier: Identificador da mensagem
 * @param  Size: Espaço necessário para armazenamento da mensagem
 * @param  Buffer: Ponteiro para os buffer que contém os dados e as informações para seu armazenamento
 * @retval ***NONE***
 */
void CAN_Storage_NEGATIVE(uint8_t Identifier, uint8_t Size, uint8_t *Buffer);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- ARMAZENAMENTO DE VALORES FLOATS --------------------------------------------------------------*/

/**
 * @brief  Função para armazenamento de valores floats, assim como informação de tipo e tamanho
 * @param  Identifier: Identificador da mensagem
 * @param  Size: Espaço necessário para armazenamento da mensagem
 * @param  Buffer: Ponteiro para os buffer que contém os dados e as informações para seu armazenamento
 * @retval ***NONE***
 */
void CAN_Storage_FLOAT(uint8_t Identifier, uint8_t Size, uint8_t *Buffer);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- ARMAZENAMENTO DE VALORES DOUBLES --------------------------------------------------------------*/

/**
 * @brief  Função para armazenamento de valores doubles, assim como informação de tipo e tamanho
 * @param  Identifier: Identificador da mensagem
 * @param  Size: Espaço necessário para armazenamento da mensagem
 * @param  Buffer: Ponteiro para os buffer que contém os dados e as informações para seu armazenamento
 * @retval ***NONE***
 */
void CAN_Storage_DOUBLE(uint8_t Identifier, uint8_t Size, uint8_t *Buffer);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- ACESSO AOS VALORES INTEIROS ARMAZENADOS --------------------------------------------------------------*/

/**
 * @brief  Função para acesso aos valores inteiros armazenados na memória
 * @param  Identifier: Identificador da mensagem
 * @retval Valor inteiro armazenado, caso o valor não seja um inteiro retorna "FDCAN_ERRO"
 */
int64_t CAN_Get_value(uint16_t Identifier);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- ACESSO AOS VALORES FLOATS ARMAZENADOS --------------------------------------------------------------*/

/**
 * @brief  Função para acesso aos valores floats armazenados na memória
 * @param  Identifier: Identificador da mensagem
 * @retval Valor float armazeenado, caso o valor não seja um float retorna "FDCAN_ERRO"
 */
float CAN_Get_value_FLOAT(uint16_t Identifier);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- ACESSO AOS VALORES DOUBLES ARMAZENADOS --------------------------------------------------------------*/

/**
 * @brief  Função para acesso aos valores doubles armazenados na memória
 * @param  Identifier: Identificador da mensagem
 * @retval Valor double armazenado, caso o valor não seja um double retorna "FDCAN_ERRO"
 */
double CAN_Get_value_DOUBLE(uint16_t Identifier);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- INCLUSÃO DE MENSAGENS NO BUFFER DE ENVIO --------------------------------------------------------------*/

/**
 * @brief  Função para adicionar uma mensagem no buffer de envio
 * @param  Identifier: Identificador da mensagem
 * @param  Buffer: Buffer de dados para envio
 * @retval Status de execução da função
 */
FDCAN_StatusTypedef CAN_TxData(uint16_t Identifier, uint64_t Buffer);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- TRATAMENTO E ENVIO DE INTEIROS --------------------------------------------------------------*/

/**
 * @brief  Função para tratamento e envio de valores inteiros pelo barramento
 * @param  Identifier: Identificador da mensagem
 * @param  Value: Valor inteiro para envio pelo barrameto
 * @retval Status de execução da função
 */
FDCAN_StatusTypedef CAN_Send(uint16_t Identifier, int64_t Value);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- TRATAMENTO E ENVIO DE FLOATS --------------------------------------------------------------*/

/**
 * @brief  Função para tratamento e envio de valores inteiros pelo barramento
 * @param  Identifier: Identificador da mensagem
 * @param  Value: Valor float para envio pelo barrameto
 * @param  Precision: Número de casas decimais de precisão
 * @retval Status de execução da função
 */
FDCAN_StatusTypedef CAN_Send_Float(uint16_t Identifier, float Data, uint8_t Precision);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- TRATAMENTO E ENVIO DE DOUBLES --------------------------------------------------------------*/

/**
 * @brief  Função para tratamento e envio de valores inteiros pelo barramento
 * @param  Identifier: Identificador da mensagem
 * @param  Value: Valor double para envio pelo barrameto
 * @param  Precision: Número de casas decimais de precisão
 * @retval Status de execução da função
 */
FDCAN_StatusTypedef CAN_Send_Double(uint16_t Identifier, double Data, uint8_t Precision);

#endif /* CAN_LOG_H_ */
