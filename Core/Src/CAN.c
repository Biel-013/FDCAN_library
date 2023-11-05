/*
 *      Comunicação via FDCAN - FDCAN.c
 *
 *      Data: 01 de novembro, 2023
 *      Autor: Gabriel Luiz
 *      Contato: (31) 97136-4334 || gabrielluiz.eletro@gmail.com
 */
/*
 * 		- Links Úteis -
 *
 *      DATASHEET: https://www.st.com/content/ccc/resource/training/technical/product_training/group0/35/ed/76/ef/91/30/44/f7/STM32H7-Peripheral-Flexible_Datarate_Controller_Area_Network_FDCAN/files/STM32H7-Peripheral-Flexible_Datarate_Controller_Area_Network_FDCAN.pdf/_jcr_content/translations/en.STM32H7-Peripheral-Flexible_Datarate_Controller_Area_Network_FDCAN.pdf
 *      FDCAN normal DOCUMENTO: https://controllerstech.com/fdcan-normal-mode-stm32/
 *      FDCAN normal mode VÍDEO: https://www.youtube.com/watch?v=sY1ie-CnOR0&t=7s
 */

#include "CAN.h"
#include <stdlib.h>
#include <math.h>

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

extern FDCAN_HandleTypeDef hfdcan1; /* Variável externa de configuração da CAN */

/* USER CODE END EV */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

FDCAN_HandleTypeDef *hFDCAN = &hfdcan1; /* Handler de configuração da CAN */

#define TIME_TO_BREAK 10 /* Tempo máximo para envio de mensagem na CAN (ms) */

/* USER CODE END PD */

/* External functions ------------------------------------------------------------*/
/* USER CODE BEGIN EF */

extern void Error_Handler(); /* Função utilizada para tratamento de erros */

/* USER CODE END EF */

/* Private variables --------------------------------------------------------*/
/* USER CODE BEGIN PV */

CAN_Buffer_t CAN_stream; /*Vetor para armazenamento de
 todos os dados da CAN*/

FDCAN_TxHeaderTypeDef TxHeader; /*Struct de armazenamento temporario de
 de informações e dados para envio na CAN - Não inclui os dados */

FDCAN_RxHeaderTypeDef RxHeader; /*Struct de armazenamento temporario de
 de informações recebidas pela CAN - Não inclui os dados */

uint8_t RxData[8]; /*Vetor para armazenamento temporario de dados recebidos
 pela CAN*/

/* USER CODE END PV */

/* Private functions ------------------------------------------------------------*/
/* USER CODE BEGIN PF */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- CALLBACK DE RECEBIMENTO DA CAN --------------------------------------------------------------------------------*/

/**
 * @brief  Função chamada quando detectado uma mensagem no barramento da CAN
 * @param  hfdcan: Handle da CAN || normalmente "hfdcan1"
 * @param  RxFifo0ITs: FIFO em que foi detectado a mensagem
 * @retval ***NONE***
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	/* Pisca o  LED 2 caso tenha algo para receber pela CAN */
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

	/* Pega as informações e dados da CAN, e armazena respectivamente em RxHeader e RxData */
	HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);

	/* Chama a função de tratamento de dados */
	CAN_Stream_ReceiveCallback(&RxHeader, RxData);

	/* Ativa novamente a notificação para caso haja algo a receber */
	if (HAL_FDCAN_ActivateNotification(&hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
									   0) != HAL_OK)
	{
		/* Caso de errado, chama a função de erro */
		Error_Handler();
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- TRATAMENTO DE MENSAGENS RECEBIDAS -----------------------------------------------------------------------------*/

/**
 * @brief  Função de tratamento das mensagens recebidas
 * @param  hRxFDCAN: Handler com as innformações do flame recebido
 * @param  Buffer: Buffer com os dados e informações da mensagem
 * @retval Status de execução da função
 */
FDCAN_StatusTypedef CAN_Stream_ReceiveCallback(FDCAN_RxHeaderTypeDef *hRxFDCAN, uint8_t *Buffer)
{
	/* Caso o indentificador não faça parte dos ID's utilizados a função quebra */
	if (hRxFDCAN->Identifier > CAN_IDS_NUMBER)
		return;

	/* Variavel para armazenamento do tamanho de dados */
	uint8_t SIZE_DATA = hRxFDCAN->DataLength >> 16U;

	uint8_t TYPE_DATA = Buffer[0] & 0x03;

	switch (TYPE_DATA)
	{
	case FDCAN_POSITIVE:
		CAN_Storage_POSITIVE(hRxFDCAN->Identifier, SIZE_DATA, Buffer);
		break;
	case FDCAN_NEGATIVE:
		CAN_Storage_NEGATIVE(hRxFDCAN->Identifier, SIZE_DATA, Buffer);
		break;
	case FDCAN_FLOAT:
		CAN_Storage_FLOAT(hRxFDCAN->Identifier, SIZE_DATA, Buffer);
		break;
	case FDCAN_DOUBLE:
		CAN_Storage_DOUBLE(hRxFDCAN->Identifier, SIZE_DATA, Buffer);
		break;
	default:
		break;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- PARAMETROS DE CONFIGURAÇÃO DA CAN --------------------------------------------------------------*/

/**
 * @brief  Configura a CAN, overwrite das configurações do .IOC
 * @param  ***NONE***
 * @retval Status de execução da função
 */
FDCAN_StatusTypedef CAN_Configure_Init()
{
	/* Configura os parâmetros da CAN - LEITURA DO RELATORIO */
	hFDCAN->Instance = FDCAN1;
	hFDCAN->Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
	hFDCAN->Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
	hFDCAN->Init.AutoRetransmission = DISABLE;
	hFDCAN->Init.TransmitPause = DISABLE;
	hFDCAN->Init.ProtocolException = DISABLE;
	hFDCAN->Init.NominalPrescaler = 1;
	hFDCAN->Init.NominalSyncJumpWidth = 7;
	hFDCAN->Init.NominalTimeSeg1 = 42;
	hFDCAN->Init.NominalTimeSeg2 = 27;
	hFDCAN->Init.DataPrescaler = 2;
	hFDCAN->Init.DataSyncJumpWidth = 12;
	hFDCAN->Init.DataTimeSeg1 = 12;
	hFDCAN->Init.DataTimeSeg2 = 12;
	hFDCAN->Init.MessageRAMOffset = 0;
	hFDCAN->Init.StdFiltersNbr = 0;
	hFDCAN->Init.ExtFiltersNbr = 0;
	hFDCAN->Init.RxFifo0ElmtsNbr = 1;
	hFDCAN->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
	hFDCAN->Init.RxFifo1ElmtsNbr = 0;
	hFDCAN->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
	hFDCAN->Init.RxBuffersNbr = 0;
	hFDCAN->Init.RxBufferSize = FDCAN_DATA_BYTES_8;
	hFDCAN->Init.TxEventsNbr = 0;
	hFDCAN->Init.TxBuffersNbr = 0;
	hFDCAN->Init.TxFifoQueueElmtsNbr = 32;
	hFDCAN->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	hFDCAN->Init.TxElmtSize = FDCAN_DATA_BYTES_8;

	/* Inicializa a CAN com os parâmetros definidos */
	if (HAL_FDCAN_Init(hFDCAN) != HAL_OK)
		/* Caso de errado, retorna erro */
		return FDCAN_ERROR;
	else
		/* Caso de tudo certo, retorna ok */
		return FDCAN_OK;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- LIMPEZA DOS BUFFERS DE ARMAZENAMENTO --------------------------------------------------------------*/

/**
 * @brief  Inicialização dos buffers de armazenamento das mensagens da CAN
 * @param  ***NONE***
 * @retval ***NONE***
 */
void CAN_Clean_Buffers(void)
{
	/* Zera cada posição do vetor de dados - Redundância */
	for (uint16_t i = 0; i < CAN_IDS_NUMBER; i++)
	{
		free(CAN_stream.Data_buf[i]);
		CAN_stream.Data_buf[i] = NULL;
		*CAN_stream.Data_buf[i] = 0;
		CAN_stream.Size_buf[i] = 0;
		CAN_stream.Type_buf[i] = FDCAN_FREE;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- INÍCIO DA COMUNICAÇÃO VIA FDCAN --------------------------------------------------------------*/

/**
 * @brief  Inicialização da comunicação via FDCAN
 * @param  ***NONE***
 * @retval Status de execução da função
 */
FDCAN_StatusTypedef CAN_Init(void)
{
	/* Chama a função de configuração dos parâmetros da CAN */
	if (CAN_Configure_Init() != FDCAN_OK)
		/* Caso de errado, retorna erro */
		return FDCAN_ERROR;

	/* Chama a função de limpeza do vetor de armazenamento de dados */
	CAN_Clean_Buffers();

	/* Começa a comunicação via CAN */
	if (HAL_FDCAN_Start(hFDCAN) != HAL_OK)
		/* Caso de errado, retorna erro */
		return FDCAN_ERROR;

	/* Ativa a notificação para caso haja algo a receber */
	if (HAL_FDCAN_ActivateNotification(hFDCAN, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
									   0) != HAL_OK)
		/* Caso de errado, retorna erro */
		return FDCAN_ERROR;

	/* Configura os parametros para envio de mensagem */
	TxHeader.IdType = FDCAN_STANDARD_ID;			  // TIPO DE IDENTIFICADOR - STANDARD OU EXTENDED
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;		  // TIPO DE FLAME - DATA OU REMOTE
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;		  // TAMANHO DOS DADOS - 0 A 64 WORDS - CONVERTIDO PRA 4
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // INDICADOR DE ERRO - ATIVO OU PASSIVO
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;			  // BIT DE INTERRUPÇÃO - ON OU OFF
	TxHeader.FDFormat = FDCAN_FD_CAN;				  // TIPO DE CAN - NORMAL OU FDCAN
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // ARMAZENAMENTO DE EVENTOS DE ENVIO - ON OU OFF
	TxHeader.MessageMarker = 0;						  // MASCARA DA MENSAGEM - 0 A 0xFF

	/* Caso de tudo certo, retorna ok */
	return FDCAN_OK;
}

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
void CAN_Storage_POSITIVE(uint8_t Identifier, uint8_t Size, uint8_t *Buffer)
{
	uint64_t VALUE = 0;
	uint8_t *pValue = (uint8_t *)&VALUE;
	/* Armazenando o tamanho da variável no buffer da CAN */
	CAN_stream.Size_buf[Identifier] = Size;

	CAN_stream.Type_buf[Identifier] = FDCAN_POSITIVE;

	/* Libera a memória para que não ocorra Hard Fault */
	free(CAN_stream.Data_buf[Identifier]);

	/* Aloca o espaço necessário para armazenamento do dado*/
	CAN_stream.Data_buf[Identifier] = malloc(Size * sizeof(uint8_t));

	for (int i = 0; i < Size; i++)
		pValue[i] = Buffer[i];

	VALUE = VALUE >> 2U;

	/* Armazena o valor na memória alocada*/
	for (int i = 0; i < Size; i++)
		CAN_stream.Data_buf[Identifier][i] = pValue[i];
}

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
void CAN_Storage_NEGATIVE(uint8_t Identifier, uint8_t Size, uint8_t *Buffer)
{
	uint64_t VALUE = 0;
	uint8_t *pValue = (uint8_t *)&VALUE;

	/* Armazenando o tamanho da variável no buffer da CAN */
	CAN_stream.Size_buf[Identifier] = Size;

	CAN_stream.Type_buf[Identifier] = FDCAN_NEGATIVE;

	/* Libera a memória para que não ocorra Hard Fault */
	free(CAN_stream.Data_buf[Identifier]);

	/* Aloca o espaço necessário para armazenamento do dado*/
	CAN_stream.Data_buf[Identifier] = malloc(Size * sizeof(uint8_t));

	for (int i = 0; i < Size; i++)
		pValue[i] = Buffer[i];

	VALUE = VALUE >> 2U;

	/* Armazena o valor na memória alocada*/
	for (int i = 0; i < Size; i++)
		CAN_stream.Data_buf[Identifier][i] = pValue[i];
}

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
void CAN_Storage_FLOAT(uint8_t Identifier, uint8_t Size, uint8_t *Buffer)
{
	uint64_t VALUE = 0;
	uint8_t *pValue = (uint8_t *)&VALUE;

	/* Armazenando o tamanho da variável no buffer da CAN */
	CAN_stream.Size_buf[Identifier] = Size;

	CAN_stream.Type_buf[Identifier] = FDCAN_FLOAT;

	/* Libera a memória para que não ocorra Hard Fault */
	free(CAN_stream.Data_buf[Identifier]);

	/* Aloca o espaço necessário para armazenamento do dado*/
	CAN_stream.Data_buf[Identifier] = malloc(Size * sizeof(uint8_t));

	for (int i = 0; i < Size; i++)
		pValue[i] = Buffer[i];

	VALUE = VALUE >> 2U;

	/* Armazena o valor na memória alocada*/
	for (int i = 0; i < Size; i++)
		CAN_stream.Data_buf[Identifier][i] = pValue[i];
}

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
void CAN_Storage_DOUBLE(uint8_t Identifier, uint8_t Size, uint8_t *Buffer)
{
	uint64_t VALUE = 0;
	uint8_t *pValue = (uint8_t *)&VALUE;

	/* Armazenando o tamanho da variável no buffer da CAN */
	CAN_stream.Size_buf[Identifier] = Size;

	CAN_stream.Type_buf[Identifier] = FDCAN_DOUBLE;

	/* Libera a memória para que não ocorra Hard Fault */
	free(CAN_stream.Data_buf[Identifier]);

	/* Aloca o espaço necessário para armazenamento do dado*/
	CAN_stream.Data_buf[Identifier] = malloc(Size * sizeof(uint8_t));

	for (uint64_t i = 0; i < Size; i++)
		pValue[i] = Buffer[i];

	VALUE = VALUE >> 2U;

	/* Armazena o valor na memória alocada*/
	for (int i = 0; i < Size; i++)
		CAN_stream.Data_buf[Identifier][i] = pValue[i];
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- ACESSO AOS VALORES INTEIROS ARMAZENADOS --------------------------------------------------------------*/

/**
 * @brief  Função para acesso aos valores inteiros armazenados na memória
 * @param  Identifier: Identificador da mensagem
 * @retval Valor inteiro armazenado, caso o valor não seja um inteiro retorna "FDCAN_ERRO"
 */
int64_t CAN_Get_value(uint16_t Identifier)
{
	int64_t VALUE = 0;
	uint8_t *pValue = (uint8_t *)&VALUE;

	switch (CAN_stream.Type_buf[Identifier])
	{
	case FDCAN_POSITIVE:
		for (int i = 0; i < CAN_stream.Size_buf[Identifier]; i++)
			pValue[i] = CAN_stream.Data_buf[Identifier][i];
		break;
	case FDCAN_NEGATIVE:
		for (int i = 0; i < CAN_stream.Size_buf[Identifier]; i++)
			pValue[i] = CAN_stream.Data_buf[Identifier][i];
		VALUE = -VALUE;
		break;
	default:
		return FDCAN_ERROR;
	}

	return VALUE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- ACESSO AOS VALORES FLOATS ARMAZENADOS --------------------------------------------------------------*/

/**
 * @brief  Função para acesso aos valores floats armazenados na memória
 * @param  Identifier: Identificador da mensagem
 * @retval Valor float armazeenado, caso o valor não seja um float retorna "FDCAN_ERRO"
 */
float CAN_Get_value_FLOAT(uint16_t Identifier)
{
	if (CAN_stream.Type_buf[Identifier] != FDCAN_FLOAT)
		return FDCAN_ERROR;

	uint64_t DATA_STORAGE = 0;
	uint8_t PRECISION = 0;
	uint8_t SIGNAL = 0;
	float VALUE = 0;
	uint8_t *pData_Storage = (uint8_t *)&DATA_STORAGE;

	for (int i = 0; i < CAN_stream.Size_buf[Identifier]; i++)
		pData_Storage[i] = CAN_stream.Data_buf[Identifier][i];

	PRECISION = DATA_STORAGE & 0x3F;

	SIGNAL = (DATA_STORAGE & 0x40) >> 6;

	VALUE = (DATA_STORAGE >> 7) * pow(10, -PRECISION);

	if (SIGNAL == 1)
		VALUE = -VALUE;

	return VALUE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- ACESSO AOS VALORES DOUBLES ARMAZENADOS --------------------------------------------------------------*/

/**
 * @brief  Função para acesso aos valores doubles armazenados na memória
 * @param  Identifier: Identificador da mensagem
 * @retval Valor double armazenado, caso o valor não seja um double retorna "FDCAN_ERRO"
 */
double CAN_Get_value_DOUBLE(uint16_t Identifier)
{
	if (CAN_stream.Type_buf[Identifier] != FDCAN_DOUBLE)
		return FDCAN_ERROR;

	uint64_t DATA_STORAGE = 0;
	uint8_t PRECISION = 0;
	uint8_t SIGNAL = 0;
	float VALUE = 0;
	uint8_t *pData_Storage = (uint8_t *)&DATA_STORAGE;

	for (int i = 0; i < CAN_stream.Size_buf[Identifier]; i++)
		pData_Storage[i] = CAN_stream.Data_buf[Identifier][i];

	PRECISION = DATA_STORAGE & 0x1FF;

	SIGNAL = (DATA_STORAGE & 0x200) >> 9;

	VALUE = (DATA_STORAGE >> 10) * pow(10, -PRECISION);

	if (SIGNAL == 1)
		VALUE = -VALUE;

	return VALUE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- INCLUSÃO DE MENSAGENS NO BUFFER DE ENVIO --------------------------------------------------------------*/

/**
 * @brief  Função para adicionar uma mensagem no buffer de envio
 * @param  Identifier: Identificador da mensagem
 * @param  Buffer: Buffer de dados para envio
 * @retval Status de execução da função
 */
FDCAN_StatusTypedef CAN_TxData(uint16_t Identifier, uint64_t Buffer)
{
	uint64_t TIME = HAL_GetTick();
	uint64_t *pBuffer = &Buffer;
	uint32_t SIZE_DATA = 0;

	for (int i = 0; i < 8; i++)
		if (Buffer >> 8 * i == 0)
		{
			SIZE_DATA = i << 16U;
			break;
		}

	/* Armazena o identificador da mensagem no struct de informação (TxHeader) */
	TxHeader.Identifier = Identifier;

	TxHeader.DataLength = SIZE_DATA;

	while (HAL_FDCAN_GetTxFifoFreeLevel(hFDCAN) == 0)
		if (HAL_GetTick() - TIME > TIME_TO_BREAK)
			return;

	/* Envia os dados recebidos na chamada (data) pela CAN, de acordo com as informações de TxHeader */
	if (HAL_FDCAN_AddMessageToTxFifoQ(hFDCAN, &TxHeader, (uint8_t *)pBuffer) != HAL_OK)
	{
		/* Caso de errado, chama a função de erro */
		Error_Handler();
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---- TRATAMENTO E ENVIO DE INTEIROS --------------------------------------------------------------*/

/**
 * @brief  Função para tratamento e envio de valores inteiros pelo barramento
 * @param  Identifier: Identificador da mensagem
 * @param  Value: Valor inteiro para envio pelo barrameto
 * @retval Status de execução da função
 */
FDCAN_StatusTypedef CAN_Send(uint16_t Identifier, int64_t Value)
{

	if (Value >= 0)

		Value = (Value << 2) | FDCAN_POSITIVE;
	else
		Value = ((-Value) << 2) | FDCAN_NEGATIVE;

	CAN_TxData(Identifier, Value);
}

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
FDCAN_StatusTypedef CAN_Send_Float(uint16_t Identifier, float Value, uint8_t Precision)
{
	int64_t VALOR = Value * pow(10, Precision);

	if (Value >= 0)
		VALOR = (VALOR << 9) | 0x000 | (Precision << 2) | FDCAN_FLOAT;
	else
		VALOR = ((-VALOR) << 9) | 0x100 | (Precision << 2) | FDCAN_FLOAT;

	CAN_TxData(Identifier, VALOR);
}

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
FDCAN_StatusTypedef CAN_Send_Double(uint16_t Identifier, double Value, uint8_t Precision)
{
	int64_t VALOR = Value * pow(10, Precision);

	if (Value >= 0)
		VALOR = (VALOR << 12) | 0x000 | (Precision << 2) | FDCAN_DOUBLE;
	else
		VALOR = ((-VALOR) << 12) | 0x800 | (Precision << 2) | FDCAN_DOUBLE;

	CAN_TxData(Identifier, VALOR);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* USER CODE END PF */
