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

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

extern FDCAN_HandleTypeDef hfdcan1; /* Variável externa de configuração da CAN */

/* USER CODE END EV */

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

/**
 * @brief  Função chamada quando detectado uma mensagem no barramento da CAN
 * @param  hfdcan: Handle da CAN || normalmente "hfdcan1"
 * @param  RxFifo0ITs: FIFO de interrupção utilizado
 * @retval ***NONE***
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	/* Pisca o  LED 2 caso tenha algo para receber pela CAN */
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

	/* Pega as informações e dados da CAN, e armazena respectivamente em RxHeader e RxData */
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);

	/* Chama a função de tratamento de dados */
	canMessageReceived(&RxHeader, RxData);

	/* Ativa novamente a notificação para caso haja algo a receber */
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			0) != HAL_OK) {
		/* Caso de errado, chama a função de erro */
		Error_Handler();
	}
}
/**
 * @brief  Configura a CAN, overwrite do .IOC
 * @param  ***NONE***
 * @retval ***NONE***
 */
void CAN_Configure_Init() {
	/* Configura os parâmetros da CAN - LEITURA DO RELATORIO */
	hfdcan1.Instance = FDCAN1;
	hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
	hfdcan1.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
	hfdcan1.Init.AutoRetransmission = DISABLE;
	hfdcan1.Init.TransmitPause = DISABLE;
	hfdcan1.Init.ProtocolException = DISABLE;
	hfdcan1.Init.NominalPrescaler = 1;
	hfdcan1.Init.NominalSyncJumpWidth = 7;
	hfdcan1.Init.NominalTimeSeg1 = 42;
	hfdcan1.Init.NominalTimeSeg2 = 27;
	hfdcan1.Init.DataPrescaler = 2;
	hfdcan1.Init.DataSyncJumpWidth = 12;
	hfdcan1.Init.DataTimeSeg1 = 12;
	hfdcan1.Init.DataTimeSeg2 = 12;
	hfdcan1.Init.MessageRAMOffset = 0;
	hfdcan1.Init.StdFiltersNbr = 0;
	hfdcan1.Init.ExtFiltersNbr = 0;
	hfdcan1.Init.RxFifo0ElmtsNbr = 1;
	hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
	hfdcan1.Init.RxFifo1ElmtsNbr = 0;
	hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
	hfdcan1.Init.RxBuffersNbr = 0;
	hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
	hfdcan1.Init.TxEventsNbr = 0;
	hfdcan1.Init.TxBuffersNbr = 0;
	hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
	hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

	/* Inicializa a CAN com os parâmetros definidos */
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
		/* Caso de errado, chama a função de erro */
		Error_Handler();
	}
}

/**
 * @brief  Inicialização do vetor de dados da CAN
 * @param  ***NONE***
 * @retval ***NONE***
 */
void CAN_stream_Init(void) {
	/* Zera cada posição do vetor de dados - Redundância */
	for (uint16_t i = 0; i < CAN_IDS_NUMBER; i++) {
		free(CAN_stream.Data_buf[i]);
		CAN_stream.Data_buf[i] = NULL;
		*CAN_stream.Data_buf[i] = 0;
		CAN_stream.Size_buf[i] = 0;
	}
}

/**
 * @brief  Inicialização da comunicação via CAN
 * @param  ***NONE***
 * @retval ***NONE***
 */
void CAN_Init() {
	/* Chama a função de configuração dos parâmetros da CAN */
	//	CAN_Configure_Init();
	/* Chama a função de limpeza do vetor de armazenamento de dados */
	CAN_stream_Init();

	/* Começa a comunicação via CAN */
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		// Caso de errado, chama a função de erro
		Error_Handler();
	}

	/* Ativa a notificação para caso haja algo a receber */
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			0) != HAL_OK) {
		/* Caso de errado, chama a função de erro */
		Error_Handler();
	}

	/* Configura os parametros para envio de mensagem */
	TxHeader.IdType = FDCAN_STANDARD_ID; // TIPO DE IDENTIFICADOR - STANDARD OU EXTENDED
	TxHeader.TxFrameType = FDCAN_DATA_FRAME; // TIPO DE FLAME - DATA OU REMOTE
	TxHeader.DataLength = FDCAN_DLC_BYTES_8; // TAMANHO DOS DADOS - 0 A 64 WORDS - CONVERTIDO PRA 4
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // INDICADOR DE ERRO - ATIVO OU PASSIVO
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;	// BIT DE INTERRUPÇÃO - ON OU OFF
	TxHeader.FDFormat = FDCAN_FD_CAN;		// TIPO DE CAN - NORMAL OU FDCAN
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // ARMAZENAMENTO DE EVENTOS DE ENVIO - ON OU OFF
	TxHeader.MessageMarker = 0;				// MASCARA DA MENSAGEM - 0 A 0xFF
}

/**
 * @brief  Função de tratamento das mensagens recebidas
 * @param  ID: Identificador da mensagem
 * @param  DATA: Buffer de dados da mensagem
 * @retval ***NONE***
 */
void canMessageReceived(FDCAN_RxHeaderTypeDef *hRxFDCAN, uint8_t *DATA) {
	/* Caso o ID passe do maior valor, a função quebra */

	/* Variavel para armazenamento do tamanho de dados */
	uint8_t SIZE_DATA = hRxFDCAN->DataLength >> 16U;

	/* Variavel para armazenamento do identificador */
	uint16_t IDENTIFIER = hRxFDCAN->Identifier;

	uint8_t TYPE_DATA = DATA[0] & 0x03;

	/* Caso o indentificador não faça parte dos ID's utilizados a função quebra */
	if (IDENTIFIER > CAN_IDS_NUMBER)
		return;

	switch (TYPE_DATA) {
	case 0x00:
		CAN_Storage_POSITIVE(IDENTIFIER, SIZE_DATA, DATA);
		break;
	case 0x01:
		CAN_Storage_NEGATIVE(IDENTIFIER, SIZE_DATA, DATA);
		break;
	case 0x02:
		CAN_Storage_FLOAT(IDENTIFIER, SIZE_DATA, DATA);
		break;
	case 0x03:
		CAN_Storage_DOUBLE(IDENTIFIER, SIZE_DATA, DATA);
		break;
	default:
		break;
	}
}

void CAN_Storage_POSITIVE(uint8_t Identifier, uint8_t Size, uint8_t *Data) {
	uint64_t value = 0;
	uint8_t *pValue = (uint8_t*) &value;
	/* Armazenando o tamanho da variável no buffer da CAN */
	CAN_stream.Size_buf[Identifier] = Size;

	CAN_stream.Type_buf[Identifier] = CAN_POSITIVE;

	/* Libera a memória para que não ocorra Hard Fault */
	free(CAN_stream.Data_buf[Identifier]);

	/* Aloca o espaço necessário para armazenamento do dado*/
	CAN_stream.Data_buf[Identifier] = malloc(Size * sizeof(uint8_t));

	for (int i = 0; i < Size; i++)
		pValue[i] = Data[i];

	value = value >> 2U;

	/* Armazena o valor na memória alocada*/
	for (int i = 0; i < Size; i++)
		CAN_stream.Data_buf[Identifier][i] = pValue[i];
}

void CAN_Storage_NEGATIVE(uint8_t Identifier, uint8_t Size, uint8_t *Data) {
	uint64_t value = 0;
	uint8_t *pValue = (uint8_t*) &value;

	/* Armazenando o tamanho da variável no buffer da CAN */
	CAN_stream.Size_buf[Identifier] = Size;

	CAN_stream.Type_buf[Identifier] = CAN_NEGATIVE;

	/* Libera a memória para que não ocorra Hard Fault */
	free(CAN_stream.Data_buf[Identifier]);

	/* Aloca o espaço necessário para armazenamento do dado*/
	CAN_stream.Data_buf[Identifier] = malloc(Size * sizeof(uint8_t));

	for (int i = 0; i < Size; i++)
		pValue[i] = Data[i];

	value = value >> 2U;

	/* Armazena o valor na memória alocada*/
	for (int i = 0; i < Size; i++)
		CAN_stream.Data_buf[Identifier][i] = pValue[i];
}

void CAN_Storage_FLOAT(uint8_t Identifier, uint8_t Size, uint8_t *Data) {
	uint64_t value = 0;
	uint8_t *pValue = (uint8_t*) &value;

	/* Armazenando o tamanho da variável no buffer da CAN */
	CAN_stream.Size_buf[Identifier] = Size;

	CAN_stream.Type_buf[Identifier] = CAN_FLOAT;

	/* Libera a memória para que não ocorra Hard Fault */
	free(CAN_stream.Data_buf[Identifier]);

	/* Aloca o espaço necessário para armazenamento do dado*/
	CAN_stream.Data_buf[Identifier] = malloc(Size * sizeof(uint8_t));

	for (int i = 0; i < Size; i++)
		pValue[i] = Data[i];

	value = value >> 2U;

	/* Armazena o valor na memória alocada*/
	for (int i = 0; i < Size; i++)
		CAN_stream.Data_buf[Identifier][i] = pValue[i];
}

void CAN_Storage_DOUBLE(uint8_t Identifier, uint8_t Size, uint8_t *Data) {
	uint64_t value = 0;
	uint8_t *pValue = (uint8_t*) &value;

	/* Armazenando o tamanho da variável no buffer da CAN */
	CAN_stream.Size_buf[Identifier] = Size;

	CAN_stream.Type_buf[Identifier] = CAN_DOUBLE;

	/* Libera a memória para que não ocorra Hard Fault */
	free(CAN_stream.Data_buf[Identifier]);

	/* Aloca o espaço necessário para armazenamento do dado*/
	CAN_stream.Data_buf[Identifier] = malloc(Size * sizeof(uint8_t));

	for (uint64_t i = 0; i < Size; i++)
		pValue[i] = Data[i];

	value = value >> 2U;

	/* Armazena o valor na memória alocada*/
	for (int i = 0; i < Size; i++)
		CAN_stream.Data_buf[Identifier][i] = pValue[i];
}

/**
 * @brief  Envio de mensagem pelo barramento CAN
 * @param  ID: Identificador da mensagem
 * @param  DATA: Buffer de dados da mensagem
 * @retval ***NONE***
 */
void CAN_TxData(uint16_t Identifier, uint64_t Data) {
	uint64_t *pData = &Data;
	uint32_t Size_data = 0;

	for (int i = 0; i < 8; i++)
		if (Data >> 8 * i == 0) {
			Size_data = i << 16U;
			break;
		}

	/* Armazena o identificador da mensagem no struct de informação (TxHeader) */
	TxHeader.Identifier = Identifier;

	TxHeader.DataLength = Size_data;

	/* Envia os dados recebidos na chamada (data) pela CAN, de acordo com as informações de TxHeader */
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, (uint8_t*) pData)
			!= HAL_OK) {
		/* Caso de errado, chama a função de erro */
		Error_Handler();
	}
}

int64_t CAN_Get_value(uint16_t Identifier) {
	int64_t VALUE = 0;
	uint8_t *pValue = (uint8_t*) &VALUE;

	switch (CAN_stream.Type_buf[Identifier]) {
	case CAN_POSITIVE:
		for (int i = 0; i < CAN_stream.Size_buf[Identifier]; i++)
			pValue[i] = CAN_stream.Data_buf[Identifier][i];
		break;
	case CAN_NEGATIVE:
		for (int i = 0; i < CAN_stream.Size_buf[Identifier]; i++)
			pValue[i] = CAN_stream.Data_buf[Identifier][i];
		VALUE = -VALUE;
		break;
	default:
		break;
	}

	return VALUE;
}

float CAN_Get_value_FLOAT(uint16_t Identifier) {
	if (CAN_stream.Type_buf[Identifier] != CAN_FLOAT)
		return 0;
	uint64_t DATA_STORAGE = 0;
	uint8_t PRECISION = 0;
	uint8_t SIGNAL = 0;
	float VALUE = 0;
	uint8_t *pData_Storage = (uint8_t*) &DATA_STORAGE;

	for (int i = 0; i < CAN_stream.Size_buf[Identifier]; i++)
		pData_Storage[i] = CAN_stream.Data_buf[Identifier][i];

	PRECISION = DATA_STORAGE & 0x3F;

	SIGNAL = (DATA_STORAGE & 0x40) >> 6;

	VALUE = (DATA_STORAGE >> 7) * pow(10, -PRECISION);

	if (SIGNAL == 1)
		VALUE = -VALUE;

	return VALUE;
}

double CAN_Get_value_DOUBLE(uint16_t Identifier) {
	if (CAN_stream.Type_buf[Identifier] != CAN_DOUBLE)
		return 0;
	uint64_t DATA_STORAGE = 0;
	uint8_t PRECISION = 0;
	uint8_t SIGNAL = 0;
	float VALUE = 0;
	uint8_t *pData_Storage = (uint8_t*) &DATA_STORAGE;

	for (int i = 0; i < CAN_stream.Size_buf[Identifier]; i++)
		pData_Storage[i] = CAN_stream.Data_buf[Identifier][i];

	PRECISION = DATA_STORAGE & 0x1FF;

	SIGNAL = (DATA_STORAGE & 0x200) >> 9;

	VALUE = (DATA_STORAGE >> 10) * pow(10, -PRECISION);

	if (SIGNAL == 1)
		VALUE = -VALUE;

	return VALUE;
}

/**
 * @brief  Envio de mensagem pelo barramento CAN
 * @param  ID: Identificador da mensagem
 * @param  DATA: Buffer de dados da mensagem
 * @retval ***NONE***
 */
void CAN_Send(uint16_t Identifier, int64_t Data) {

	if (Data > 0)

		Data = (Data << 2) | 0x00;
	else
		Data = ((-Data) << 2) | 0x01;

	CAN_TxData(Identifier, Data);
}

void CAN_Send_Float(uint16_t Identifier, float Data, uint8_t Precision) {
	int64_t Valor = Data * pow(10, Precision);

	if (Data > 0)
		Valor = (Valor << 9) | 0x000 | (Precision << 2) | 0x02;
	else
		Valor = ((-Valor) << 9) | 0x100 | (Precision << 2) | 0x02;

	CAN_TxData(Identifier, Valor);
}

void CAN_Send_Double(uint16_t Identifier, double Data, uint8_t Precision) {
	int64_t Valor = Data * pow(10, Precision);

	if (Data > 0)
		Valor = (Valor << 12) | 0x000 | (Precision << 2) | 0x03;
	else
		Valor = ((-Valor) << 12) | 0x800 | (Precision << 2) | 0x03;

	CAN_TxData(Identifier, Valor);
}

/* USER CODE END PF */
