/*
 *      Comunicação via CAN - CAN.c
 *
 *      Data: 13 de junho, 2023
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

CAN_Buffer_t can_vector[CAN_IDS_NUMBER]; /*Vetor para armazenamento de
 todos os dados da CAN*/

can_vector = malloc(CAN_IDS_NUMBER * sizeof(uint8_t*)); 

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
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	/* Pisca o  LED 2 caso tenha algo para receber pela CAN */
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);

	/* Pega as informações e dados da CAN, e armazena respectivamente em RxHeader e RxData */
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);

	/* Chama a função de tratamento de dados */
	canMessageReceived(RxHeader.Identifier, RxData);

	/* Ativa novamente a notificação para caso haja algo a receber */
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
									   0) != HAL_OK)
	{
		/* Caso de errado, chama a função de erro */
		Error_Handler();
	}
}

/**
 * @brief  Configura a CAN, overwrite do .IOC
 * @param  ***NONE***
 * @retval ***NONE***
 */
void CAN_Configure_Init()
{
	/* Configura os parâmetros da CAN - LEITURA DO RELATORIO */
	hfdcan1.Instance = FDCAN1;
	hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
	hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
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
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
	{
		/* Caso de errado, chama a função de erro */
		Error_Handler();
	}
}

/**
 * @brief  Inicialização do vetor de dados da CAN
 * @param  ***NONE***
 * @retval ***NONE***
 */
void CAN_Vector_Init(void)
{
	/* Zera cada posição do vetor de dados - Redundância */
	for (uint16_t i = 0; i < CAN_IDS_NUMBER; i++)
	{
		can_vector[i].word_0 = 0;
		can_vector[i].word_1 = 0;
		can_vector[i].word_2 = 0;
		can_vector[i].word_3 = 0;
	}
}

/**
 * @brief  Inicialização da comunicação via CAN
 * @param  ***NONE***
 * @retval ***NONE***
 */
void CAN_Init()
{
	/* Chama a função de configuração dos parâmetros da CAN */
	//	CAN_Configure_Init();

	/* Chama a função de limpeza do vetor de armazenamento de dados */
	Clean_CAN_vector();

	/* Começa a comunicação via CAN */
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		// Caso de errado, chama a função de erro
		Error_Handler();
	}

	/* Ativa a notificação para caso haja algo a receber */
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
									   0) != HAL_OK)
	{
		/* Caso de errado, chama a função de erro */
		Error_Handler();
	}

	/* Configura os parametros para envio de mensagem */
	TxHeader.IdType = FDCAN_STANDARD_ID;			  // TIPO DE IDENTIFICADOR - STANDARD OU EXTENDED
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;		  // TIPO DE FLAME - DATA OU REMOTE
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;		  // TAMANHO DOS DADOS - 0 A 64 WORDS - CONVERTIDO PRA 4
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // INDICADOR DE ERRO - ATIVO OU PASSIVO
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;			  // BIT DE INTERRUPÇÃO - ON OU OFF
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;			  // TIPO DE CAN - NORMAL OU FDCAN
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // ARMAZENAMENTO DE EVENTOS DE ENVIO - ON OU OFF
	TxHeader.MessageMarker = 0;						  // MASCARA DA MENSAGEM - 0 A 0xFF
}

/**
 * @brief  Função de tratamento das mensagens recebidas
 * @param  ID: Identificador da mensagem
 * @param  DATA: Buffer de dados da mensagem
 * @retval ***NONE***
 */
void canMessageReceived(uint16_t ID, uint8_t *DATA)
{
	/* Caso o ID passe do maior valor, a função quebra */
	if (ID > CAN_IDS_NUMBER - 1)
		return;
	/* Converte os dados que chegam em 'byte' (uint8_t), em uma 'word' (uint16_t) */
	uint16_t *data_word = (uint16_t *)DATA;
	/* Armazena os dados no vetor */
	can_vector[ID].word_0 = data_word[0];
	can_vector[ID].word_1 = data_word[1];
	can_vector[ID].word_2 = data_word[2];
	can_vector[ID].word_3 = data_word[3];
}

/**
 * @brief  Envio de mensagem pelo barramento CAN
 * @param  ID: Identificador da mensagem
 * @param  DATA: Buffer de dados da mensagem
 * @retval ***NONE***
 */
void CAN_TxData(uint16_t id, uint16_t *data)
{
	/* Armazena o identificador da mensagem no struct de informação (TxHeader) */
	TxHeader.Identifier = id;
	/* Envia os dados recebidos na chamada (data) pela CAN, de acordo com as informações de TxHeader */
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, (uint8_t *)data) != HAL_OK)
	{
		/* Caso de errado, chama a função de erro */
		Error_Handler();
	}
}

/* USER CODE END PF */
