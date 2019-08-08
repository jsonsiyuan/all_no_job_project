
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include<string.h>
#include "device_manager.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define user_rfid_all 50
//RFID UART 2
typedef struct
{
	uint32_t user_crc1;
	uint32_t user_crc2;
}rfid_DATA_T;

rfid_DATA_T user_rfid_data[user_rfid_all];

static int rfid_antenna_num;//0 is 1 1is other
static int rfid_antenna_num_tmp;//0 is 1 1is other

#define rfid_recvbuff_len 200
#define rfid_pack_len 200

uint8_t rfid_tmp_data=0;

uint8_t rfid_recvstart=0;
uint8_t rfid_recvend=0;

uint8_t rfid_recvbuff[rfid_recvbuff_len];
uint8_t rfid_pack[rfid_pack_len];

//??1
uint8_t cmd_set_work_antenna_1[6]={0xA0, 0x04, 0x01, 0x74 ,0x00, 0xE7};
uint8_t cmd_real_time_inventory_1[6]={0xA0 ,0x04, 0x01, 0x89 ,0x01, 0xD1};
//??2
uint8_t cmd_set_work_antenna_2[6]={0XA0, 0X04, 0X01, 0X74, 0X01, 0XE6};
uint8_t cmd_real_time_inventory_2[6]={0XA0, 0X04, 0X01, 0X89, 0X01, 0XD1};

//GPRS UART 1
typedef struct
{
	uint8_t head;
	uint8_t Command_flag;
	uint8_t Index;
	uint8_t In;
	uint8_t crc[2];
	uint8_t end;
}GPRS_DATA_T;

GPRS_DATA_T gprs_data={
	.head='{',
	.Command_flag=0x04,
	.end='}',
};
#define debug 0

#define OUT 0x0F
#define IN    0xF0

#define gprs_pack_len 200
#define gprs_recvbuff_len 200
#define gprs_sendbuff_len sizeof(GPRS_DATA_T)

uint8_t gprs_tmp_data=0;

uint8_t gprs_recvstart=0;
uint8_t gprs_recvend=0;

uint8_t firstId[12];
uint8_t secondId[12];

uint8_t gprs_recvbuff[gprs_recvbuff_len];
uint8_t gprs_pack[gprs_pack_len];

uint8_t gprs_sendbuff[gprs_sendbuff_len];
uint8_t gprs_resendbuff[gprs_sendbuff_len];


uint8_t gprs_start_cmd[5]={'{', 0x03,0x00,0x00, '}'};
uint8_t gprs_ack_ok[6]={'{',0x02,0x0f,0x00,0x00,'}'};
uint8_t gprs_ack_fail[6]={'{',0x02,0xf0,0x00,0x00,'}'};
uint8_t gprs_send_length;

uint8_t gprs_work_flag=0;
uint32_t gprs_work_num=0;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SendPack(uint8_t index,int INout);
 uint8_t  CheckSum( uint8_t *uBuff,  uint8_t uBuffLen);
void CheckRfiddata(uint8_t data);
void Checkgprsdata(uint8_t data);
int rfid_Checkonepack(uint8_t *pack);
int gprs_Checkonepack(uint8_t *pack);
uint8_t find_crc_index(uint32_t crc_data);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static const unsigned int crc32tab[] = {
 0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL,
 0x076dc419L, 0x706af48fL, 0xe963a535L, 0x9e6495a3L,
 0x0edb8832L, 0x79dcb8a4L, 0xe0d5e91eL, 0x97d2d988L,
 0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L, 0x90bf1d91L,
 0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL,
 0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L,
 0x136c9856L, 0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL,
 0x14015c4fL, 0x63066cd9L, 0xfa0f3d63L, 0x8d080df5L,
 0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L, 0xa2677172L,
 0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
 0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L,
 0x32d86ce3L, 0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L,
 0x26d930acL, 0x51de003aL, 0xc8d75180L, 0xbfd06116L,
 0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L, 0xb8bda50fL,
 0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L,
 0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL,
 0x76dc4190L, 0x01db7106L, 0x98d220bcL, 0xefd5102aL,
 0x71b18589L, 0x06b6b51fL, 0x9fbfe4a5L, 0xe8b8d433L,
 0x7807c9a2L, 0x0f00f934L, 0x9609a88eL, 0xe10e9818L,
 0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,
 0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL,
 0x6c0695edL, 0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L,
 0x65b0d9c6L, 0x12b7e950L, 0x8bbeb8eaL, 0xfcb9887cL,
 0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L, 0xfbd44c65L,
 0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L,
 0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL,
 0x4369e96aL, 0x346ed9fcL, 0xad678846L, 0xda60b8d0L,
 0x44042d73L, 0x33031de5L, 0xaa0a4c5fL, 0xdd0d7cc9L,
 0x5005713cL, 0x270241aaL, 0xbe0b1010L, 0xc90c2086L,
 0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
 0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L,
 0x59b33d17L, 0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL,
 0xedb88320L, 0x9abfb3b6L, 0x03b6e20cL, 0x74b1d29aL,
 0xead54739L, 0x9dd277afL, 0x04db2615L, 0x73dc1683L,
 0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L,
 0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L,
 0xf00f9344L, 0x8708a3d2L, 0x1e01f268L, 0x6906c2feL,
 0xf762575dL, 0x806567cbL, 0x196c3671L, 0x6e6b06e7L,
 0xfed41b76L, 0x89d32be0L, 0x10da7a5aL, 0x67dd4accL,
 0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,
 0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L,
 0xd1bb67f1L, 0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL,
 0xd80d2bdaL, 0xaf0a1b4cL, 0x36034af6L, 0x41047a60L,
 0xdf60efc3L, 0xa867df55L, 0x316e8eefL, 0x4669be79L,
 0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L,
 0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL,
 0xc5ba3bbeL, 0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L,
 0xc2d7ffa7L, 0xb5d0cf31L, 0x2cd99e8bL, 0x5bdeae1dL,
 0x9b64c2b0L, 0xec63f226L, 0x756aa39cL, 0x026d930aL,
 0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
 0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L,
 0x92d28e9bL, 0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L,
 0x86d3d2d4L, 0xf1d4e242L, 0x68ddb3f8L, 0x1fda836eL,
 0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L, 0x18b74777L,
 0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL,
 0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L,
 0xa00ae278L, 0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L,
 0xa7672661L, 0xd06016f7L, 0x4969474dL, 0x3e6e77dbL,
 0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L, 0x37d83bf0L,
 0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
 0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L,
 0xbad03605L, 0xcdd70693L, 0x54de5729L, 0x23d967bfL,
 0xb3667a2eL, 0xc4614ab8L, 0x5d681b02L, 0x2a6f2b94L,
 0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL, 0x2d02ef8dL
};
 
 
static unsigned int crc32( const unsigned char *buf, unsigned int size)
{
     unsigned int i, crc;
     crc = 0xFFFFFFFF;
 
 
     for (i = 0; i < size; i++)
      crc = crc32tab[(crc ^ buf[i]) & 0xff] ^ (crc >> 8);
 
     return crc^0xFFFFFFFF;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  memset(user_rfid_data,0,sizeof(rfid_DATA_T));
  dev_man_init();
  HAL_UART_Receive_IT(&huart2,(uint8_t *)&rfid_tmp_data,1);
  HAL_UART_Receive_IT(&huart1,(uint8_t *)&gprs_tmp_data,1);
  rfid_antenna_num=    0;
 
	
  HAL_UART_Transmit_IT(&huart2,(uint8_t *)cmd_set_work_antenna_1, sizeof(cmd_set_work_antenna_1));
	//HAL_UART_Transmit(&huart2,(uint8_t *)cmd_set_work_antenna_1, sizeof(cmd_set_work_antenna_1),2000);
	
  NDevice data;
	gprs_send_length=sizeof(gprs_start_cmd);
  memcpy(gprs_resendbuff,gprs_start_cmd,sizeof(gprs_start_cmd));
  //HAL_UART_Transmit(&huart1,(uint8_t *)gprs_start_cmd, sizeof(gprs_start_cmd),2000);
	if(debug)
	{
		printf("### start\r\n");
		//HAL_UART_Transmit_IT(&huart1,"0x01\r\n", 6);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
       
    	HAL_Delay(30);
		if(0==gprs_work_flag)
		{
			if(0==(gprs_work_num%120))
			{
				HAL_UART_Transmit(&huart1,(uint8_t *)gprs_start_cmd, sizeof(gprs_start_cmd),2000);
			}
			gprs_work_num++;
		}
  	memset(gprs_pack,0,sizeof(gprs_pack));
	if( gprs_Checkonepack(gprs_pack)>0)
	{
		if(debug)
		{
			printf("###rec data\r\n");
		}
		switch(gprs_pack[1])
		{
			case 0x01:
				if(debug)
				{
					printf("### data is 0x01\r\n");
				}
				if(0==gprs_work_flag)
				{
					gprs_work_flag=1;
				}	
				user_rfid_data[gprs_pack[2]].user_crc1=(uint32_t)(*(uint32_t*)(gprs_pack+3));
				user_rfid_data[gprs_pack[2]].user_crc2=(uint32_t)(*(uint32_t*)(gprs_pack+7));
				if(debug)
				{
					printf("###index is [%d]\r\n",gprs_pack[2]);
					printf("###rec data1 is [%x] 2 is [%x]\r\n",user_rfid_data[gprs_pack[2]].user_crc1,user_rfid_data[gprs_pack[2]].user_crc2);
				}
				//HAL_UART_Transmit_IT(&huart1,gprs_ack_ok,sizeof(gprs_ack_ok) );	
				HAL_UART_Transmit(&huart1,gprs_ack_ok,sizeof(gprs_ack_ok),2000 );
			break;
			case 0x02:
					if(debug)
	{
		printf("### start\r\n");
		//HAL_UART_Transmit_IT(&huart1,"0x01\r\n", 6);
	}
				if(gprs_pack[2]==0xF0)
				{
					HAL_UART_Transmit_IT(&huart1,(uint8_t *)gprs_resendbuff, gprs_send_length);
				}
				
			break;
		}	
	}


	memset(rfid_pack,0,sizeof(rfid_pack));
	if( rfid_Checkonepack(rfid_pack)>0)
	{
		switch(rfid_pack[3])
		{
			case 0x74:
			if(rfid_pack[4]==0x10)
			{
				if(rfid_antenna_num)
				{
					 HAL_UART_Transmit_IT(&huart2,(uint8_t *)cmd_real_time_inventory_2, sizeof(cmd_real_time_inventory_2));

				}
				else
				{
					
					 HAL_UART_Transmit_IT(&huart2,(uint8_t *)cmd_real_time_inventory_1, sizeof(cmd_real_time_inventory_1));
				}
			}
			else
			{
				if(rfid_antenna_num)
				{
					 HAL_UART_Transmit_IT(&huart2,(uint8_t *)cmd_real_time_inventory_2, sizeof(cmd_real_time_inventory_2));
				}
				else
				{
					
				
						 HAL_UART_Transmit_IT(&huart2,(uint8_t *)cmd_real_time_inventory_1, sizeof(cmd_real_time_inventory_1));
				}				
			}
			break;
			case 0x89:
			if(rfid_pack[1]==0x0A)
			{
				
				if(rfid_antenna_num)
				{
					rfid_antenna_num=0;
					 HAL_UART_Transmit_IT(&huart2,(uint8_t *)cmd_set_work_antenna_1, sizeof(cmd_set_work_antenna_1));
				}
				else
				{
					rfid_antenna_num=1;
					 HAL_UART_Transmit_IT(&huart2,(uint8_t *)cmd_set_work_antenna_2, sizeof(cmd_set_work_antenna_1));
				}
				
			}
			else if(rfid_pack[1]==0x04)
			{
				if(rfid_antenna_num)
				{
					rfid_antenna_num=0;
					 HAL_UART_Transmit_IT(&huart2,(uint8_t *)cmd_set_work_antenna_1, sizeof(cmd_set_work_antenna_1));
				}
				else
				{
					rfid_antenna_num=1;
					 HAL_UART_Transmit_IT(&huart2,(uint8_t *)cmd_set_work_antenna_2, sizeof(cmd_set_work_antenna_1));
				}
			}
			else
			{
				
				if(rfid_antenna_num)
				{
						rfid_antenna_num_tmp=0;
				}
				else
				{
					rfid_antenna_num_tmp=1;
				}
				uint32_t crc_data_tmp;
				uint8_t crc_data_array[1];
				uint8_t user_index;
				crc_data_tmp=crc32(rfid_pack+7, 12);
				if(debug)
				{
				printf("###crc_data_tmp is [%x] \r\n",crc_data_tmp);
				}
				user_index=find_crc_index(crc_data_tmp);
				if(debug)
				{
					printf("###user_index is [%d]\r\n",user_index);
				}
				
				if(0xff!=user_index)
				{
					crc_data_array[0]=user_index;
					if(0==dev_man_get_device_by_id(rfid_antenna_num,( uint8_t *)crc_data_array,&data))
					{
						if(0==dev_man_get_device_by_id(rfid_antenna_num_tmp,( uint8_t *)crc_data_array,&data))
						{
						dev_man_del_by_id(rfid_antenna_num,( uint8_t *)crc_data_array);
						dev_man_del_by_id(rfid_antenna_num_tmp,( uint8_t *)crc_data_array);
						//send gprs
						SendPack(user_index,rfid_antenna_num);
						if(debug)
						{
							printf("### SendPack\r\n");
						}

						}
					}
					else
					{
						if(debug)
						{
							printf("#####add\r\n");
						}
					memcpy(data.id,( uint8_t *)crc_data_array,1);
					dev_man_add(rfid_antenna_num, data);


					}
				}
				
			}
			break;
		}
	}
	else
	{
		
	}
 }

  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 
PUTCHAR_PROTOTYPE
{
	 HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 2000);
	return ch;
}

void SendPack(uint8_t index,int INout)
{

	if(INout)
	{
		gprs_data.In=OUT;
	}
	else
	{
		gprs_data.In=IN;
	}
	gprs_data.Index=index;
	//crc
	gprs_data.crc[0]=0;
	gprs_data.crc[1]=0;
	//data_buf
	gprs_send_length=sizeof(GPRS_DATA_T);
	memcpy(gprs_sendbuff,&gprs_data,gprs_send_length);
	memcpy(gprs_resendbuff,gprs_sendbuff,gprs_send_length);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)gprs_sendbuff, gprs_send_length);
}
uint8_t find_crc_index(uint32_t crc_data)
{
	uint8_t i;
	for(i=0;i<user_rfid_all;i++)
	{
		if(debug)
		{
			printf("find_crc_index\r\n");
			printf("user_crc1 is [%x]\r\n",user_rfid_data[i].user_crc1);
			printf("user_crc2 is [%x]\r\n",user_rfid_data[i].user_crc2);
			printf("crc_data is [%x]\r\n",crc_data);
		}
		
		if((user_rfid_data[i].user_crc1==crc_data)||(user_rfid_data[i].user_crc2==crc_data))
		{
			return i;
		}
	}
	return 0xff;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart1)
	{	
		Checkgprsdata(gprs_tmp_data);
		HAL_UART_Receive_IT(&huart1,(uint8_t *)&gprs_tmp_data,1);
		//gprs_recv_flag=1;
	}
	else if(huart==&huart2)
	{
		
		{
			CheckRfiddata(rfid_tmp_data);
			HAL_UART_Receive_IT(&huart2,(uint8_t *)&rfid_tmp_data,1);
		}
	}
  /* Prevent unused argument(s) compilation warning */

  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
}

 uint8_t  CheckSum( uint8_t *uBuff,  uint8_t uBuffLen)
{
	uint8_t i,uSum=0;
	for(i=0;i<uBuffLen;i++)
	{
		uSum = uSum + uBuff[i];
	}
	uSum = (~uSum) + 1;
	return uSum;
}
void CheckRfiddata(uint8_t data)
{
	rfid_recvbuff[rfid_recvend]=data;
	if(rfid_recvend==(rfid_recvbuff_len-1))
	{
		rfid_recvend=0;
	}
	else
	{
		rfid_recvend++;
	}
}
void Checkgprsdata(uint8_t data)
{
	gprs_recvbuff[gprs_recvend]=data;
	if(gprs_recvend==(gprs_recvbuff_len-1))
	{
		gprs_recvend=0;
	}
	else
	{
		gprs_recvend++;
	}
}
int rfid_Checkonepack(uint8_t *pack)
{
	int length=0;
	if(rfid_recvstart!=rfid_recvend)
	{
		while((rfid_recvbuff[rfid_recvstart]!=0xA0)&&(rfid_recvstart!=rfid_recvend))
		{
			if(rfid_recvstart==rfid_recvbuff_len)
			{
				rfid_recvstart=0;
			}
			else
			{
				rfid_recvstart++;
			}
		}
		if(rfid_recvend>rfid_recvstart)
		{
			
			length=rfid_recvend-rfid_recvstart;
			if(length>2)
			{
				if(length>=rfid_recvbuff[rfid_recvstart+1]+2)
				{
					memcpy(pack,&rfid_recvbuff[rfid_recvstart],rfid_recvbuff[rfid_recvstart+1]+2);
					rfid_recvstart=rfid_recvstart+rfid_recvbuff[rfid_recvstart+1]+2;
				}
				else goto ret;
				
			}
			else goto ret;
		}
		else if(rfid_recvstart>rfid_recvend)
		{
			length=rfid_recvbuff_len-rfid_recvstart+rfid_recvend+1;
			if(length>2)
			{
				if(rfid_recvstart==rfid_recvbuff_len-1)
				{
					if(length>=rfid_recvbuff[0]+2)
					{
						memcpy(pack,&rfid_recvbuff[rfid_recvstart],1);
						memcpy(pack+1,&rfid_recvbuff[0],rfid_recvbuff[0]+1);
						rfid_recvstart=rfid_recvbuff[0]+1;
					}
					else goto ret;
				}
				else
				{
					if(length>=rfid_recvbuff[rfid_recvstart+1]+2)
					{
						if((rfid_recvbuff_len-rfid_recvstart)<rfid_recvbuff[rfid_recvstart+1]+2)
						{

							memcpy(pack,&rfid_recvbuff[rfid_recvstart],rfid_recvbuff_len-rfid_recvstart);
							memcpy(pack+(rfid_recvbuff_len-rfid_recvstart),&rfid_recvbuff[0],rfid_recvbuff[rfid_recvstart+1]+2-(rfid_recvbuff_len-rfid_recvstart));

							rfid_recvstart=rfid_recvbuff[rfid_recvstart+1]+2-(rfid_recvbuff_len-rfid_recvstart);
						}
						else if((rfid_recvbuff_len-rfid_recvstart)>=rfid_recvbuff[rfid_recvstart+1]+2)
						{
							memcpy(pack,&rfid_recvbuff[rfid_recvstart],rfid_recvbuff[rfid_recvstart+1]+2);
							rfid_recvstart=rfid_recvstart+rfid_recvbuff[rfid_recvstart+1]+2;
						}
					}
					else goto ret;
				}
			}
			else goto ret;
		}
		

			
		if(CheckSum(( uint8_t *)pack,pack[1]+1)==pack[pack[1]+1])
		{
			return pack[1]+2;
		}		
		
		
	}
	
ret:	
	return 0;
	
}

int gprs_Checkonepack(uint8_t *pack)
{
	int length=0;
	if(gprs_recvstart!=gprs_recvend)
	{
		while((gprs_recvbuff[gprs_recvstart]!='{')&&(gprs_recvstart!=gprs_recvend))
		{
			if(gprs_recvstart==gprs_recvbuff_len)
			{
				gprs_recvstart=0;
			}
			else
			{
				gprs_recvstart++;
			}
		}
		if(gprs_recvend>gprs_recvstart)
		{
			
			length=gprs_recvend-gprs_recvstart;
			if(length>2)
			{
				switch(gprs_recvbuff[gprs_recvstart+1])
				{
					case 0x01:
						if(length>=14)
						{
						memcpy(pack,&gprs_recvbuff[gprs_recvstart],14);
						gprs_recvstart=gprs_recvstart+14;
						}
						else goto ret;
					break;
					case 0x02:
						if(length>=6)
						{
						memcpy(pack,&gprs_recvbuff[gprs_recvstart],6);
						gprs_recvstart=gprs_recvstart+6;
						}
						else goto ret;
					break;
				}

				
			}
			else goto ret;
		}
		else if(gprs_recvstart>gprs_recvend)
		{
			length=gprs_recvbuff_len-gprs_recvstart+gprs_recvend+1;
			if(length>2)
			{
				if(gprs_recvstart==gprs_recvbuff_len-1)
				{
					switch(gprs_recvbuff[0])
					{
						case 0x01:
							if(length>=14)
							{
							memcpy(pack,&gprs_recvbuff[gprs_recvstart],1);
							memcpy(pack+1,&gprs_recvbuff[0],13);
							gprs_recvstart=13;
							}
							else goto ret;
						break;
						case 0x02:
							if(length>=6)
							{
							memcpy(pack,&gprs_recvbuff[gprs_recvstart],1);
							memcpy(pack+1,&gprs_recvbuff[0],5);
							gprs_recvstart=5;
							}
							else goto ret;
						break;
					}
				}
				else
				{
					
					switch(gprs_recvbuff[gprs_recvstart+1])
					{
						case 0x01:
							if(length>=14)
							{
								if((gprs_recvbuff_len-gprs_recvstart)<14)
								{

									memcpy(pack,&gprs_recvbuff[gprs_recvstart],gprs_recvbuff_len-gprs_recvstart);
									memcpy(pack+(gprs_recvbuff_len-gprs_recvstart),&rfid_recvbuff[0],14-(gprs_recvbuff_len-gprs_recvstart));

									gprs_recvstart=14-(gprs_recvbuff_len-gprs_recvstart);
								}
								else if((gprs_recvbuff_len-gprs_recvstart)>=14)
								{
									memcpy(pack,&gprs_recvbuff[gprs_recvstart],14);
									gprs_recvstart=gprs_recvstart+14;
								}
							}
							else goto ret;
						break;
						case 0x02:
							if(length>=6)
							{
								if((gprs_recvbuff_len-gprs_recvstart)<6)
								{

									memcpy(pack,&gprs_recvbuff[gprs_recvstart],gprs_recvbuff_len-gprs_recvstart);
									memcpy(pack+(gprs_recvbuff_len-gprs_recvstart),&gprs_recvbuff[0],6-(gprs_recvbuff_len-gprs_recvstart));

									gprs_recvstart=6-(gprs_recvbuff_len-gprs_recvstart);
								}
								else if((gprs_recvbuff_len-gprs_recvstart)>=6)
								{
									memcpy(pack,&gprs_recvbuff[gprs_recvstart],6);
									gprs_recvstart=gprs_recvstart+6;
								}
							}
							else goto ret;
						break;
					}
				}
			}
			else goto ret;
		}

			
		if(pack[1]==0x01)
		{
			return 14;
		}
		else
		{
			return 6;
		}
		
		
	}
	
ret:	
	return 0;
	
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
