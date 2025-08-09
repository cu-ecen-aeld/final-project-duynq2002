/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#define STM32F401xE
#include "stdbool.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "KEYPAD.h"
#include "rc522.h"

/* USER CODE END Includes */
/* Private define ------------------------------------------------------------*/
#define LCD_PORT	GPIOB
#define LCD_DATA	GPIOA
#define LCD_ENA	GPIO_PIN_15
#define LCD_RST	GPIO_PIN_14
#define LCD_D4	GPIO_PIN_8
#define LCD_D5	GPIO_PIN_9
#define LCD_D6	GPIO_PIN_10
#define LCD_D7	GPIO_PIN_11
#define MAXLENGTH 6
/* USER CODE BEGIN PD */
#define FLASH_USER_START_ADDR 0x08012000
/* USER CODE END PD */

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0    ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1    ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2    ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3    ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4    ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5    ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6    ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7    ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
char Key;
char DATA[MAXLENGTH+1];
char PASSWORD[6]={'0','0','0','0','0','0'};
int length = 0;
bool state = false;
int PassErr = 0;
uint32_t cardPASS[6] = {1,2,3,4,5,6};
uint32_t cardRx[6];
/* USER CODE END PTD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
void LCD_Enable()
{
    HAL_GPIO_WritePin(LCD_PORT,LCD_ENA,1);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_PORT,LCD_ENA,0);	
    HAL_Delay(1);	
}

void LCD_Send4Bit(unsigned char Data)
{
    HAL_GPIO_WritePin(LCD_DATA,LCD_D4,Data&0x01);
    HAL_GPIO_WritePin(LCD_DATA,LCD_D5,(Data>>1)&0x01);
    HAL_GPIO_WritePin(LCD_DATA,LCD_D6,(Data>>2)&0x01);
    HAL_GPIO_WritePin(LCD_DATA,LCD_D7,(Data>>3)&0x01);
}

void LCD_SendCommand(unsigned char command)
{
    LCD_Send4Bit(command >>4);
    LCD_Enable();
    LCD_Send4Bit(command);
    LCD_Enable();
}

void LCD_Clear()
{
    LCD_SendCommand(0x01);
    HAL_Delay(1);
}

void LCD_Init()
{
    LCD_Send4Bit(0x00);
    HAL_GPIO_WritePin(LCD_PORT,LCD_RST,0);
    LCD_Send4Bit(0x03);
    LCD_Enable();
    LCD_Enable();
    LCD_Enable();
    LCD_Send4Bit(0x02);
    LCD_Enable();
    LCD_SendCommand(0x28);
    LCD_SendCommand(0x0C);
    LCD_SendCommand(0x06);
    LCD_SendCommand(0x01);
}

void LCD_Gotoxy(unsigned char x, unsigned char y)
{
unsigned char address;
if(y==0)
address=0x80;
else if(y==1)
{
address=0xc0;
}
else if(y==2)
{
address=0x94;
}
else
address=0xd4;
address+=x;
LCD_SendCommand(address);
}

void LCD_PutChar(unsigned char Data)
{
  HAL_GPIO_WritePin(LCD_PORT,LCD_RST,1);
     LCD_SendCommand(Data);
  HAL_GPIO_WritePin(LCD_PORT,LCD_RST,0);
}

void LCD_Puts(char *s)
{
       while (*s){
          LCD_PutChar(*s);
         s++;
       }
}	

KEYPAD_Name KeyPad;
char KEYMAP[NUMROWS][NUMCOLS]={
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'},
};

uint16_t CheckPassWord (char DATA[MAXLENGTH],char PASS[MAXLENGTH]){
    if (length!=6) return 0;
    for (int i=0;i<MAXLENGTH;i++){
        if (DATA[i] != PASS[i]) return 0;
    }
    return 1;
}

uint16_t ClearPassWord (){
    for (int i=0;i<MAXLENGTH;i++){
        DATA[i] = -1;
    }
    length = 0;
}
uint8_t  str[5];
int card[100]={536874093,536874735};
int cardNum;


int converCardNum(uint8_t* CardID){
    int cardNum;
    for (int i = 0; i < 5; i++) {
            cardNum+=CardID[i]<<i;
    }
    return cardNum;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


 
/**
 * @brief Gets the sector of a given address
 * @param None
 * @retval The sector of a given address
 */
uint32_t GetSector(uint32_t Address)
{
 uint32_t sector = 0;
 
 if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
 {
   sector = FLASH_SECTOR_0; 
 }
 else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
 {
   sector = FLASH_SECTOR_1; 
 }
 else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
 {
   sector = FLASH_SECTOR_2; 
 }
 else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
 {
   sector = FLASH_SECTOR_3; 
 }
 else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
 {
   sector = FLASH_SECTOR_4; 
 }
 else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
 {
   sector = FLASH_SECTOR_5; 
 }
 else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
 {
   sector = FLASH_SECTOR_6; 
 }
 else if((Address < 0x0807FFFF) && (Address >= ADDR_FLASH_SECTOR_7))
 {
   sector = FLASH_SECTOR_7; 
 }
 else
 {
    sector = HAL_FLASH_GetError();
 }
 return sector;
}

uint32_t Flash_Write_Data (uint32_t FlashUserStartAdd, uint32_t *Data, uint16_t numberofwords)
{
    uint32_t retval = 0;
    static FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError;
    uint32_t FlashUserEndAdd;
    int sofar=0;
    uint32_t FirstSector;
    uint32_t NbOfSector;

    FlashUserEndAdd = FlashUserStartAdd + numberofwords*4;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    /* Xóa vùng Flash người sử dụng
    (xác định bởi FlashUserStartAdd và FLASH_USER_END_ADDR) ***********/
    /* Nhận vùng đầu tiên để xóa */
    FirstSector = GetSector(FlashUserStartAdd);
    /* Lấy số lượng sector để xóa từ sector 1 vừa nhận*/
    NbOfSector = GetSector(FlashUserEndAdd) - FirstSector + 1;
    /* Cấu trúc khởi tạo Xóa*/
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    /* với hiệu điện thế nằm giữa 2,7V và 3,6V ta truyền vào VoltageRange_3*/
    EraseInitStruct.Sector = FirstSector;
    EraseInitStruct.NbSectors = NbOfSector;
    if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
    { 
        /* Xảy ra lỗi tròn khi xóa Sector sẽ cần thêm một số code để xử lý lỗi này
        SectorError sẽ chứa sector bị lỗi,và sau đó để biết mã lỗi trên sector này
        bạn cần gọi hàm 'HAL_FLASH_GetError()'*/
        retval = HAL_FLASH_GetError();
    }

    /* Program the user Flash area word by word*/

    while (sofar<numberofwords)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FlashUserStartAdd, Data[sofar]) == HAL_OK)
        {
            FlashUserStartAdd += 4;  // use FlashUserStartAdd += 2 for half word and 8 for double word
            sofar++;
        }
        else
        {
        /* Error occurred while writing data in Flash memory*/
            return HAL_FLASH_GetError();
        }
    }

    /* Lock the Flash to disable the flash control register access (recommended
        to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();

    return 0;
}


void Flash_Read_Data (uint32_t FlashUserStartAdd, uint32_t *RxBuf, uint16_t numberofwords)
{
    while (1)
    {
        *RxBuf = *(__IO uint32_t *)FlashUserStartAdd;
        FlashUserStartAdd += 4;
        RxBuf++;
        if (!(numberofwords--)) break;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    KEYPAD4X4_Init(&KeyPad,KEYMAP,GPIOB,GPIO_PIN_5,GPIOB,GPIO_PIN_4,GPIOB,GPIO_PIN_3,GPIOA,GPIO_PIN_15
                                                                ,GPIOB,GPIO_PIN_9,GPIOB,GPIO_PIN_8,GPIOB,GPIO_PIN_7,GPIOB,GPIO_PIN_6);	
    TM_MFRC522_Init();
    LCD_Init();
    LCD_Clear();
    LCD_Gotoxy(0,0);
    LCD_Puts("Trang thai");
    LCD_Gotoxy(0,1);
    LCD_Puts("Dong Cua");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
        Key = KEYPAD4X4_Readkey(&KeyPad);
        if (Key){
                char PASS[MAXLENGTH];
                LCD_Clear();
                LCD_Gotoxy(0,0);
                LCD_Puts("Nhap mat khau ");
                switch(Key){
                    case 'A':
                    {
                        Flash_Write_Data(FLASH_USER_START_ADDR,(uint32_t *)cardPASS,6);
                        Flash_Read_Data(FLASH_USER_START_ADDR,(uint32_t *)cardRx,6);
                        break;
                    }
                    case 'B':{
                        if (length<=0)break;
                        LCD_Clear();
                        if (PassErr>=3){
                            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,1);
                            LCD_Gotoxy(0,0);
                            LCD_Puts("Canh bao");
                            HAL_Delay(5000);
                            ClearPassWord();
                            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
                            LCD_Gotoxy(0,0);
                            LCD_Puts("Trang thai");
                            LCD_Gotoxy(0,1);
                            LCD_Puts("Dong cua");
                            PassErr=0;
                            break;
                        }
                        if (CheckPassWord(DATA,PASSWORD)){
                            PassErr = 0;
                            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
                            LCD_Gotoxy(0,0);
                            LCD_Puts("Trang thai: ");
                            LCD_Gotoxy(0,1);
                            LCD_Puts("Mo cua");
                            HAL_Delay(5000);
                            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
                            LCD_Clear();
                            LCD_Gotoxy(0,0);
                            LCD_Puts("Trang thai: ");
                            LCD_Gotoxy(0,1);
                            LCD_Puts("Dong cua");
                            ClearPassWord();
                        }
                        else{
                            PassErr = PassErr+1;
                            LCD_Gotoxy(0,0);
                            LCD_Puts("Nhap sai mat khau");
                            HAL_Delay(2000);
                            LCD_Clear();
                            ClearPassWord();
                            LCD_Gotoxy(0,0);
                            LCD_Puts("Trang thai");
                            LCD_Gotoxy(0,1);
                            LCD_Puts("Dong cua");
                            length = 0;
                        }
                        break;
                    }
                    case 'D':{
                        if (length>0) {
                            DATA[length-1] = NULL;
                            length=length-1;
                            LCD_Clear();
                            LCD_Gotoxy(0,0);
                            LCD_Puts("Nhap mat khau");
                            LCD_Gotoxy(0,1);
                            sprintf(PASS,"%s",DATA);
                            LCD_Puts(DATA);
                        }
                        else{
                            LCD_Clear();
                            ClearPassWord();
                            LCD_Gotoxy(0,0);
                            LCD_Puts("Trang thai");
                            LCD_Gotoxy(0,1);
                            LCD_Puts("Dong cua");
                        }
                        break;
                    }
                    case 'C':{
                        if (length<=0)break;
                        LCD_Clear();
                        ClearPassWord();
                        LCD_Gotoxy(0,0);
                        LCD_Puts("Trang thai");
                        LCD_Gotoxy(0,1);
                        LCD_Puts("Dong cua");
                        length = 0;
                        break;
                    }
                    default:{
                        if (length>=MAXLENGTH){
                            LCD_Gotoxy(0,0);
                            LCD_Puts("Nhap sai mat khau");
                            HAL_Delay(2000);
                            LCD_Clear();
                            ClearPassWord();
                            LCD_Gotoxy(0,0);
                            LCD_Puts("Trang thai");
                            LCD_Gotoxy(0,1);
                            LCD_Puts("Dong cua");
                            length = 0;
                            break;
                        }
                        DATA[length] = Key;
                        length = length+1;	
                        LCD_Gotoxy(0,1);
                        sprintf(PASS,"%s",DATA);
                        LCD_Puts(DATA);
                        break;
                    } 
                }
        }
        if(length==0 && !TM_MFRC522_Request(PICC_REQIDL,str)){
            if(!TM_MFRC522_Anticoll(str)){
                char code[100];
                cardNum = converCardNum(str);
                LCD_Clear();
                LCD_Gotoxy(0,0);
                LCD_Puts("Dang doc the");
            //	LCD_Gotoxy(0,1);
                //LCD_Puts("Dang doc the");
            //	sprintf(code,"%d",cardNum);
            //	LCD_Puts(code);
            //	HAL_Delay(50);
                for(int i=0;i<100;i++){
                    if (cardNum==card[i]){
                        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
                        LCD_Clear();
                        LCD_Gotoxy(0,0);
                        LCD_Puts("Trang thai");
                        LCD_Gotoxy(0,1);
                        LCD_Puts("Mo cua");	
                        HAL_Delay(5000);
                        cardNum=0;
                        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
                        break;
                    }
                }
                HAL_Delay(1000);
                LCD_Clear();
                LCD_Gotoxy(0,0);
                LCD_Puts("Trang thai");
                LCD_Gotoxy(0,1);
                LCD_Puts("Dong Cua");
            }
        }
        HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
