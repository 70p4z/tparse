/*******************************************************************************
*   TPARSE Demonstration tool: iobridge
*   (c) 2022 Olivier TOMAZ
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
********************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tparse.h"
#include "stddef.h"

/* Private functions ---------------------------------------------------------*/

uint8_t tmp[TMP_BUFFER_SIZE_B];

uint32_t tparse_al_time(void) {
  return uwTick; /* todo bind the systick count here */;
}

extern uint8_t n2h(uint8_t c);


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 80000000
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{

}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  while (1)
  {
    interp();
  }
}
