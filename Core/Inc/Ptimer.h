#ifndef PTIMER_H_
#define PTIMER_H_

/* Includes -----------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include <stdint.h>

/* Type Defines -------------------------------------------------------*/

// Program timer structure
typedef struct
{
  uint8_t Number;		// Timer number
  uint32_t Time;		// Time to count
} Ptimer_t;

// The maximum number of timers that could be simultaneously present in program
#define MAX_NUMBER_OF_TIMERS 8

// Program timer flags type
typedef struct
{
  unsigned timer         :MAX_NUMBER_OF_TIMERS;
} ptimerFlags_t;
/* Defines ------------------------------------------------------------*/


/* Share variables  ---------------------------------------------------*/
extern __IO Ptimer_t Ptimer[MAX_NUMBER_OF_TIMERS];
extern __IO ptimerFlags_t ptimerFlag;
/* Function prototypes  -----------------------------------------------*/
void SetPTimer(uint8_t NewNumber, uint32_t NewTime);
void PTimer(void);
void InitPTimer(void);
void KillPTimer(uint8_t timerNumber);

#endif /* PTIMER_H_ */
