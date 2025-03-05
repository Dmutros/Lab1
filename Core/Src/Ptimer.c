#include "Ptimer.h"

__IO Ptimer_t Ptimer[MAX_NUMBER_OF_TIMERS] = {{255,0}};	// Global array of program timers
__IO ptimerFlags_t ptimerFlag 						 = {0};				// Global program timers flags

/**************************************************************************************************
 * This function setup the program timer (if there is free space in program timers array) or update
 * the timeout if the timer with the same number already exists.
 * Input parameters:
 * 			 NewNumber - the number of a new timer (should be less then MAX_NUMBER_OF_TIMERS), or the
 * 			 number of timer that you want to update.
 * 			 NewTime - the new time delay in case of existing timer or time delay for the new timer in
 * 			 case if timer dosen't exist yet.
 */
void SetPTimer(uint8_t NewNumber, uint32_t NewTime)
  {
		// The number of a free timer in array (255 - means that no free timers available)
    uint8_t free_timer        = 255;
    uint8_t interrupts_enable = ~__get_PRIMASK(); 			// Get the current state of interrupts

    if (interrupts_enable) __disable_irq();       			// Disable interrupts if they were enabled (atomic access)
    // Go thru timers queue. Looking for existing or empty timer
    for (uint8_t i = 0; i != MAX_NUMBER_OF_TIMERS; i++)
      {
        if (Ptimer[i].Number == NewNumber)              // If there is the timer with the same "Number"
          {
            Ptimer[i].Time = NewTime;                   // Give it a new "Time"
            free_timer = 255;                           // Mark that no free timers available and break the cycle
            break;
          }
        if (Ptimer[i].Number == 255)                    // If there is a free timer, save it number
          {
            free_timer = i;
          }
      }
    if(free_timer < MAX_NUMBER_OF_TIMERS)               // If there is a free timer in array
      {
        Ptimer[free_timer].Number = NewNumber;          // Give it a "Number"
        Ptimer[free_timer].Time = NewTime;              // And a "Time"
      }

    if (interrupts_enable) __enable_irq(); // Enable interrupts if they were disabled (the end of atomic access)
  }

/************************************************************************************************
 * This function realizes the program timer functionality, it should be called from the interrupt
 * service routing of the hardware timer that provides time base for the program timer
 */
void PTimer(void)
{
	for (uint8_t i = 0; i != MAX_NUMBER_OF_TIMERS; i++) 		// Go thru timers queue.
	    {
	      if (Ptimer[i].Number != 255)                  		// If timer exists and its time delay dosen't
	        {
	          if (Ptimer[i].Time != 0)                    	// equal to 0,
	            {
	              Ptimer[i].Time--;                       	// decrement the time delay
	            }
	          else
	            {
	          		ptimerFlag.timer |= 1 << Ptimer[i].Number;// If time dalay equal 0 then set the flag
	              Ptimer[i].Number = 255;                   // and remove timer from queue
	            }
	        }
	    }
}

/**************************************************************************************
 * This function initialize program timer queue. It mast be called before program timer
 * can be used.
 */
void InitPTimer(void)
{
	for (uint8_t i = 0; i != MAX_NUMBER_OF_TIMERS; i++) // Go thru timers queue.
		{
			Ptimer[i].Number = 255;													// Set timer number to "not used" (the number 255)
			Ptimer[i].Time = 0;															// Set time delay to 0
		}
}

/*****************************************************************************
 * This function kills the timer with given number if it exists in timer queue
 * Input parameters:
 * 			 timerNumber - the number of a timer to kill.
 */
void KillPTimer(uint8_t timerNumber)
{
	uint8_t interrupts_enable = ~__get_PRIMASK (); // Get the current state of interrupts
	if (interrupts_enable) __disable_irq ();       // Disable interrupts if they were enabled (atomic access)
	// Go thru timers queue. Looking for the timer with timerNumber
	for (uint8_t i = 0; i != MAX_NUMBER_OF_TIMERS; i++)
		{
			if (Ptimer[i].Number == timerNumber)       // If there is the timer with the same "Number"
				{
					Ptimer[i].Time = 0;                    // Clear "Time"
					Ptimer[i].Number = 255;          			 // Mark timer queue entry as free and break the cycle
					break;
				}
		}
	if (interrupts_enable) __enable_irq();  // Enable interrupts if they were disabled (the end of atomic access)
}
