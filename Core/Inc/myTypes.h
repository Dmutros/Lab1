#ifndef MYTYPES_H_
#define MYTYPES_H_

typedef enum
{
	notpressed,
	pressed,
} buttonStates_t;

typedef enum
{
	wait,
	antibounce
} buttonAntibounceStates_t;

typedef struct
{
	unsigned previous_state 			 :1;
	unsigned current_state  			 :1;
} buttonPinState_t;

typedef struct
{
	unsigned button_action  		 	 :1;
} globalFlags_t;

typedef struct
{
	unsigned rising_edge  		 	 	 :1;
	unsigned falling_edge      		 :1;
} buttonAntibounceFlags_t;

typedef struct
{
	unsigned pressed  		 	 			 :1;
	unsigned released        			 :1;
} buttonFlags_t;


#endif /* MYTYPES_H_ */
