typedef void (*TPTR)(void);
typedef struct {
	TPTR GoToTask;
	uint32_t Time;
} Stimer;

typedef enum {
	NEW,
	UPDATE
} actionType;

void SetTimerTask(TPTR TS, uint32_t NewTime, actionType action) {
	uint8_t index = 0;
	uint8_t interrupts_enable = ~_get_PRIMASK();

	if (interrupts_enable)
	{
		_disable_irq();
	}

	if (action == UPDATE) {
		// Оновлення таймера для існуючої задачі
		for (index = 0; index < MainTimerQueueSize; ++index) {
			if (MainTimer[index].GoToTask == TS)
			{
				MainTimer[index].Time = NewTime;

				if (interrupts_enable)_enable_irq();

				return;
			}
		}
	} else if (action == NEW) {
		// Створення нового таймера для задачі
		for (index = 0; index < MainTimerQueueSize; ++index) {
			if (MainTimer[index].GoToTask == Idle)
			{
				MainTimer[index].GoToTask = TS;

				MainTimer[index].Time = NewTime;

				if (interrupts_enable)_enable_irq();

				return;
			}
		}
	}

	if (interrupts_enable)_enable_irq();
}
