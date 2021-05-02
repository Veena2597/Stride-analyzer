#ifndef bsp_h
#define bsp_h

void BSP_init(void);
void ISR_gpio(void);
void ISR_timer(void);

#define BSP_showState(prio_, state_) ((void)0)

#define ENCODER_INTERRUPT XGPIO_IR_MASK
#define BUTTON_INTERRUPT XGPIO_IR_MASK
#define NAV_INTERRUPT XGPIO_IR_MASK

extern int timerTrigger;
extern int stage;

#endif                                                             


