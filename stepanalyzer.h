#ifndef stepanalyzer_h
#define stepanalyzer_h

enum Lab3ASignals {
	CALIBRATE = Q_USER_SIG,
	WAYPOINT,
	TICK,
	TOUR
};


extern struct Lab3ATag AO_Lab3A;

void Lab3A_ctor(void);
void GpioHandler(void *CallbackRef);
void TwistHandler(void *CallbackRef);
void dispatch(QSignal sig);

extern float xval;
extern float yval;
extern float zval;
extern int direction;

#endif
