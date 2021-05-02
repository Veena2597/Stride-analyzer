#define AO_LAB3A

#include "qpn_port.h"
#include "bsp.h"
#include "stepanalyzer.h"
#include "PmodNAV.h"
#include "lcd.h"
#include "stdlib.h"

typedef struct Lab3ATag  {               
	QActive super;
}  Lab3A;

static QState Lab3A_initial (Lab3A *me);
static QState Lab3A_on      (Lab3A *me);
static QState Lab3A_Calibrate (Lab3A *me);
static QState Lab3A_Waypoint (Lab3A *me);
static QState Lab3A_Tour(Lab3A *me);

//static void dispatch (QSignal sig);
void stateCall(u32 btn);
void stepCalc(float x, float y, float z);
void xdistCalc(float x, float y);
void zdistCalc(float z);
void printVal(float x, float y, float z);
void Display();
void resetDisplay();

/**********************************************************************/

Lab3A AO_Lab3A;

int calstatus=0;
int waystatus=0;
int tourstatus=0;
int num=0;

int phase = 0;
int stage = -3;
int next = 0;
int count = 0;
int step_count = 0;
int zstep =0;
int waypoint = 0;
int samples = 0;
int tour = 0;
int tstate = 0;
int dir[8];

float acc = 0.0;
float t = 0.1;

float xavg_error = 0.0;
float yavg_error = 0.0;
float zavg_error = 0.0;

float xvel = 0.0;
float xdist = 0.0;
float yvel = 0.0;
float ydist = 0.0;
float zvel = 0.0;
float zdist = 0.0;
float slx = 0.0;
float slz = 0.0;
float xcord = 0.0;
float ycord = 0.0;
float zcord = 0.0;
float xdata[8];
float ydata[8];
float zdata[8];

char temp='0';
char step_print[3];
char tx[4];
char ty[4];
char tz[4];
char wayprint[1];

int step_status = 0;
char *compass[8] = {"North", "North-East", "East", "South-East", "South",
          "South-West", "West", "North-West"};

void Lab3A_ctor(void)  {
	Lab3A *me = &AO_Lab3A;
	QActive_ctor(&me->super, (QStateHandler)&Lab3A_initial);
}


QState Lab3A_initial(Lab3A *me) {
	xil_printf("\n\rInitialization");
	return Q_TRAN(&Lab3A_on);
}

QState Lab3A_on(Lab3A *me) {
	switch (Q_SIG(me)) {
		case Q_ENTRY_SIG: {
			xil_printf("\n\rOn");
			}

		case Q_INIT_SIG: {
			return Q_TRAN(&Lab3A_Calibrate);
			}
	}

	return Q_SUPER(&QHsm_top);
}

QState Lab3A_Calibrate(Lab3A *me) {
	switch (Q_SIG(me)) {
		case Q_ENTRY_SIG: {
			//xil_printf("Startup Calibrate\n");
			return Q_HANDLED();
		}

		case WAYPOINT: {
			if(calstatus==1){
				Display();
				next = 0;
				num=0;
				step_count = 0;
				waypoint=0;
				zstep=0;
				for(int i=0; i<8;i++){
					xdata[i]=0.0;
					ydata[i]=0.0;
					zdata[i]=0.0;
					dir[i]=0;
				}
				xcord=0;
				ycord=0;
				zcord=0;
				direction=0;
				for(int i=0;i<4;i++){
					tx[i]='0';
					ty[i]='0';
					tz[i]='0';
				}

				resetDisplay();
				setColor(0,255,0);
				setFont(BigFont);
				lcdPrint("WAY POINT ", 35, 35);
				lcdPrint("STEPCOUNT:", 30, 135);
				lcdPrint("NEXT:M17", 50, 215);
				lcdPrint("END:P18", 60, 240);

				return Q_TRAN(&Lab3A_Waypoint);
			}
			else if(calstatus==0){
				Display();
				resetDisplay();
				setColor(0,255,0);
				lcdPrint("CALIBRATION", 35, 160);
				lcdPrint("NOT DONE", 50, 185);
				lcdPrint("CALIBRATION:", 35, 215);
				lcdPrint("PRESS M18", 45, 240);
				return Q_HANDLED();
			}
		}

		case TOUR:{
			if(waystatus==1){
				Display();
				resetDisplay();
				tour = 0;
				next=0;
				tstate = 1;
				for(int i=0;i<4;i++){
					tx[i]='0';
					ty[i]='0';
					tz[i]='0';
				}
				return Q_TRAN(&Lab3A_Tour);
			}
			else if(waystatus==0){
				Display();
				resetDisplay();
				setColor(0,255,0);
				lcdPrint("WAYPOINTS", 50, 160);
				lcdPrint("NOT STORED", 40, 185);
				lcdPrint("WAYPOINTS:", 35, 215);
				lcdPrint("PRESS M17", 45, 240);
				return Q_HANDLED();
			}

		}

		case TICK:{
			switch(stage){
			case -3:{
				setFont(BigFont);
				resetDisplay();
				setColor(0,255,0);
				lcdPrint("CALIBRATION:M18", 1, 160);
				lcdPrint("WAYPOINT:M17", 35, 185);
				lcdPrint("TOUR:P17", 70, 210);
				lcdPrint("EXIT:P18", 70, 235);
				stage =-2;
				break;
			}
			case 0:{
				stage = 1;
				next = 0;

				resetDisplay();

				setColor(0,255,0);
				lcdPrint("CALIBRATION", 35, 35);
				lcdPrint("ROTATE TWICE", 30, 160);
				break;
			}
			case 1:{
				if(samples<50){
					caliMag();
					samples += 1;
					//xil_printf("samples");
				}
				else{
					lcdPrint("DONE!", 90, 185);
					lcdPrint("NEXT:", 90, 215);
					lcdPrint("PRESS P18", 45, 240);

				}
				if(next==1){
					stage = 2;
					resetDisplay();
					setColor(0,255,0);
					lcdPrint(" STAND STILL ", 30, 160);
				}
			}
			case 2:{
				if(timerTrigger!=4){
					DataRead();
					xavg_error += xval;
					yavg_error += yval;
					zavg_error += zval;

					count = count + 1;
				}
				break;
			}
			case -1:{
				xavg_error = xavg_error/count;
				yavg_error = yavg_error/count;
				zavg_error = zavg_error/count;

				stage = 3;
				xdist = 0;
				ydist = 0;
				zdist = 0;
				xvel = 0;
				yvel = 0;
				zvel = 0;

				resetDisplay();
				setColor(0,255,0);
				lcdPrint("MOVE 3 STEPS", 25, 135);
				lcdPrint("NORMAL", 75, 160);
				lcdPrint("NEXT:", 90, 215);
				lcdPrint("PRESS P18", 45, 240);
				break;
			}
			case 3:{
				if(next == 1){
					DataRead();
					stepCalc(xval,yval,zval);
					xdistCalc(xval,yval);
				}
				else if(next == 2){
					stage = 4;

					slx = sqrt(xdist*xdist+ydist*ydist)/3;
					//printf("xdist: %f\n",slx);
					if(slx>0.6){
						slx=0.6;
					}
					setColor(0,50,100);
					fillRect(60,160,200,190);
					setColor(0,255,0);
					lcdPrint("FAST", 90, 160);

				}
				break;
			}
			case 4:{
				if(next == 2){
					DataRead();
					stepCalc(xval,yval,zval);
				}
				else if(next == 3){
					stage = 5;
					lcdPrint("UP/DOWN", 60, 160);
				}
				break;
			}
			case 5:{
				if(next == 3){
					DataRead();
					stepCalc(xval,yval,zval);
					zdistCalc(zval);
					//printf("%f ",zval);
				}
				else if(next == 4){
					stage = 6;
					calstatus=1;
					slz = zdist/3;
					if(slz>0.3){
						slz=0.3;
					}
					//printf("zdist: %f ",slz);
					resetDisplay();
					setColor(0,255,0);
					lcdPrint("CALIBRATION", 35, 135);
					lcdPrint("DONE", 90, 165);

					lcdPrint("WAYPOINTS:", 35, 215);
					lcdPrint("PRESS M17", 45, 240);

				}
				break;
			}
			}
			return Q_HANDLED();
		}
	}

	return Q_SUPER(&Lab3A_on);

}

QState Lab3A_Waypoint(Lab3A *me) {
	switch (Q_SIG(me)) {
		case Q_ENTRY_SIG: {
			step_count = 0;
			//xil_printf("Startup State measure\n");
			return Q_HANDLED();
		}

		case TICK: {
			if(next == 0){
				DataRead();
				stepCalc(xval,yval,zval);

				//xil_printf("d:%d ",direction);
				if(step_status == 1){
					getDirection();
					setColor(0,255,0);
					itoa(step_count, step_print, 10);
					lcdPrint(&step_print, 190, 135);
				}
			}
			if(next == 1){
				resetDisplay();
				setColor(0,255,0);
				lcdPrint("THANK YOU", 45, 188);
				next = 7;
			}
			return Q_HANDLED();
		}

		case CALIBRATE: {
			Display();
			stage = 0;
			return Q_TRAN(&Lab3A_Calibrate);
		}

		case TOUR:{
			//Display();
			//resetDisplay();
			tour = 0;
			next=0;
			tstate = 1;
			for(int i=0;i<4;i++){
				tx[i]='0';
				ty[i]='0';
				tz[i]='0';
			}

			return Q_TRAN(&Lab3A_Tour);
		}

		case WAYPOINT: {
			if(waypoint==1){
				resetDisplay();
				setColor(0,255,0);
				lcdPrint("STEPCOUNT:", 30, 135);
				lcdPrint("X:", 60, 165);
				lcdPrint("Y:", 60, 190);
				lcdPrint("Z:", 60, 215);
			}

			switch(direction){
			case 0: {
				ycord += step_count*slx;
				break;
			}
			case 1: {
				xcord += 0.707*step_count*slx;
				ycord += 0.707*step_count*slx;
				break;
			}
			case 2: {
				xcord += step_count*slx;
				break;
			}
			case 3: {
				xcord += 0.707*step_count*slx;
				ycord -= 0.707*step_count*slx;
				break;
			}
			case 4: {
				ycord -= step_count*slx;
				break;
			}
			case 5: {
				xcord -= 0.707*step_count*slx;
				ycord -= 0.707*step_count*slx;
				break;
			}
			case 6: {
				xcord -= step_count*slx;
				break;
			}
			case 7: {
				xcord -= 0.707*step_count*slx;
				ycord += 0.707*step_count*slx;
				break;
			}
			}
			zcord += zstep*slz;

			//printf("x: %f, y: %f \n",xcord,ycord);
			gcvt(xcord,4,tx);
			lcdPrint(&tx,87,165);
			gcvt(ycord,4,ty);
			lcdPrint(&ty,87,190);
			gcvt(zcord,4,tz);
			lcdPrint(&tz,87,215);
			setColor(0,255,0);
			if(direction%2 == 1){
				lcdPrint(compass[direction],35, 240);
			}
			else{
				setColor(0,50,100);
				fillRect(35,240,200,275);
				setColor(0,255,0);
				lcdPrint(compass[direction],90, 240);
			}

			usleep(500000);

			if(waypoint <= 8 && next == 0){
				itoa(waypoint, wayprint, 10);
				lcdPrint(&wayprint, 190, 35);
				dir[waypoint-1]=direction;
				xdata[waypoint-1] = xcord;
				ydata[waypoint-1] = ycord;
				zdata[waypoint-1] = zcord;
				num = waypoint;
				step_count = 0;
				zstep=0;
				setColor(0,50,100);
				fillRect(190,135,230,160);
				setColor(0,255,0);
			}
			else if(waypoint>8 || next == 1){
				resetDisplay();
				setColor(0,255,0);
				lcdPrint("THANK YOU", 45, 188);
				if(waypoint>1){
					waystatus=1;
				}
				next=7;
			}

			return Q_HANDLED();
		}
	}

	return Q_SUPER(&Lab3A_on);

}

QState Lab3A_Tour(Lab3A *me) {
	switch (Q_SIG(me)) {
		case Q_ENTRY_SIG: {
			//xil_printf("\n\rOn");
			}
		case CALIBRATE: {
			Display();
			stage = 0;
			return Q_TRAN(&Lab3A_Calibrate);
			}
		case TOUR:{
			tourstatus=1;
			return Q_HANDLED();
		}
		case TICK:{
			if(next==0){
				if(tour==0 && tstate ==1){
					resetDisplay();
					setColor(0,255,0);
					lcdPrint("3D TOUR", 60, 170);
					lcdPrint("NEXT:", 90, 215);
					lcdPrint("PRESS P17", 45, 240);
					tstate=0;
				}
				if(tour<=num && tour>=1 && tourstatus==1){
					if(tour==1){
						resetDisplay();
						setColor(0,255,0);
						lcdPrint("WAY POINT ", 35, 35);
						lcdPrint("X:", 60, 135);
						lcdPrint("Y:", 60, 160);
						lcdPrint("Z:", 60, 185);
					}
					//xil_printf("hi");
					itoa(tour, wayprint, 10);
					lcdPrint(&wayprint, 190, 35);
					gcvt(xdata[tour-1],4,tx);
					lcdPrint(&tx,87,135);
					gcvt(ydata[tour-1],4,ty);
					lcdPrint(&ty,87,160);
					gcvt(zdata[tour-1],4,tz);
					lcdPrint(&tz,87,185);
					//xil_printf("%s %d\n",compass[dir[tour-1]],tour-1);
					setColor(0,255,0);
					if(dir[tour-1]%2 == 1){
						lcdPrint(compass[dir[tour-1]],35, 240);
					}
					else{
						setColor(0,50,100);
						fillRect(35,240,200,275);
						setColor(0,255,0);
						lcdPrint(compass[dir[tour-1]],90, 240);
					}
					tourstatus=0;
					//xil_printf("%s %s %s",tx,ty,compass[dir[tour-1]]);
				}
				else if(tour>num){
					next=1;
				}
			}
			else if(next==1){
				resetDisplay();
				setColor(0,255,0);
				lcdPrint("THANK YOU", 45, 188);
				next=7;
				tour=-1;
				tstate=1;
			}
			return Q_HANDLED();
		}
	}

	return Q_SUPER(&QHsm_top);
}

void stateCall(u32 btn){
	//xil_printf("%d ",btn);
	if(btn == 1){
		//lcdPrint("<MODE 1>", 40, 80);
		stage = 0;
		dispatch(CALIBRATE);

	}
	else if(btn == 4){
		//lcdPrint("<MODE 2>", 40, 80);
		waypoint +=1;
		dispatch(WAYPOINT);
		//usleep(50000);
	}
	else if(btn == 8){
		next +=1;
		//usleep(50000);
	}
	else if(btn == 2){
		tour += 1;
		dispatch(TOUR);
		//usleep(50000);
	}
}

void stepCalc(float x, float y, float z){
	acc = 9.8*sqrt(x*x+y*y+z*z);
	//printf("acc: %f \n",acc);
	if(acc>2 && acc<3){
		phase = 1;
	}
	else if (acc>0.7 && acc<1.5 && phase == 1){
		step_count+=1;
		step_status = 1;
		phase = 0;
		//xil_printf("step:%d ",step_count);
	}
	if(z>0.13 && acc>1.5 && acc<2){
		zstep+=1;
		//printf("step:%d z:%f acc:%f\n ",zstep,z,acc);
	}
	else if(z<-0.13 && acc>1.5 && acc<2){
		zstep-=1;
		//printf("step:%d z:%f acc:%f\n ",zstep,z,acc);
	}
}

void xdistCalc(float x, float y){
	//printf("X:%.2f, Y%.2f", x, y);
	if(x>xavg_error){
		xvel = xvel + 9.8*x*t;
	}
	if(xvel>0){
		xdist = xdist + xvel*t;
	}
	else{
		xdist = xdist - xvel*t;
	}

	if(y>yavg_error){
		yvel = yvel + 9.8*y*t;
	}
	if(yvel>0){
		ydist = ydist + yvel*t;
	}
	else{
		ydist = ydist - yvel*t;
	}
}

void zdistCalc(float z){
	if(z>zavg_error){
		zvel = zvel + 9.8*z*t;
	}
	if(zvel>0){
		zdist = zdist + zvel*t;
	}
	else{
		zdist = zdist - zvel*t;
	}
}

void Display(){
	setColor(0, 0, 0);
	fillRect(0,0,240,320);
	setColor(255, 0, 0);
	for (int i=0; i<8; i++){
		for (int j=0; j<6; j++){
			fillRect(5+j*40, 5+i*40, 35+j*40, 35+i*40);
		}
	}
}

void resetDisplay()
{
	setColor(0, 50, 100);
	fillRect(5, 125, 235, 275);

}
