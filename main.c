/******************************************************************************/
/*                                                                            */
/* main.c -- Demo for the PmodNAV using SPI and UART                          */
/*                                                                            */
/******************************************************************************/
/* Author: Samuel Lowe, Arthur Brown                                          */
/* Copyright 2016, Digilent Inc.                                              */
/******************************************************************************/
/*
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
/******************************************************************************/
/* File Description:                                                          */
/*                                                                            */
/* This file contains a demo to be used with the PmodNAV. Accelerometer,      */
/* Magnetometer, Temperature, and Barometric Pressure data is captured over a */
/* SPI interface and displayed over UART.                                     */
/*                                                                            */
/* This application configures UART 16550 to baud rate 9600.                  */
/* PS7 UART (Zynq) is not initialized by this application, since bootrom/bsp  */
/* configures it to baud rate 115200                                          */
/*                                                                            */
/*  ------------------------------------------------                          */
/*  | UART TYPE   BAUD RATE                        |                          */
/*  ------------------------------------------------                          */
/*    Uartns550   9600                                                        */
/*    Uartlite    Configurable only in HW design                              */
/*    ps7_uart    115200 (configured by bootrom/bsp)                          */
/*                                                                            */
/******************************************************************************/
/* Revision History:                                                          */
/*                                                                            */
/*    11/16/2016(SamL):     Created                                           */
/*    03/16/2017(ArtVVB):   Completed & Validated                             */
/*    11/01/2017(ArtVVB):   2016.4 Maintenance                                */
/*    02/20/2018(atangzwj): Validated for Vivado 2017.4                       */
/*                                                                            */
/******************************************************************************/
/* Problems:                                                                  */
/*                                                                            */
/* In order to include "math.h" you must include m in compiler settings.      */
/* See https://www.xilinx.com/support/answers/52971.html                      */
/*                                                                            */
/******************************************************************************/

/***************************** Include Files *******************************/

#include <stdio.h>
#include "math.h"
#include "PmodNAV.h"
#include "sleep.h"
#include "xil_cache.h"
#include "xparameters.h"
#include "lcd.h"
#include "bsp.h"
#include "qpn_port.h"                                       /* QP-nano port */
#include "stepanalyzer.h"

/*************************** Global Variables ******************************/

PmodNAV nav;
float xval = 0.0;
float yval = 0.0;
float zval = 0.0;
float magXYd = 0.0;
int direction = 2;

float magData_X = 0.0;
float magData_Y = 0.0;
float magData_Z = 0.0;
char dir[3];

/********************* Function Forward Declarations ***********************/

void NavDemo_Initialize(void);
void NavDemo_Run(void);
void NavDemo_Cleanup(void);
void DataRead(void);

float NavDemo_ComputePref(float hPa, float altitudeMeters);
float NavDemo_ConvPresToAltF(float Pref, float hPa);
float NavDemo_ConvPresToAltM(float Pref, float hPa);

float NavDemo_ConvTempCToTempF(float tempC);
float NavDemo_ConvFeetToMeters(float feet);
float NavDemo_ConvMetersToFeet(float meters);

float NavDemo_AngleInXY(NAV_RectCoord r);
float NavDemo_DegreesFromVertical(NAV_RectCoord r);
float NavDemo_ScalarProjection(NAV_RectCoord orient, NAV_RectCoord r);

void NavDemo_EnableCaches(void);
void NavDemo_DisableCaches(void);

void DataRead();
void getDirection();
void caliMag();

/***************************** Function Definitions ************************/
static QEvent l_lab3AQueue[30];

QActiveCB const Q_ROM Q_ROM_VAR QF_active[] = {
	{ (QActive *)0,            (QEvent *)0,          0                    },
	{ (QActive *)&AO_Lab3A,    l_lab3AQueue,         Q_DIM(l_lab3AQueue)  }
};

Q_ASSERT_COMPILE(QF_MAX_ACTIVE == Q_DIM(QF_active) - 1);

int main(void) {
	magData_X = 0.0;
	magData_Y = 0.0;
	NavDemo_Initialize();
	Lab3A_ctor(); // inside of stepanalyzer.c
	BSP_init(); // inside of bsp.c, starts out empty!
	QF_run(); // inside of qfn.c

    //NavDemo_Cleanup();
   return 0;
}

void DataRead(){
	NAV_GetData(&nav);
	xval = nav.acclData.X;
	yval = nav.acclData.Y;
	zval = nav.acclData.Y;
}

void getDirection(){
	magXYd = NavDemo_AngleInXY(nav.magData);
	direction = (int)((magXYd + 22.5) / 45.0) % 8;
}

void caliMag(){
	NAV_ReadMagGauss(&nav);
	magData_X += nav.magData.X;
	magData_Y += nav.magData.Y;
	magData_Z += nav.magData.Z;
	//xil_printf("Hi");
}

/*** void NavDemo_Initialize()
**
**   Parameters:
**      None
**
**   Return Values:
**      None
**
**   Errors:
**      None
**
**   Description:
**      This function initializes the hardware used in the demo and starts a
**      PmodNAV driver device
*/
void NavDemo_Initialize(void) {
    NavDemo_EnableCaches();
    xil_printf("Pmod Nav Demo Initializing...\n\r");
    NAV_begin ( // intialize the PmodNAV driver device
        &nav,
        XPAR_GPIO_NAV_BASEADDR,
        XPAR_SPI_NAV_BASEADDR
    );
    xil_printf("Pmod Nav Demo Initialized\n\r");
    NAV_Init(&nav); // initialize the connection with each spi slave
    NAV_GetData(&nav);
}

/*** float NavDemo_AngleInXY(NAV_RectCoord r)
**
**   Parameters:
**      r - the vector in rectangular coordinates to be converted to polar
**
**   Return Value:
**      p - returns the polar coordinate representation of the vector r
**          projected onto the XY plane
**
**   Errors:
**      None
**
**   Description:
**      The function computes the degrees the vector r is rotated about the
**      Z-axis from the vector (X=1,0,0)
*/
float NavDemo_AngleInXY(NAV_RectCoord r) {
   float d;
   if (r.X - (magData_X/50) == 0)
      d = (r.Y-(magData_Y/50) < 0) ? 90 : 0;
   else
      d = atan2f(r.Y-(magData_Y/50), r.X-(magData_X/50)) * 180 / M_PI;
   if (d > 360)
      d -= 360;
   else if (d < 0)
      d += 360;
   return d;
}

/*** void NavDemo_EnableCaches(void)
**
**   Parameters:
**      None
**
**   Return Values:
**      None
**
**   Errors:
**      None
**
**   Description:
**      This function enables the instruction and/or data caches on
**      architectures that require them
*/
void NavDemo_EnableCaches(void) {
#ifdef __MICROBLAZE__
#ifdef XPAR_MICROBLAZE_USE_ICACHE
   Xil_ICacheEnable();
#endif
#ifdef XPAR_MICROBLAZE_USE_DCACHE
   Xil_DCacheEnable();
#endif
#endif
}

/*** void NavDemo_DisableCaches(void)
**
**   Parameters:
**      None
**
**   Return Values:
**      None
**
**   Errors:
**      None
**
**   Description:
**      This function disables the instruction and/or data caches on
**      architectures that require them
*/
void NavDemo_DisableCaches(void) {
#ifdef __MICROBLAZE__
#ifdef XPAR_MICROBLAZE_USE_ICACHE
   Xil_ICacheDisable();
#endif
#ifdef XPAR_MICROBLAZE_USE_DCACHE
   Xil_DCacheDisable();
#endif
#endif
}
