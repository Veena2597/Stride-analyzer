# Stride-analyzer
The project is stand-alone personal navigation unit using PMOD NAV module with GUI on the LCD display. The hardware consists of a free-standing Nexsys board with PMOD NAV, powered by a 5V battery (small USB supply capable of 1.5A), and the display. 

Project details:
The code consists of two parts: calibration routine and real-time measurement.

The calibration routine is divided into 4 parts:
I. Magnetometer Offset: The user is instructed to rotate twice to estimate the magnetic offset.

II. Error estimation: The user is instructed to remain still for a period of 3 seconds, while holding the embedded system to estimate the expected value of error in acceleration measurement. This step calculates the expected error in all x,y and z axes. The major motive is z axis calibration. 
	
III. XY Step calibration: The user is instructed to move 3 steps at a normal pace and a faster pace to capture maximum acceleration a person can achieve while walking. This calibrate the step count algorithm to count only when the person is walking and not any sudden movements. 

IV. Z Step calibration: The user is instructed to move 3 steps in the upward or downward direction (staircase) to estimate the acceleration threshold of the z component. This differentiates actual movement in the z-direction with z-changes while walking. 
After eliminating the expected error calculated in step I, the average stride length of the user in XY and Z planes are estimated.  

The magnetometer reading is used to determine the direction in which the user takes each step. Depending on the direction the X, Y and Z coordinates are incremented or decremented correspondingly. North is taken as +Y axis, East as +X axis and Upward as +Z axis. The average stride calculated in the calibration is used to calculate the increment or decrement in the coordinates for every increase in the step count. The waypoints push button is pressed on the harwdare to record the real-time measurement, the instantaneous coordinates will stored in an array and displayed during the 3D tour. The way points stored were displayed on the LCD display. Push buttons were used to navigate through the points. 





