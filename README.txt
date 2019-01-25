Gal Meir - 305382137
I have implemented only the basic functions (no bonuses)
Commands implemented:
	Left and Right arrows – rotation around the Z axis of the picked item.
	Up and Down arrows – rotation around the X axis of the picked item.
	space - starts and stop the IK solver. Any limitation on the solver (if the box is too far or too close in Euler mode) will appear in the window title.
	'E' - toggle between Euler an Angle-Axis mode. The current mode will appear in the window title.
	'P'- unlike what was asked will print only the global rotation matrix. The Euler angles of the picked item (even for the global matrix) are visible in all time in the window title.
	All mouse movement as required

Added functions (done for testing):
	'X' - restarts the position of the arm and box.
	'A' and 'Z' - toggle between the global, arm parts and box as if they were picked. 
