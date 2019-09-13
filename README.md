Developed for ARM board FRDM-KL25Z
	
	This project implements the following features:
	
	1) Vary motor speed with PWM
	2) Set 6 different speeds, including no speed at all
	3) Calculate and display RPM speed
	4) Change and display the direction of rotation
	
	Non-implemented requirements:
	1) Detect the motor is stopped
	
	Threads:
	1)t_control: it implements the motor control state-machine
	2)t_rpm: it calculates the rpm speed
	3)t_rotation: it determines the direction of rotation
	4)t_dirButton: it polls a button event used to change the direction of rotation
	5)t_startButton: it polls a button event used to start up the system
  	
	Messages Queues:
	1)interval_q_id: it sends a sum of channel values of channel 0
	2)rotation_q_id: it sends a sum difference of two different channel values at 
	two different channel interrupt (inside flag checks)
	
	Event Flags:
	1) start_evt_flags: it sends a signal to the t_control thread to start up 
