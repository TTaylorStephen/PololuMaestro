#include "armcode.h"

// initializes maestro board of choosing 
int maestro::Init(const char * device){
	int fd = open(device, O_RDWR | O_NOCTTY);
	struct termios options;
 	tcgetattr(fd, &options);
 	options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
 	options.c_oflag &= ~(ONLCR);
 	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
 	tcsetattr(fd, TCSANOW, &options);
	if (fd == -1){
		perror(device);
  return 1;
	}
return fd;
}

//returns error code if communication unsuccessful 
//shifts high byte to the left (256*response)
int maestro::getError(){
  unsigned char command[] = {0xA1};
  if(write(fd, command, sizeof(command)) == -1){
		perror("error writing");
  	return -1;
  }
  unsigned char response[2];
  if(read(fd,response,2) != 2){
    perror("error reading");
    return -1;
  }
	return response[0] + 256*response[1];
}

//gets current position of each motor
int maestro::getPosition(unsigned char channel){
  unsigned char command[] = {0x90, channel};
  if(write(fd, command, sizeof(command)) == -1){
    perror("error writing");
    return -1;
  }
  unsigned char response[2];
  if(read(fd,response,2) != 2){
    perror("error reading");
    return -1;
  }
   return response[0] + 256*response[1];
}		

//sets values for pulse width to change position 
int maestro::setTarget(unsigned char channel, unsigned short target){
  unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
  if (write(fd, command, sizeof(command)) == -1){
		perror("error writing");
  	return -1;
 	}
 	return 0;
}

//sets up to 12 targets, first byte is target number, second is the lowest channel in use, remainder are input as a vector
int maestro::setMultipleTargets(unsigned char target_num, unsigned char low_channel, unsigned short targets[12]){
	unsigned char command[] = {0x9F, target_num, low_channel, targets[0] & 0x7F, targets[0] >> 7 & 0x7F, targets[1] & 0x7F, targets[1]>>7 & 0x7F, targets[2] & 0x7F, targets[2]>>7 & 0x7F, targets[3] & 0x7F, targets[3]>>7 & 0x07F,targets[4] & 0x07F, targets[4]>>7 & 0x7F,targets[5] & 0x7F, targets[5]>>7 & 0x7F,targets[6] & 0x07F, targets[6]>>7 & 0x07F,targets[7] & 0x07F, targets[7]>>7 & 0x7F,targets[8] & 0x7F, targets[8]>>7 & 0x07F, targets[9] & 0x07F, targets[9]>>7 & 0x07F, targets[10] & 0x7F, targets[10]>>7 & 0x7F, targets[11] & 0x7F, targets[11]>>7 & 0x7F};
	if (write(fd, command, sizeof(command)) == -1){
		perror("error writing");
  	return -1;
 	}
 	return 0;
} 

// Sets the speed of the Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// Corresponds to speed* 0.25microseconds/10millisecond in the PWM duty cycle.
// A speed of zero makes the PWM signal change immediately.
int maestro::setSpeed(unsigned char channel, unsigned short speed){
   unsigned char command[] = {0x87, channel, speed & 0x7F, speed >> 7 & 0x7F};
   if(write(fd, command, sizeof(command)) == -1){
   perror("error writing");
   return -1;
   }
	return 0;
}

//sets acceleration of motor on desired maestro channel
// ramp>>7 = lower byte, else ramp = higher byte
// write (const char* s, streamsize n ); inserts the first no chars of the array pointed by s
int maestro::setAcceleration(unsigned char channel, unsigned short ramp){
	unsigned char command[] = {0x89, channel, ramp & 0x7F , ramp >> 7 & 0x7F}; 
	if (write(fd, command, sizeof(command))==-1){
		perror("error writing");
		return -1;
	}
	return 0;
}

maestro::maestro(){
printf("Initializing Maestro... \n");
}

maestro::~maestro(){
printf("Parameters Set. \n");
}

//back and forth motion
int maestro::sweep(int button){
		int position = getPosition(0);
		printf("Current position is %d.\n", position);
		if (button==2){
			if (position<4100){
				int target=7000;
				printf("Setting target to %d (%d us).\n", target, target/4);
				setTarget(0,target);
			}
			if (position==7000){
				int target=4000;
				printf("Setting target to %d (%d us).\n", target, target/4);
				setTarget(0,target);
			}
		return 0;
	}
}

//spinning motion
int maestro::spin(int button){
	int target = getPosition(1); 
	while(1){
	printf("Setting target to %d (%d us).\n", target, target/4);
	setTarget(1, target);
	target = target++;
	}
	return 0;
}

//operates servo with keyboard 119="w", 115="s"
int maestro::extend(int button){
	int target = getPosition(0);
	if (button==3){
		target+=100;
		printf("Setting target to %d (%d us) \n", target, target/4);
		setTarget(0,target);
		}	
	if (button==0){
		target-=100;
		printf("Setting target to %d (%d us) \n", target, target/4);
		setTarget(0,target);
		}
	return 0;
}

void maestro::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  int a=joy->buttons[0];
  int b=joy->buttons[1];
  int x=joy->buttons[2];
	int y=joy->buttons[3];
	sweep(x);
	spin(b);
	if(y){
		extend(y);
	}
	if(a){
		extend(a); 
	}
	std::cout<<"buttons: ["<<  joy->buttons[0]  <<","<< joy->buttons[1] <<","<<joy->buttons[2] <<","<<joy->buttons[3]<<std::endl;
}


 	
int main(int argc, char** argv){

	maestro Arm; 
	Arm.fd = Arm.Init("/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Micro_Maestro_6-Servo_Controller_00253572-if00");
	ros::init(argc, argv, "science_node");

	while(1){

		Arm.setSpeed(0,150);
		Arm.setAcceleration(0,150);
		ros::NodeHandle nh;
		ros::Subscriber joy_sub;
		joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &maestro::joyCallback, &Arm);
	}

	ros::spin();
	return 0;
}


//s pwm cycle as specified on time and period in units of 1/48 us, 
//both on-time and period encoded w/ 7 bits per byte, same as setTarget
//periods 1020 (47.1 kHz), 4080 (11.7 kHz), and 16320 (2.9 kHz) provide the best possible res with 100% and 0% duty cycle 
/*int setPWM(int fd, unsigned short ontime, unsigned short period){
	unsigned char command[]={0x8A, ontime & 0x07F, ontime >> 7 & 0x07F, period & 0x07F, period >> 7 & 0x07F};
	if (write(fd, command, sizeof(command)) == -1){
		perror("error writing");
  	return -1;
 	}
 	return 0;
} */

//sets multiple targets
	/*unsigned short targets[12] ;
	targets[0] = (position < 6000) ? 12567 : 1000 ;
	targets[1] = (position1 < 6000) ? 12567 : 1000 ;
	printf("Setting position to %d.\nSetting position1 to %d.\n", targets[0], targets[1]);
  Arm.setMultipleTargets(2, 0, targets);*/


