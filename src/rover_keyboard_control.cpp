// ROS Libraries
#include <ros/ros.h>
#include <std_msgs/Float64.h>

// Misc Libraries
#include <termios.h>
#include <thread>

#define MOTOR_NUM 6

// defining publishers
ros::Publisher vel_state_pub[MOTOR_NUM];

// Control variables
std_msgs::Float64 set_vel[MOTOR_NUM];
float cur_vel[MOTOR_NUM];													
float prev_vel[MOTOR_NUM];

// Misc variables
static struct termios old, current;
bool do_not_quit = true;
bool vel_changed = false;
bool show_vel_values = true;

// Prototype
void display_help(void);

/* Receiving set point from the user */
void initTermios(void){
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  current = old; /* make new settings same as old settings */
  current.c_lflag &= ~ICANON; /* disable buffered i/o */
  current.c_lflag &= ~ECHO; /* set no echo mode */
  tcsetattr(0, TCSANOW, &current); /* use these new terminal i/o settings now */
}
void resetTermios(void){
  tcsetattr(0, TCSANOW, &old);
}
char getch(void){
  char ch;
  initTermios();
  ch = getchar();
  resetTermios();
  return ch;
}


void read_input(void){
	char input;
	
	while(do_not_quit){
		input = getch();
		
		switch(input){
			case 'x': do_not_quit = false;
					  break;
			case 'h': display_help();
					  break;
			case 'v': if(show_vel_values) show_vel_values = false;
					  else show_vel_values = true;
					  break;
			case 'w': for(int i=0;i<MOTOR_NUM;i=i+2) cur_vel[i] += 0.1;
					  for(int i=1;i<MOTOR_NUM;i=i+2) cur_vel[i] -= 0.1;						
					  break;
			case 's': for(int i=0;i<MOTOR_NUM;i=i+2) cur_vel[i] -= 0.1;
					  for(int i=1;i<MOTOR_NUM;i=i+2) cur_vel[i] += 0.1;						
					  break;
			case 'a': for(int i=0;i<MOTOR_NUM;i=i+2) cur_vel[i] = 0;
					  for(int i=1;i<MOTOR_NUM;i=i+2) cur_vel[i] -= 0.05;
					  break;
			case 'd': for(int i=0;i<MOTOR_NUM;i=i+2) cur_vel[i] += 0.05;
					  for(int i=1;i<MOTOR_NUM;i=i+2) cur_vel[i] = 0;
					  break;
					  
			case 'W': for(int i=0;i<MOTOR_NUM;i=i+2) cur_vel[i] += 0.8;
					  for(int i=1;i<MOTOR_NUM;i=i+2) cur_vel[i] -= 0.8;							
					  break;
			case 'S': for(int i=0;i<MOTOR_NUM;i=i+2) cur_vel[i] -= 0.8;
					  for(int i=1;i<MOTOR_NUM;i=i+2) cur_vel[i] += 0.8;						
					  break;
			case 'A': for(int i=0;i<MOTOR_NUM;i=i+2) cur_vel[i] = 0;
					  for(int i=1;i<MOTOR_NUM;i=i+2) cur_vel[i] -= 0.4;
					  break;
			case 'D': for(int i=0;i<MOTOR_NUM;i=i+2) cur_vel[i] += 0.4;
					  for(int i=1;i<MOTOR_NUM;i=i+2) cur_vel[i] = 0;
					  break;	
			
			case 'f': for(int i=0;i<MOTOR_NUM;i++) cur_vel[i] =0;	  				
		}		
	}
}

/* Sending efforts to Gazebo */
void send_vel_values(void){
	for(int i=0;i<MOTOR_NUM;i++){
		if(prev_vel[i]!=cur_vel[i]){
			vel_changed = true;
			break;
		}
	}
	if(vel_changed){
		for(int i=0;i<MOTOR_NUM;i++){
			prev_vel[i] = cur_vel[i];
			set_vel[i].data = cur_vel[i];
			vel_state_pub[i].publish(set_vel[i]);
			ros::spinOnce();
		}
		
		if(show_vel_values){
			std::cout << cur_vel[0] << " " << cur_vel[1] << " | "
				  << cur_vel[2] << " " << cur_vel[3] << " | "
				  << cur_vel[4] << " " << cur_vel[5] << std::endl;
		}
		
		vel_changed = false;
	}
}

void display_help(void){
	std::cout << "\nUse the following commands: "<< std::endl
			<< "w: Forward" << std::endl
			<< "a: Left" << std::endl
			<< "s: Back" << std::endl
			<< "d: Right" << std::endl
			<< "f: Stop" << std::endl
			<< "Use shift to increase acceleration" << std::endl
			<< "v: Toggle display of velocity when changed (default: ON)" << std::endl
			<< "h: Display help" << std::endl
			<< "x: Quit\n" << std::endl;
}

int main(int argc, char **argv){
	
	ros::init(argc,argv,"rover_keyboard_controller");
	ros::NodeHandle nh;
	ros::Rate rate(2);

	
	// advertising the publishers
	vel_state_pub[0] = nh.advertise<std_msgs::Float64>("/rover/wheel_1_vel_controller/command", 1000);
	vel_state_pub[1] = nh.advertise<std_msgs::Float64>("/rover/wheel_2_vel_controller/command", 1000);
	vel_state_pub[2] = nh.advertise<std_msgs::Float64>("/rover/wheel_3_vel_controller/command", 1000);
	vel_state_pub[3] = nh.advertise<std_msgs::Float64>("/rover/wheel_4_vel_controller/command", 1000);
	vel_state_pub[4] = nh.advertise<std_msgs::Float64>("/rover/wheel_5_vel_controller/command", 1000);
	vel_state_pub[5] = nh.advertise<std_msgs::Float64>("/rover/wheel_6_vel_controller/command", 1000);
	
	for(int i=0;i<MOTOR_NUM;i++){
		cur_vel[i] = 0;
		prev_vel[i] = 0;
		set_vel[i].data=0.0;
	}
	
	display_help();
			
	std::thread reading_input_thread(read_input);
	reading_input_thread.detach();

	while(do_not_quit && ros::ok()){
		send_vel_values();
	}
}
