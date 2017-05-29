#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <stdio.h>
#include <sstream>
#include <termios.h>
#define KEYCODE_W 0x57
#define KEYCODE_S 0x53
#define KEYCODE_A 0x41
#define KEYCODE_D 0x44


class TurtleKey{

public:
	TurtleKey();
	void mainloop();
private:
	ros::NodeHandle node;
	ros::Publisher turtle_vel;
	int getch();
	ros::Rate loop_rate(10);   
};

TurtleKey::TurtleKey(){
	
	
}

int TurtleKey::getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}
void TurtleKey::mainloop()
{
	
	bool isTrue = false;
	while (ros::ok()){
		int c = getch();
		switch(c)
		{
			case 'a':
			send message 'A'			
			break;

    
  		}
    	}
};

int main(int argc, char** argv){
  	ros::init(argc, argv, "my_turtle");
  	TurtleKey turtle_key;

  	turtle_key.mainloop();
  
	return(0);
};





