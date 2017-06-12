#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "anrolab5/oint_control_srv.h"
#include <math.h>

double const PI=M_PI;
double pozycja[3]; // Pozycja układu końcówki w globalnym układzie odniesienia
double teta1,teta2,d3; // Szukane wartości
double f=10;
double teta10=0; // Wartości początkowe - może zbędne
double teta20=0;
double d30=1.65;
bool oint_aktywny=false;

void callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	pozycja[0]=msg->pose.position.x;
	pozycja[1]=msg->pose.position.y;
	pozycja[2]=msg->pose.position.z;
	oint_aktywny=true;
}

int main(int argc, char **argv)
{
	// Inicjalizacja ros-a
	ros::init(argc,argv,"IKIN");
	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<sensor_msgs::JointState>("/joint_states",1000);
	ros::Subscriber sub=nh.subscribe<geometry_msgs::PoseStamped>("/PoseStamped",1000,callback);
	ros::Rate rate(f);

	double a1=0.9,a2=0.6,d1=0.5; // Wartości do pobrania z serwera parametrów
	double joint2_lower=-2.9,joint2_upper=2.9,joint3_lower=-0.15,joint3_upper=0.15; // Dane domyślne 
	// Pobranie danych z serwera parametrów

	if(!nh.getParam("/arm1",a1))
		{
			ROS_WARN("Nie udalo sie pobrac parametrow. Uzyte zostana domyslne wartosci.");	
		}
	if(!nh.getParam("/arm2",a2))
		{
			ROS_WARN("Nie udalo sie pobrac parametrow. Uzyte zostana domyslne.wartosci");	
		}
	if(!nh.getParam("/joint2_lower",joint2_lower))
		{
			ROS_WARN("Nie udalo sie pobrac parametrow. Uzyte zostana domyslne wartosci.");	
		}
	if(!nh.getParam("/joint2_upper",joint2_upper))
		{
					ROS_WARN("Nie udalo sie pobrac parametrow. Uzyte zostana domyslne wartosci.");	
		}
	if(!nh.getParam("/joint3_lower",joint3_lower))
		{
					ROS_WARN("Nie udalo sie pobrac parametrow. Uzyte zostana domyslne wartosci.");
		}
	if(!nh.getParam("/joint3_upper",joint3_upper))
		{
					ROS_WARN("Nie udalo sie pobrac parametrow. Uzyte zostana domyslne wartosci.");
		}
	// Wysłanie wiadomości do węzła oint - początkowe położenie robota
	ros::ServiceClient klient = nh.serviceClient<anrolab5::oint_control_srv>("oint_control_srv");

	anrolab5::oint_control_srv srv;
	srv.request.x = 1.5;
	srv.request.y = 0;
	srv.request.z = 0.5;
	srv.request.time = 0.1;
	for (int i=0;i<50;i++)	
	klient.call(srv);

	while(ros::ok())
	{
		ros::spinOnce(); // Pobranie informacji
		if(oint_aktywny)		
		{
			// Obliczenie odwrotnej kinematyki
			double costeta2=(pozycja[0]*pozycja[0]+pozycja[1]*pozycja[1]-a1*a1-a2*a2)/(2*a1*a2);
			double sinteta2=sqrt(1-costeta2*costeta2);
			double A=a1+a2*costeta2;
			double B=a2*sinteta2;
			double sinteta1=(B*pozycja[0]+A*pozycja[1])/(pozycja[0]*pozycja[0]+pozycja[1]*pozycja[1]);
			double costeta1=(A*pozycja[0]-B*pozycja[1])/(pozycja[0]*pozycja[0]+pozycja[1]*pozycja[1]);
			teta1=atan2(sinteta1,costeta1);
			teta2=-atan2(sinteta2,costeta2);
			d3=-pozycja[2]+d1;
			// Potrzeba sprawdzić, czy wartości spełniają ograniczenia

		}
		if(oint_aktywny)
		{
			sensor_msgs::JointState doWyslania;
			doWyslania.header.stamp=ros::Time::now();
			// Tworzenie wiadomości do wysłania	
			doWyslania.header.frame_id="";
			doWyslania.position.push_back(teta1); // Dodaje element na końcu wektora
			doWyslania.position.push_back(teta2);
			doWyslania.position.push_back(d3);
			doWyslania.name.push_back("joint1");
			doWyslania.name.push_back("joint2");
			doWyslania.name.push_back("joint3");

			pub.publish(doWyslania);
		}
		rate.sleep();		
	}
	return 0;
}
