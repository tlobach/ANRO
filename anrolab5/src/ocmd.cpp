#include "ros/ros.h"
#include "anrolab5/oint_control_srv.h"
#include <math.h>
#include <iostream>

double const PI=M_PI;
double f=10;
double a_elip=1.5,b_elip=0.7;

int main(int argc, char **argv)
{
	// Inicjalizacja ros-a
	ros::init(argc,argv,"OCMD");
	ros::NodeHandle nh;
	ros::Rate rate(f);
	ros::ServiceClient klient = nh.serviceClient<anrolab5::oint_control_srv>("oint_control_srv");
	anrolab5::oint_control_srv srv; // Wiadomość do wysłania	

	std::cout << "Wybierz rodzaj ruchu (wpisz wlasciwa cyfre i nacisnij ENTER): " << std::endl;
	std::cout << "1 - ruch po prostokacie" << std:: endl;
	std::cout << "2 - ruch po elipsie" << std::endl;
	int wybor;
	std::cin >> wybor;
	int licznik; // Zmienna do odmierzania czasu
	int wierzcholek; // Numer wierzcholka
	double x; // Zmienna dotycząca elipsy
	int kierunek; // Kierunek poruszania się elipsy 1-lewo, 2-prawo
	if(wybor==1) // Ruch po prostokacie
		{	
			wierzcholek=1;
			srv.request.x=1.2;
			srv.request.y=0.85;
			srv.request.z=0.5;
			srv.request.time=0.1;
			klient.call(srv);
	
		}

	while(ros::ok())
	{
		if(wybor==1) // Ruch po prostokacie
		{

				srv.request.x=1.2;
				srv.request.y=-0.85;
				srv.request.z=0.5;
				srv.request.time=8;
				klient.call(srv);
				ros::Duration(8).sleep();

				srv.request.x=-1.2;
				srv.request.y=-0.85;
				srv.request.z=0.5;
				srv.request.time=10;
				klient.call(srv);
				ros::Duration(10).sleep();

				srv.request.x=-1.2;
				srv.request.y=0.85;
				srv.request.z=0.5;
				srv.request.time=8;
				klient.call(srv);
				ros::Duration(8).sleep();

				srv.request.x=1.2;
				srv.request.y=0.85;
				srv.request.z=0.5;
				srv.request.time=10;
				klient.call(srv);
				ros::Duration(10).sleep();


		}
		else // Elipsa
		{

    			double dt=0.1;
    			srv.request.z = 0.5;
    			srv.request.time = 5;

			srv.request.x = a_elip*cos(0);
  			srv.request.y = b_elip*sin(0);
			srv.request.time = 2;
			klient.call(srv);
  			ros::Duration(2).sleep();
	
			srv.request.time = dt;
			double phi=0;
			while(phi<=2*PI){
	  			srv.request.x = a_elip*cos(phi);
  	  			srv.request.y = b_elip*sin(phi);
	  			klient.call(srv);
  	  			ros::Duration(dt).sleep();
	  			phi+=0.1;
			}
	  		srv.request.x = a_elip*cos(0);
  	  		srv.request.y = b_elip*sin(0);
	  		klient.call(srv);
	  		ros::Duration(dt).sleep();			

		}	
		if(klient.call(srv)){
      				ROS_INFO("Service called, %s", srv.response.status.c_str());
   		}else{
      				ROS_ERROR("Failed to call service.");
    				}
			ros::spinOnce();	
	}
	return 0;
}
