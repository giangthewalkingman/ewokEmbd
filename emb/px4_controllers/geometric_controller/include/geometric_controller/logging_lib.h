#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
using namespace std;
string name="";
void creates()
{  
	std::fstream file; 
    name = ros::package::getPath("geometric_controller") + "/glog/rigid.csv";
	file.open(name, std::ios::out | std::ios::app); 
	file << "TimeStamp,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16,a17,a18,Thrust\n";
    file.close(); 
} 
 
void updates(double time, 
                   double a1, double a2, double a3, 
                   double a4, double a5, double a6, 
                   double a7, double a8, double a9, 
                   double a10, double a11, double a12, 
                   double a13, double a14, double a15,
                   double a16, double a17, double a18,
                   double thrust
                   )
{
// {   int time_=floor(time*1000);
        // if(time_%100 < 8 ){
	std::fstream file;  
    file.open(name, std::ios::out | std::ios::app); 
    if(!file.is_open()) cout << "file oppen error";
    file <<setprecision(2)<<fixed<< time<< ", " 
                         << std::fixed << std::setprecision(8) << a1 << ", " 
						 << std::fixed << std::setprecision(8) << a2 << ", " 
						 << std::fixed << std::setprecision(8) << a3 << ", "
                         << std::fixed << std::setprecision(8) << a4 << ", " 
						 << std::fixed << std::setprecision(8) << a5 << ", " 
						 << std::fixed << std::setprecision(8) << a6 << ", "
						 << std::fixed << std::setprecision(8) << a7 << ", " 
						 << std::fixed << std::setprecision(8) << a8 << ", " 
						 << std::fixed << std::setprecision(8) << a9 << ", "
                         << std::fixed << std::setprecision(8) << a10 << ", " 
						 << std::fixed << std::setprecision(8) << a11 << ", " 
						 << std::fixed << std::setprecision(8) << a12 << ", "
                         << std::fixed << std::setprecision(8) << a13 << ", " 
						 << std::fixed << std::setprecision(8) << a14 << ", " 
						 << std::fixed << std::setprecision(8) << a15 << ", "
                         << std::fixed << std::setprecision(8) << a16 << ", " 
						 << std::fixed << std::setprecision(8) << a17 << ", " 
                         << std::fixed << std::setprecision(8) << a18 << ", " 
						 << std::fixed << std::setprecision(8) << thrust<< "\n";

	file.close(); 
// } 
}
