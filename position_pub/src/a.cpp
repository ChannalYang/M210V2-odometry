
#include <fstream>

#include "stdio.h"

#include <iostream>
#include <stdlib.h>
using namespace std;

int main(int argc, char **argv){

float temp;
std::ifstream infile("/home/dji/catkin_onboard/src/position_pub/src/aim_point.txt");
 if(!infile.is_open()){
 std::cout<<"failed";
 }
else {
infile>>temp;
 std::cout<<temp;
}
	return 0;
}


