#include <pose_ekf_fusion/Node.hpp>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "pose_ekf_fusion");
	ros::NodeHandle nh;
	//ros::NodeHandle pnh("~");
	eskf::ESKFNode node(nh);
	ros::spin();
	return 0;
}