#include <ros/ros.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tps_base_link");

  ros::NodeHandle n;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  tf2_ros::TransformBroadcaster tb;
  geometry_msgs::TransformStamped ts;
  
  ros::Rate r(25.0);
  
  while(ros::ok()){
	geometry_msgs::TransformStamped transform;
	try{
	  transform = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
	}
	catch (tf2::TransformException &ex){
	  ROS_ERROR("%s", ex.what());
	  ros::Duration(1.0).sleep();
	  continue;
	}

	ts.header.stamp = transform.header.stamp;
	ts.header.frame_id = "map";
	ts.child_frame_id = "tps_base_link";
    
    ts.transform.translation.x = transform.transform.translation.x;
	ts.transform.translation.y = transform.transform.translation.y;
	ts.transform.translation.z = 0;

    tf2::Quaternion rotation;
	rotation.setRPY(0, 0, 0);
	ts.transform.rotation.x = rotation.x();
	ts.transform.rotation.y = rotation.y();
	ts.transform.rotation.z = rotation.z();
	ts.transform.rotation.w = rotation.w();

    tb.sendTransform(ts);
    
	r.sleep();
  }
  return 0;
}
