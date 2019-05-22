#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

ros::Subscriber odom_subscriber;

float goals[2][2] =
{
    {+0.6,  +4.9},
    {-4.75, -0.5}
};

int current_goal = 0;
bool current_goal_reached = false;
const float epsilon = 0.2f;

void SetMarker(ros::Publisher& marker_pub, visualization_msgs::Marker& marker, float x, float y, bool visible)
{
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visible ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETEALL;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker_pub.publish(marker);
}

// The laser_callback function will be called each time a laser scan data is received
void odom_callback(const nav_msgs::Odometry::ConstPtr& nav_msg)
{
    if (current_goal_reached)   // Nothing to do
        return;

    //ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]",
    //    nav_msg->pose.pose.position.x, nav_msg->pose.pose.position.y, nav_msg->pose.pose.position.z);

    float dx = nav_msg->pose.pose.position.x - goals[current_goal][0];
    float dy = nav_msg->pose.pose.position.y - goals[current_goal][1];

    float d = sqrt(dx*dx + dy*dy);
    
    ROS_INFO("Distance = %f\n", d);

    if (d < epsilon)
    {
        current_goal_reached = true;
    }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;

  // Subscribe to the /scan topic and call the laser_callback function
  odom_subscriber = n.subscribe("/odom", 1000, odom_callback);

  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;    
    marker.lifetime = ros::Duration();

    SetMarker(marker_pub, marker, goals[current_goal][0], goals[current_goal][1], true);

    // Enter an infinite loop where the laser_callback function will be called when new laser messages arrive
    ros::Duration time_between_ros_wakeups(0.001);
    while (ros::ok() && !current_goal_reached) {
        ros::spinOnce();
        time_between_ros_wakeups.sleep();
    }

    ROS_INFO("Picking up object ...\n");


    // Picking object
    sleep(5);
    SetMarker(marker_pub, marker, 0.0, 0.0, false);

    ROS_INFO("Object picked up, transporting ...\n");

    // Move to next goal
    ++current_goal;
    current_goal_reached = false;

    // Enter an infinite loop where the laser_callback function will be called when new laser messages arrive
    while (ros::ok() && !current_goal_reached) {
        ros::spinOnce();
        time_between_ros_wakeups.sleep();
    }

    ROS_INFO("Dropping object...\n");

    SetMarker(marker_pub, marker, goals[current_goal][0], goals[current_goal][1], true);

    ROS_INFO("Object dropped.\n");

    // Keep spinning for debug output
    while (ros::ok()) {
        ros::spinOnce();
        time_between_ros_wakeups.sleep();
    }
}
