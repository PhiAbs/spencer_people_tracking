#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sstream>

#include <algorithm>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <spencer_tracking_msgs/TrackedPersons.h>
#include <spencer_tracking_msgs/TrajectoryPredictionLocalVelocity.h>
#include <spencer_tracking_msgs/TrajectoryPredictionsLocalVelocities.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

ros::Publisher pub;


spencer_tracking_msgs::TrajectoryPredictionLocalVelocity computeLocalVelocity(
    const spencer_tracking_msgs::TrackedPerson& tracked_person, 
    spencer_tracking_msgs::TrackedPersons& tracked_persons,
    std::string local_frame)
{
    // transform linear twist into local person tf
  spencer_tracking_msgs::TrajectoryPredictionLocalVelocity local_velocity;
  local_velocity.track_id = tracked_person.track_id;
  local_velocity.globalpose.header = tracked_persons.header;
  local_velocity.globalpose.pose = tracked_person.pose;
  local_velocity.localtwist.header = tracked_persons.header;
  local_velocity.localtwist.header.frame_id = local_frame;

  // Create rotation matrix from quaternion
  tf::Quaternion q_tf;
  tf::quaternionMsgToTF(tracked_person.pose.pose.orientation , q_tf);
  tf::Matrix3x3 rotation_matrix(q_tf);

  // Create tf vector from geometry_msgs vector
  tf::Vector3 twist_global;
  tf::vector3MsgToTF(tracked_person.twist.twist.linear, twist_global);

  // compute local twist in person tf
  tf::Vector3 twist_local;
  twist_local = rotation_matrix.inverse() * twist_global;
  tf::vector3TFToMsg(twist_local, local_velocity.localtwist.twist.linear);

  return local_velocity;
}


void pubTfAndLocalVelocityForTrackedPersons(spencer_tracking_msgs::TrackedPersons tracked_persons)
{
  if(tracked_persons.tracks.size() > 0)
  {
    spencer_tracking_msgs::TrajectoryPredictionsLocalVelocities local_velocities;    
    static tf::TransformBroadcaster tf_broadcaster;

    foreach(const spencer_tracking_msgs::TrackedPerson& tracked_person, tracked_persons.tracks)
    {
      std::ostringstream os;
      tf::Transform transform;
      tf::Vector3 tf_person_position;
      tf::Quaternion tf_person_quaternion;

      os << "person_" << tracked_person.track_id;
      tf::pointMsgToTF(tracked_person.pose.pose.position, tf_person_position);
      tf::quaternionMsgToTF(tracked_person.pose.pose.orientation, tf_person_quaternion);
      transform.setOrigin(tf_person_position);
      transform.setRotation(tf_person_quaternion);
      tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tracked_persons.header.frame_id, os.str()));

      // set the name of the person as a parameter for the static map extraction
      // ros::param::set("/costmap_node/costmap/global_frame", os.str());
      // ros::param::set("/costmap_node/costmap/robot_base_frame", os.str());
      // std::cout << os.str() << std::endl;

      // compute local velocities, store them together with position and frame information in message and push_back into message array
      local_velocities.tracks.push_back(computeLocalVelocity(tracked_person, tracked_persons, os.str()));
    }

    pub.publish(local_velocities);
  }
}


void trackedPersonsCallback(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg)
{
  pubTfAndLocalVelocityForTrackedPersons(*msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracked_and_confirmed_persons_tf");
  ros::NodeHandle n;

  //   load params
  std::string sub_topic;
  n.param("yolo_confirmed_tracks_topic", sub_topic, std::string("/spencer/perception/tracked_persons_confirmed_by_yolo"));

  pub = n.advertise<spencer_tracking_msgs::TrajectoryPredictionsLocalVelocities>("output", 1);
  ros::Subscriber sub = n.subscribe(sub_topic, 1, trackedPersonsCallback);

  ros::Rate loop_rate(20);
  
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
