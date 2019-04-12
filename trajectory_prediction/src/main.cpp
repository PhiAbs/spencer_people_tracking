#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sstream>

#include <algorithm>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <spencer_tracking_msgs/TrackedPersons.h>
#include <spencer_tracking_msgs/TrackedPerson.h>


void pubTfForTrackedPersons(spencer_tracking_msgs::TrackedPersons tracked_persons)
{
  if(tracked_persons.tracks.size() > 0)
  {
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
    }
  }
}


void trackedPersonsCallback(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg)
{
  pubTfForTrackedPersons(*msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracked_and_confirmed_persons_tf");
  ros::NodeHandle n;

  //   load params
  std::string sub_topic;
  n.param("yolo_confirmed_tracks_topic", sub_topic, std::string("/spencer/perception/tracked_persons_confirmed_by_yolo"));

  ros::Subscriber sub = n.subscribe(sub_topic, 1, trackedPersonsCallback);

  ros::Rate loop_rate(20);
  
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
