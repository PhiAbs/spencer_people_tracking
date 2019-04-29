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
#include <spencer_tracking_msgs/TrajectoryPredictionAPG.h>
#include <spencer_tracking_msgs/TrajectoryPredictionAPGs.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <cmath>
#include <math.h>

#define PI 3.14159265

ros::Publisher pub_local_velocities;
ros::Publisher pub_apg;

// variables, read from ros params
std::string sub_topic;
std::string local_tf_prefix;
int num_sectors;
double max_range;

// store subscribed persons messages globally
spencer_tracking_msgs::TrackedPersons tracked_persons;

spencer_tracking_msgs::TrajectoryPredictionLocalVelocity computeLocalVelocity(
    const spencer_tracking_msgs::TrackedPerson& tracked_person,
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
  tf::Quaternion q_person_to_world;
  tf::quaternionMsgToTF(tracked_person.pose.pose.orientation , q_person_to_world);
  tf::Matrix3x3 rotation_matrix(q_person_to_world);

  // Create tf vector from geometry_msgs vector
  tf::Vector3 twist_global;
  tf::vector3MsgToTF(tracked_person.twist.twist.linear, twist_global);

  // compute local twist in person tf
  tf::Vector3 twist_local;
  twist_local = rotation_matrix.inverse() * twist_global;
  tf::vector3TFToMsg(twist_local, local_velocity.localtwist.twist.linear);

  return local_velocity;
}

// For every detected person, check how far away all the other persons are and find their location relative to the query person
// Store this information in a polar coordinate system grid
void angularPedestrianGrid(tf::TransformListener& tf_listener)
{
  double sector_size = 2 * PI / num_sectors;
  if(tracked_persons.tracks.size() > 0)
  {
    spencer_tracking_msgs::TrajectoryPredictionAPGs apgs;

    for(int i = 0; i < tracked_persons.tracks.size(); i++)
    {
      spencer_tracking_msgs::TrajectoryPredictionAPG apg;

      // fill header for each person's APG
      std::ostringstream local_query_tf;
      local_query_tf << local_tf_prefix << tracked_persons.tracks[i].track_id;
      apg.header = tracked_persons.header;
      apg.header.frame_id = local_query_tf.str();

      // fill array with max_range
      for(int idx = 0; idx < num_sectors; idx++)
      {
        apg.min_distances.push_back(max_range);
      }

      // lookup transform between global and local person frame and compute rotation matrix
      tf::StampedTransform tf_world_to_person;
      tf_listener.waitForTransform(local_query_tf.str(), "odom", ros::Time(0), ros::Duration(0.5));
      tf_listener.lookupTransform(local_query_tf.str(), "odom", ros::Time(0), tf_world_to_person);
      tf::Matrix3x3 rotation_matrix_world_to_person(tf_world_to_person.getRotation());

      // compute the distance between the query pedestrian and every other pedestrian
      for(int j = 0; j < tracked_persons.tracks.size(); j++)
      {
        if(j != i)
        {
          double abs_distance = sqrt(pow(tracked_persons.tracks[i].pose.pose.position.x - tracked_persons.tracks[j].pose.pose.position.x, 2) + 
            pow(tracked_persons.tracks[i].pose.pose.position.y - tracked_persons.tracks[j].pose.pose.position.y, 2));
          
          if(abs_distance < max_range)
          {
            // change data type to tf::Vector3
            tf::Vector3 global_position;
            tf::pointMsgToTF(tracked_persons.tracks[i].pose.pose.position, global_position);

            // Transform the global position of a pedestrian into the coordinate system of the query pedestrian
            tf::Vector3 local_position = rotation_matrix_world_to_person * global_position;

            // find the person's angle relative to the local coordinate frame (polar coordinates)
            double angle = atan2(local_position.getY(), local_position.getX());
            int sector_nr = std::floor(angle / sector_size);
            apg.min_distances[sector_nr] = abs_distance;
          }
        }
      }
      apgs.apg_array.push_back(apg);
    }
    pub_apg.publish(apgs);
  }
}


void pubTfAndLocalVelocityForTrackedPersons()
{
  if(tracked_persons.tracks.size() > 0)
  {
    spencer_tracking_msgs::TrajectoryPredictionsLocalVelocities local_velocities;    
    static tf::TransformBroadcaster tf_broadcaster;

    foreach(const spencer_tracking_msgs::TrackedPerson& tracked_person, tracked_persons.tracks)
    {
      std::ostringstream local_tf;
      tf::Transform transform;
      tf::Vector3 tf_person_position;
      tf::Quaternion q_person_to_world;

      local_tf << local_tf_prefix << tracked_person.track_id;
      tf::pointMsgToTF(tracked_person.pose.pose.position, tf_person_position);
      tf::quaternionMsgToTF(tracked_person.pose.pose.orientation, q_person_to_world);
      transform.setOrigin(tf_person_position);
      transform.setRotation(q_person_to_world);
      tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tracked_persons.header.frame_id, local_tf.str()));

      // compute local velocities, store them together with position and frame information in message and push_back into message array
      local_velocities.tracks.push_back(computeLocalVelocity(tracked_person, local_tf.str()));
    }

    pub_local_velocities.publish(local_velocities);
  }
}


void trackedPersonsCallback(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg)
{
  tracked_persons = *msg;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracked_and_confirmed_persons_tf");
  ros::NodeHandle n;
  tf::TransformListener tf_listener;

  //   load params, globally defined
  ros::param::param<std::string>("yolo_confirmed_tracks_topic", sub_topic, "/spencer/perception/tracked_persons_confirmed_by_yolo");
  ros::param::param<std::string>("local_tf_prefix", local_tf_prefix, "person_");
  ros::param::param<int>("num_sectors", num_sectors, 72);
  ros::param::param<double>("max_range", max_range, 6.0);

  pub_local_velocities = n.advertise<spencer_tracking_msgs::TrajectoryPredictionsLocalVelocities>("output_local_velocities", 1);
  pub_apg = n.advertise<spencer_tracking_msgs::TrajectoryPredictionAPGs>("output_apg", 1);
  ros::Subscriber sub = n.subscribe(sub_topic, 1, trackedPersonsCallback);

  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
    ros::spinOnce();

    pubTfAndLocalVelocityForTrackedPersons();
    angularPedestrianGrid(tf_listener);

    loop_rate.sleep();
  }

  return 0;
}
