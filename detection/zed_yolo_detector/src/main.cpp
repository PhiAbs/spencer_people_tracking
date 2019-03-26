#include "ros/ros.h"
#include <spencer_tracking_msgs/DetectedPersons.h>
#include <spencer_tracking_msgs/DetectedPerson.h>

spencer_tracking_msgs::DetectedPersons detected_persons;
const double LARGE_VARIANCE = 999999999;
double pose_variance;
int detection_id_increment, detection_id_offset, current_detection_id; // added for multi-sensor use in SPENCER

void personCallback(const spencer_tracking_msgs::DetectedPersons::ConstPtr& msg)
{
    detected_persons = *msg;
}


void addAdditionalInformation()
{
  int num_detections = detected_persons.detections.size();

  if(num_detections > 0)
  {
    for(int i = 0; i < num_detections; i++)
    {
        detected_persons.detections[i].pose.covariance[0*6 + 0] = pose_variance;
        detected_persons.detections[i].pose.covariance[1*6 + 1] = pose_variance;
        detected_persons.detections[i].pose.covariance[2*6 + 2] = pose_variance;
        detected_persons.detections[i].pose.covariance[3*6 + 3] = LARGE_VARIANCE;
        detected_persons.detections[i].pose.covariance[4*6 + 4] = LARGE_VARIANCE;
        detected_persons.detections[i].pose.covariance[5*6 + 5] = LARGE_VARIANCE;

        detected_persons.detections[i].detection_id = current_detection_id;
        current_detection_id += detection_id_increment;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spencer_yolo_listener");
  ros::NodeHandle n;

  //   load params
  std::string pub_topic;
  std::string sub_topic;
  n.param("pose_variance", pose_variance, 0.05);
  n.param("detected_persons", pub_topic, std::string("/spencer/perception_internal/detected_persons/rgbd_front_top/yolo"));
  n.param("person_listener", sub_topic, std::string("/zed_yolo_detected_persons"));
  n.param("detection_id_increment", detection_id_increment, 1);
  n.param("detection_id_offset",    detection_id_offset, 0);

  current_detection_id = detection_id_offset;

  ros::Subscriber sub = n.subscribe(sub_topic, 10, personCallback);
  ros::Publisher pub = n.advertise<spencer_tracking_msgs::DetectedPersons>(pub_topic, 10);

  ros::Rate loop_rate(40);
  
  while (ros::ok())
  {
    addAdditionalInformation();
    detected_persons.header.stamp = ros::Time::now();
    pub.publish(detected_persons);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
