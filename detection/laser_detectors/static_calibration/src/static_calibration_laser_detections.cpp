
// Subscribe to the topic on which information about detected persons is published (YOLO detections)

#include "ros/ros.h"
#include <spencer_tracking_msgs/DetectedPersons.h>
#include <spencer_tracking_msgs/DetectedPerson.h>

class StaticCalibration
{
  public:

    // Constructor
    StaticCalibration()
    {
      // load params
      n.param("detected_persons", sub_topic, std::string("/detected_persons"));
      n.param("detected_persons_high_recall", sub_topic_high_recall, std::string("/detected_persons_high_recall"));
      n.param("static_calibration_offset_x", static_calibration_offset_x_, 0.0);

      sub = n.subscribe(sub_topic, 1, &StaticCalibration::detectedPersonCallback, this);
      sub_high_recall = n.subscribe(sub_topic_high_recall, 1, &StaticCalibration::detectedPersonHighRecallCallback, this);

      pub = n.advertise<spencer_tracking_msgs::DetectedPersons>("output", 1);
      pub_high_recall = n.advertise<spencer_tracking_msgs::DetectedPersons>("output_high_recall", 1);
    }

    void detectedPersonCallback(const spencer_tracking_msgs::DetectedPersons::ConstPtr& msg)
    {
      spencer_tracking_msgs::DetectedPersons calibrated_msg = *msg;
      
      // Add the static offset to each detection
      for(int i=0; i < calibrated_msg.detections.size(); i++)
      {
        calibrated_msg.detections[i].pose.pose.position.x += static_calibration_offset_x_;
      }
      pub.publish(calibrated_msg);
    }

    void detectedPersonHighRecallCallback(const spencer_tracking_msgs::DetectedPersons::ConstPtr& msg)
    {
      spencer_tracking_msgs::DetectedPersons calibrated_msg = *msg;

      // Add the static offset to each detection
      for(int i=0; i < calibrated_msg.detections.size(); i++)
      {
        calibrated_msg.detections[i].pose.pose.position.x += static_calibration_offset_x_;
      }
      pub_high_recall.publish(calibrated_msg);
    }

  private:
    ros::NodeHandle n;

    ros::Subscriber sub;
    ros::Subscriber sub_high_recall;
    ros::Publisher pub;
    ros::Publisher pub_high_recall;

    std::string sub_topic;
    std::string sub_topic_high_recall;
    double static_calibration_offset_x_;
}; // End of class StaticCalibration



int main(int argc, char **argv)
{
  ros::init(argc, argv, "spencer_yolo_listener");
  ros::Rate loop_rate(20);

  StaticCalibration static_calibration;
  
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
