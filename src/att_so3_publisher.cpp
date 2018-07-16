/**
 * @file
 * @date July 2018
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Node for adaptive IMU attitude estimation on SO3.
 */


#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dscl_msgs/Imu9DOF.h> 
#include <att_so3/so3_att.h>
#include <helper_funcs/helper_funcs.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <iostream>   



/**
 *
 * @brief Class for launching MEMS bias estimation rosnode.
 * 
 */
class AttNode
{

private:
  
  ros::Publisher chatter_; /**< Quaternion publisher. */
  ros::Publisher chatter_rpy_; /**< RPY publisher. */

  SO3Att* att_;
  
public:

  AttParams params;

  /**
   *
   * @brief Constructor.
   * 
   * @param n ROS NodeHandle
   * 
   */
  AttNode(ros::NodeHandle n){


    ROS_INFO("Loading estimator params.");

    std::string k_g;
    std::string k_north;
    std::string r0;
    std::string lat;

    Eigen::Vector3d rpy;

    n.param<std::string>("k_g",k_g, "1,1,1");
    n.param<std::string>("k_north",k_north, "1,1,1");
    n.param<std::string>("r0",r0,"0,0,0");
    n.param<std::string>("lat",lat,"39.32");
    n.param<std::string>("frame_id",params.frameId,"imu");
    
    sscanf(r0.c_str(),"%lf,%lf,%lf",&rpy(0),&rpy(1),&rpy(2));

    params.K_g          = stringToDiag(k_g);
    params.K_north      = stringToDiag(k_north);
    params.R0           = rpy2rot(rpy);
    params.lat          = std::stod(lat);
    
    chatter_ = n.advertise<geometry_msgs::QuaternionStamped>("att",1);
    chatter_rpy_ = n.advertise<geometry_msgs::Vector3Stamped>("rpy",1);


    att_ = new SO3Att(params);

  }

  /**
   *
   * @brief Publishing callback function.
   * 
   */
  void callback(const dscl_msgs::Imu9DOF::ConstPtr& msg)
  {

    ImuPacket measurement;

    measurement.acc << msg->acc.x,msg->acc.y,msg->acc.z;
    measurement.ang << msg->ang.x,msg->ang.y,msg->ang.z;
    measurement.mag << msg->mag.x,msg->mag.y,msg->mag.z;
    
    measurement.t = msg->header.stamp.toSec();

    att_->step(measurement);

    Eigen::Quaterniond q(att_->R_ni);
    
    // initialize mems_bias msg
    geometry_msgs::QuaternionStamped att;
    geometry_msgs::Vector3Stamped rpy;

    Eigen::Vector3d rph = rot2rph(att_->R_ni)*180.0/M_PI;

    rpy.header.stamp    = msg->header.stamp;
    rpy.header.frame_id = params.frameId;
      
    rpy.vector.x = rph(0);
    rpy.vector.y = rph(1);
    rpy.vector.z = rph(2);

    att.header.stamp    = msg->header.stamp;
    att.header.frame_id = params.frameId;
    
    att.quaternion.x = q.x();
    att.quaternion.y = q.y();
    att.quaternion.z = q.z();
    att.quaternion.w = q.w(); 

    // publish packet
    chatter_.publish(att);
    chatter_rpy_.publish(rpy);
    
  }
 
};

int main(int argc, char **argv)
{

    // initialize node
    ros::init(argc, argv, "att");

    ros::NodeHandle n("~");

    ros::Subscriber sub; /**< Node subscriber to imu topic. */

 
    /**********************************************************************
     * Initialize Bias Estimator
     **********************************************************************/
    AttNode att_est(n);

    char buffer[64];

    sprintf(buffer,"/%s_bias/imu_corrected",att_est.params.frameId.c_str());

    sub = n.subscribe(buffer,1,&AttNode::callback, &att_est);
    
    ros::spin();
    
    return 0;
}
