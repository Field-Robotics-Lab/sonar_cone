/*
 * Desc: NPS Ros Sonar controller.
 * Author: Bruce Allen
 * Date: 01 May 2020
 */

#ifndef NPS_GAZEBO_ROS_SONAR_H
#define NPS_GAZEBO_ROS_SONAR_H

// Custom Callback Queue
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/SonarPlugin.hh>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <sensor_msgs/Range.h>

namespace gazebo
{

  class NpsGazeboRosSonar: public SonarPlugin
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: NpsGazeboRosSonar();

    /// \brief Destructor
    public: ~NpsGazeboRosSonar();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    private: void OnScan(ConstSonarStampedPtr&_msg);

    /// \brief Put sonar data to the ROS topic
    private: void PutSonarData(common::Time &_updateTime);

    private: common::Time last_update_time_;

    /// \brief Keep track of number of connctions
    private: int sonar_connect_count_;
    private: void SonarConnect();
    private: void SonarDisconnect();

    // \brief Pointer to the model
    private: physics::WorldPtr world_;

    /// \brief The parent sensor
    private: sensors::SensorPtr parent_sensor_;
    private: sensors::SonarSensorPtr parent_sonar_sensor_;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;

    /// \brief ros message
    private: sensor_msgs::Range range_msg_;

    /// \brief Gazebo subscription
    private: gazebo::transport::SubscriberPtr sonar_scan_sub_;

    /// \brief topic name
    private: std::string topic_name_;

    /// \brief frame transform name, should match link name
    private: std::string frame_name_;

    /// \brief A mutex to lock access to fields that are used in message callbacks
    private: boost::mutex lock;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    // Custom Callback Queue
    private: ros::CallbackQueue range_queue_;
    private: void SonarQueueThread();
    private: boost::thread callback_range_queue_thread_;

    // subscribe to world stats
    private: transport::NodePtr gazebo_node_;
    private: common::Time sim_time_;
    public: void OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg);

  };

}

#endif

