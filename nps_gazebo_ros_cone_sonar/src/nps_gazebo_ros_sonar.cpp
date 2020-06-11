/*
 * Desc: NPS Ros Sonar controller.
 * Author: Bruce Allen
 * Date: 01 May 2020
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_utils.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/SonarSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/MessageTypes.hh>

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>

#include <tf/tf.h>

#include "gazebo_plugins/nps_gazebo_ros_sonar.h"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(NpsGazeboRosSonar)

////////////////////////////////////////////////////////////////////////////////
// Constructor
NpsGazeboRosSonar::NpsGazeboRosSonar()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
NpsGazeboRosSonar::~NpsGazeboRosSonar()
{
  ////////////////////////////////////////////////////////////////////////////////
  // Finalize the controller / Custom Callback Queue
  this->range_queue_.clear();
  this->range_queue_.disable();
  this->rosnode_->shutdown();
  this->callback_range_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void NpsGazeboRosSonar::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  SonarPlugin::Load(_parent, _sdf);

  // Get then name of the parent sensor
  this->parent_sensor_ = _parent;

  // Get the world name.
  std::string worldName = _parent->WorldName();
  this->world_ = physics::get_world(worldName);

#if GAZEBO_MAJOR_VERSION >= 8
  last_update_time_ = this->world_->SimTime();
#else
  last_update_time_ = this->world_->GetSimTime();
#endif

  this->gazebo_node_ = transport::NodePtr(new transport::Node());
  this->gazebo_node_->Init(worldName);

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_sonar_sensor_ = dynamic_pointer_cast<sensors::SonarSensor>(this->parent_sensor_);

  if (!this->parent_sonar_sensor_)
    gzthrow("NpsGazeboRosSonar controller requires a Sonar Sensor as its parent");

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO_NAMED("sonar", "Sonar plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

  if (!_sdf->HasElement("topicName"))
  {
    ROS_INFO_NAMED("sonar", "Sonar plugin missing <topicName>, defaults to /range");
    this->topic_name_ = "/range";
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  this->sonar_connect_count_ = 0;

  // persistent fields
  this->range_msg_.header.frame_id = this->frame_name_;
  // see http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html
  this->range_msg_.radiation_type = sensor_msgs::Range::ULTRASOUND;

  this->range_msg_.max_range = this->parent_sonar_sensor_->RangeMax();
  this->range_msg_.min_range = this->parent_sonar_sensor_->RangeMin();
  this->range_msg_.field_of_view = 0.0;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("sonar", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  if (this->topic_name_ != "")
  {
    // Custom Callback Queue
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::Range>(
      this->topic_name_,1,
      boost::bind( &NpsGazeboRosSonar::SonarConnect,this),
      boost::bind( &NpsGazeboRosSonar::SonarDisconnect,this), ros::VoidPtr(), &this->range_queue_);
    this->pub_ = this->rosnode_->advertise(ao);
  }


  // Initialize the controller

  // sensor generation off by default
  this->parent_sonar_sensor_->SetActive(false);
  // start custom queue for sonar
  this->callback_range_queue_thread_ = boost::thread( boost::bind( &NpsGazeboRosSonar::SonarQueueThread,this ) );
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void NpsGazeboRosSonar::SonarConnect()
{
  this->sonar_connect_count_++;

  if (this->sonar_connect_count_ == 1)
    this->sonar_scan_sub_ =
      this->gazebo_node_->Subscribe(this->parent_sonar_sensor_->Topic(),
                                    &NpsGazeboRosSonar::OnScan, this);
  this->parent_sonar_sensor_->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void NpsGazeboRosSonar::SonarDisconnect()
{
  this->sonar_connect_count_--;

  if (this->sonar_connect_count_ == 0)
  {
    this->parent_sonar_sensor_->SetActive(false);
    this->sonar_scan_sub_.reset();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void NpsGazeboRosSonar::OnScan(ConstSonarStampedPtr &_msg)
{
  if (this->topic_name_ != "")
  {
    common::Time sensor_update_time = this->parent_sensor_->LastUpdateTime();
    if (sensor_update_time < last_update_time_)
    {
        ROS_WARN_NAMED("sonar", "Negative sensor update time difference detected.");
        last_update_time_ = sensor_update_time;
    }

    if (last_update_time_ < sensor_update_time)
    {
      this->PutSonarData(sensor_update_time);
      last_update_time_ = sensor_update_time;
    }
  }
  else
  {
    ROS_INFO_NAMED("sonar", "nps_gazebo_ros_sonar topic name not set");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put sonar data to the interface
void NpsGazeboRosSonar::PutSonarData(common::Time &_updateTime)
{
  this->parent_sonar_sensor_->SetActive(false);

  // perform sonar scan from Gazebo Sonar sensor
  {
    boost::mutex::scoped_lock sclock(this->lock);

    // Add Frame Name
    this->range_msg_.header.frame_id = this->frame_name_;
    this->range_msg_.header.stamp.sec = _updateTime.sec;
    this->range_msg_.header.stamp.nsec = _updateTime.nsec;

    // range
    double range = this->parent_sonar_sensor_->Range();
    this->range_msg_.range = range;

    this->parent_sonar_sensor_->SetActive(true);

    // send data out via ros message
    if (this->sonar_connect_count_ > 0)
        this->pub_.publish(this->range_msg_);
  }
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void NpsGazeboRosSonar::SonarQueueThread()
{
  static const double timeout = 2.0;
//  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->range_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void NpsGazeboRosSonar::OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg)
{
  this->sim_time_  = msgs::Convert( _msg->sim_time() );

  ignition::math::Pose3d pose;
  pose.Pos().X() = 0.5*sin(0.01*this->sim_time_.Double());
  gzdbg << "plugin simTime [" << this->sim_time_.Double() << "] update pose [" << pose.Pos().X() << "]\n";
}
}

