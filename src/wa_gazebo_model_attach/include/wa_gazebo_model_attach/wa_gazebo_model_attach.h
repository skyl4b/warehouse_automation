// Header for the model attacher plugin for Gazebo
//
// A simple plugin to add a service to Gazebo that allows models to be attached
// to each other. This is useful for creating simple movement in Gazebo,
// such as when a robot must move / attach to a box.
//
// Based on: https://github.com/Boeing/gazebo_model_attachment_plugin
// License: Apache License 2.0

#ifndef WA_GAZEBO_MODEL_ATTACH_H
#define WA_GAZEBO_MODEL_ATTACH_H

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <wa_interfaces/srv/attach.hpp>
#include <wa_interfaces/srv/detach.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gazebo_ros/node.hpp"

namespace gazebo
{

  class ModelAttachmentPlugin : public WorldPlugin
  {
  public:
    ModelAttachmentPlugin();

    virtual ~ModelAttachmentPlugin();

  protected:
    void Load(physics::WorldPtr world, sdf::ElementPtr sdf);

  private:
    physics::WorldPtr world_;

    bool attachCallback(const std::shared_ptr<wa_interfaces::srv::Attach::Request> req,
                        std::shared_ptr<wa_interfaces::srv::Attach::Response> res);
    bool detachCallback(const std::shared_ptr<wa_interfaces::srv::Detach::Request> req,
                        std::shared_ptr<wa_interfaces::srv::Detach::Response> res);

    void attach(const std::string &joint_name, physics::ModelPtr m1, physics::ModelPtr m2, physics::LinkPtr l1,
                physics::LinkPtr l2);
    void detach(const std::string &joint_name, physics::ModelPtr m1, physics::ModelPtr m2);

    gazebo_ros::Node::SharedPtr node_;

    rclcpp::Service<wa_interfaces::srv::Attach>::SharedPtr attach_srv_;
    rclcpp::Service<wa_interfaces::srv::Detach>::SharedPtr detach_srv_;
  };
} // namespace gazebo

#endif
