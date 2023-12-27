#ifndef PRIUSCUP_PLUGINS_PRIUSHYBRIDPLUGIN_HH_
#define PRIUSCUP_PLUGINS_PRIUSHYBRIDPLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>


#include <prius_msgs/msg/control.hpp>

#include <memory>

namespace gazebo_plugins
{
// Forward declaration of private data class.
class PriusHybridPluginPrivate;

/// Example ROS-powered Gazebo plugin with some useful boilerplate.
/// \details This is a `ModelPlugin`, but it could be any supported Gazebo plugin type, such as
/// System, Visual, GUI, World, Sensor, etc.
class PriusHybridPlugin : public gazebo::ModelPlugin
{
public:
  /// Constructor
  PriusHybridPlugin();

  /// Destructor
  virtual ~PriusHybridPlugin();

  /// Gazebo calls this when the plugin is loaded.
  /// \param[in] model Pointer to parent model. Other plugin types will expose different entities,
  /// such as `gazebo::sensors::SensorPtr`, `gazebo::physics::WorldPtr`,
  /// `gazebo::rendering::VisualPtr`, etc.
  /// \param[in] sdf SDF element containing user-defined parameters.
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;


  void OnPriusCommand(const prius_msgs::msg::Control::SharedPtr msg);
  void OnReset(const ignition::msgs::Any & /*_msg*/);
  void OnStop(const ignition::msgs::Any & /*_msg*/);

  void OnCmdVel(const ignition::msgs::Pose &_msg);
  void OnCmdGear(const ignition::msgs::Int32 &_msg);
  void OnCmdMode(const ignition::msgs::Boolean &/*_msg*/);

  void UpdateHandWheelRatio();
  double CollisionRadius(gazebo::physics::CollisionPtr _coll);
  void OnKeyPress(ConstAnyPtr &_msg);
  void OnKeyPressIgn(const ignition::msgs::Any &_msg);
  void KeyControlTypeA(const int _key);
  void KeyControlTypeB(const int _key);
  void KeyControl(const int _key);
  double GasTorqueMultiplier();
  
protected:
  /// Optional callback to be called at every simulation iteration.
  virtual void OnUpdate();

private:
  /// Recommended PIMPL pattern. This variable should hold all private
  /// data members.
  std::unique_ptr<PriusHybridPluginPrivate> impl_;

  std::string robot_namespace_;  
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_TEMPLATE_HPP_