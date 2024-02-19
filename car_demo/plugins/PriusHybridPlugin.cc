#include <gazebo/physics/Model.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ignition/transport.hh>

#include <memory>

#include "PriusHybridPlugin.hh"

namespace gazebo_plugins
{
  /// Class to hold private data members (PIMPL pattern)
  class PriusHybridPluginPrivate
  {
  public:
    /// \enum DirectionType
    /// \brief Direction selector switch type.
    enum DirectionType
    {
      /// \brief Reverse
      REVERSE = -1,
      /// \brief Neutral
      NEUTRAL = 0,
      /// \brief Forward
      FORWARD = 1
    };

    /// Connection to world update event. Callback is called while this is alive.
    gazebo::event::ConnectionPtr update_connection_;

    /// Node for ROS communication.
    gazebo_ros::Node::SharedPtr ros_node_;

    // public: ros::Subscriber controlSub;
    rclcpp::Subscription<prius_msgs::msg::Control>::SharedPtr controlSub_;

    /// \brief Pointer to the world
    gazebo::physics::WorldPtr world;

    /// \brief Pointer to the parent model
    gazebo::physics::ModelPtr model;

    /// \brief Transport node
    gazebo::transport::NodePtr gznode;

    /// \brief Ignition transport node
    ignition::transport::Node node;

    /// \brief Ignition transport position pub
    ignition::transport::Node::Publisher posePub;

    /// \brief Ignition transport console pub
    ignition::transport::Node::Publisher consolePub;

    /// \brief Physics update event connection
    gazebo::event::ConnectionPtr updateConnection;

    /// \brief Chassis link
    gazebo::physics::LinkPtr chassisLink;

    /// \brief Front left wheel joint
    gazebo::physics::JointPtr flWheelJoint;

    /// \brief Front right wheel joint
    gazebo::physics::JointPtr frWheelJoint;

    /// \brief Rear left wheel joint
    gazebo::physics::JointPtr blWheelJoint;

    /// \brief Rear right wheel joint
    gazebo::physics::JointPtr brWheelJoint;

    /// \brief Front left wheel steering joint
    gazebo::physics::JointPtr flWheelSteeringJoint;

    /// \brief Front right wheel steering joint
    gazebo::physics::JointPtr frWheelSteeringJoint;

    /// \brief Steering wheel joint
    gazebo::physics::JointPtr handWheelJoint;

    /// \brief PID control for the front left wheel steering joint
    gazebo::common::PID flWheelSteeringPID;

    /// \brief PID control for the front right wheel steering joint
    gazebo::common::PID frWheelSteeringPID;

    /// \brief PID control for steering wheel joint
    gazebo::common::PID handWheelPID;

    /// \brief Last pose msg time
    gazebo::common::Time lastMsgTime;

    /// \brief Last sim time received
    gazebo::common::Time lastSimTime;

    /// \brief Last sim time when a pedal command is received
    gazebo::common::Time lastPedalCmdTime;

    /// \brief Last sim time when a steering command is received
    gazebo::common::Time lastSteeringCmdTime;

    /// \brief Last sim time when a EV mode command is received
    gazebo::common::Time lastModeCmdTime;

    /// \brief Current direction of the vehicle: FORWARD, NEUTRAL, REVERSE.
    DirectionType directionState;

    /// \brief Chassis aerodynamic drag force coefficient,
    /// with units of [N / (m/s)^2]
    double chassisAeroForceGain = 0;

    /// \brief Max torque that can be applied to the front wheels
    double frontTorque = 0;

    /// \brief Max torque that can be applied to the back wheels
    double backTorque = 0;

    /// \brief Max speed (m/s) of the car
    double maxSpeed = 0;

    /// \brief Max steering angle
    double maxSteer = 0;

    /// \brief Max torque that can be applied to the front brakes
    double frontBrakeTorque = 0;

    /// \brief Max torque that can be applied to the rear brakes
    double backBrakeTorque = 0;

    /// \brief Angle ratio between the steering wheel and the front wheels
    double steeringRatio = 0;

    /// \brief Max range of hand steering wheel
    double handWheelHigh = 0;

    /// \brief Min range of hand steering wheel
    double handWheelLow = 0;

    /// \brief Front left wheel desired steering angle (radians)
    double flWheelSteeringCmd = 0;

    /// \brief Front right wheel desired steering angle (radians)
    double frWheelSteeringCmd = 0;

    /// \brief Steering wheel desired angle (radians)
    double handWheelCmd = 0;

    /// \brief Front left wheel radius
    double flWheelRadius = 0;

    /// \brief Front right wheel radius
    double frWheelRadius = 0;

    /// \brief Rear left wheel radius
    double blWheelRadius = 0;

    /// \brief Rear right wheel radius
    double brWheelRadius = 0;

    /// \brief Front left joint friction
    double flJointFriction = 0;

    /// \brief Front right joint friction
    double frJointFriction = 0;

    /// \brief Rear left joint friction
    double blJointFriction = 0;

    /// \brief Rear right joint friction
    double brJointFriction = 0;

    /// \brief Distance distance between front and rear axles
    double wheelbaseLength = 0;

    /// \brief Distance distance between front left and right wheels
    double frontTrackWidth = 0;

    /// \brief Distance distance between rear left and right wheels
    double backTrackWidth = 0;

    /// \brief Gas energy density (J/gallon)
    const double kGasEnergyDensity = 1.29e8;

    /// \brief Battery charge capacity in Watt-hours
    double batteryChargeWattHours = 280;

    /// \brief Battery discharge capacity in Watt-hours
    double batteryDischargeWattHours = 260;

    /// \brief Gas engine efficiency
    double gasEfficiency = 0.37;

    /// \brief Minimum gas flow rate (gallons / sec)
    double minGasFlow = 1e-4;

    /// \brief Gas consumption (gallon)
    double gasConsumption = 0;

    /// \brief Battery state-of-charge (percent, 0.0 - 1.0)
    double batteryCharge = 0.75;

    /// \brief Battery charge threshold when it has to be recharged.
    const double batteryLowThreshold = 0.125;

    /// \brief Whether EV mode is on or off.
    bool evMode = false;

    /// \brief Gas pedal position in percentage. 1.0 = Fully accelerated.
    double gasPedalPercent = 0;

    /// \brief Power for charging a low battery (Watts).
    const double kLowBatteryChargePower = 2000;

    /// \brief Threshold delimiting the gas pedal (throttle) low and medium
    /// ranges.
    const double kGasPedalLowMedium = 0.25;

    /// \brief Threshold delimiting the gas pedal (throttle) medium and high
    /// ranges.
    const double kGasPedalMediumHigh = 0.5;

    /// \brief Threshold delimiting the speed (throttle) low and medium
    /// ranges in miles/h.
    const double speedLowMedium = 26.0;

    /// \brief Threshold delimiting the speed (throttle) medium and high
    /// ranges in miles/h.
    const double speedMediumHigh = 46.0;

    /// \brief Brake pedal position in percentage. 1.0 =
    double brakePedalPercent = 0;

    /// \brief Hand brake position in percentage.
    double handbrakePercent = 1.0;

    /// \brief Angle of steering wheel at last update (radians)
    double handWheelAngle = 0;

    /// \brief Steering angle of front left wheel at last update (radians)
    double flSteeringAngle = 0;

    /// \brief Steering angle of front right wheel at last update (radians)
    double frSteeringAngle = 0;

    /// \brief Linear velocity of chassis c.g. in world frame at last update (m/s)
    ignition::math::Vector3d chassisLinearVelocity;

    /// \brief Angular velocity of front left wheel at last update (rad/s)
    double flWheelAngularVelocity = 0;

    /// \brief Angular velocity of front right wheel at last update (rad/s)
    double frWheelAngularVelocity = 0;

    /// \brief Angular velocity of back left wheel at last update (rad/s)
    double blWheelAngularVelocity = 0;

    /// \brief Angular velocity of back right wheel at last update (rad/s)
    double brWheelAngularVelocity = 0;

    /// \brief Subscriber to the keyboard topic
    gazebo::transport::SubscriberPtr keyboardSub;

    /// \brief Mutex to protect updates
    std::mutex mutex;

    /// \brief Odometer
    double odom = 0.0;

    /// \brief Keyboard control type
    int keyControl = 1;

    /// \brief Publisher for the world_control topic.
    gazebo::transport::PublisherPtr worldControlPub;
  };

  PriusHybridPlugin::PriusHybridPlugin()
      : impl_(std::make_unique<PriusHybridPluginPrivate>())
  {
    this->robot_namespace_ = "";
    this->impl_->directionState = gazebo_plugins::PriusHybridPluginPrivate::FORWARD;
    this->impl_->flWheelRadius = 0.3;
    this->impl_->frWheelRadius = 0.3;
    this->impl_->blWheelRadius = 0.3;
    this->impl_->brWheelRadius = 0.3;
  }

  void PriusHybridPlugin::OnPriusCommand(const prius_msgs::msg::Control::SharedPtr msg)
  {
    this->impl_->lastSteeringCmdTime = this->impl_->world->SimTime();
    this->impl_->lastPedalCmdTime = this->impl_->world->SimTime();

    // Steering wheel command
    double handCmd = (msg->steer < 0.)
                         ? (msg->steer * -this->impl_->handWheelLow)
                         : (msg->steer * this->impl_->handWheelHigh);

    handCmd = ignition::math::clamp(handCmd, this->impl_->handWheelLow,
                                    this->impl_->handWheelHigh);
    this->impl_->handWheelCmd = handCmd;

    // Brake command
    this->impl_->brakePedalPercent = ignition::math::clamp(msg->brake, 0.0, 1.0);

    // Throttle command
    this->impl_->gasPedalPercent = ignition::math::clamp(msg->throttle, 0.0, 1.0);

    switch (msg->shift_gears)
    {
    case prius_msgs::msg::Control::NEUTRAL:
      this->impl_->directionState = PriusHybridPluginPrivate::NEUTRAL;
      break;
    case prius_msgs::msg::Control::FORWARD:
      this->impl_->directionState = PriusHybridPluginPrivate::FORWARD;
      break;
    case prius_msgs::msg::Control::REVERSE:
      this->impl_->directionState = PriusHybridPluginPrivate::REVERSE;
      break;
    default:
      break;
    }
  }

  PriusHybridPlugin::~PriusHybridPlugin()
  {
    // this->impl_->update_connection_();
  }

  void PriusHybridPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Create a GazeboRos node instead of a common ROS node.
    // Pass it SDF parameters so common options like namespace and remapping
    // can be handled.
    this->impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
    const gazebo_ros::QoS &qos = this->impl_->ros_node_->get_qos();

    // The model pointer gives you direct access to the physics object,
    // for example:
    RCLCPP_INFO(this->impl_->ros_node_->get_logger(), "Starting Plugin [%s]", _model->GetName().c_str());
    // RCLCPP_WARN(impl_->ros_node_->get_logger(), _model->GetName().c_str());

    this->impl_->model = _model;
    this->impl_->world = this->impl_->model->GetWorld();
    auto physicsEngine = this->impl_->world->Physics();
    physicsEngine->SetParam("friction_model", std::string("cone_model"));

    // Gazebo internal Node to subscribe to keyboard
    this->impl_->gznode = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->impl_->gznode->Init();

    if (_sdf->HasElement("robot_namespace"))
      this->robot_namespace_ = _sdf->GetElement("robot_namespace")->Get<std::string>() + "/";
    this->impl_->controlSub_ = this->impl_->ros_node_->create_subscription<prius_msgs::msg::Control>(
        "prius", qos.get_subscription_qos("prius", rclcpp::QoS(1)),
        std::bind(&PriusHybridPlugin::OnPriusCommand, this, std::placeholders::_1));
    RCLCPP_INFO(
        this->impl_->ros_node_->get_logger(), "Subscribed to [%s]",
        this->impl_->controlSub_->get_topic_name());

    this->impl_->node.Subscribe("/prius/reset",
                                &PriusHybridPlugin::OnReset, this);
    this->impl_->node.Subscribe("/prius/stop",
                                &PriusHybridPlugin::OnStop, this);

    this->impl_->node.Subscribe("/cmd_vel",
                                &PriusHybridPlugin::OnCmdVel, this);
    this->impl_->node.Subscribe("/cmd_gear",
                                &PriusHybridPlugin::OnCmdGear, this);
    this->impl_->node.Subscribe("/cmd_mode",
                                &PriusHybridPlugin::OnCmdMode, this);

    this->impl_->posePub = this->impl_->node.Advertise<ignition::msgs::Pose>(
        "/prius/pose");
    this->impl_->consolePub =
        this->impl_->node.Advertise<ignition::msgs::Double_V>("/prius/console");

    std::string chassisLinkName = this->impl_->model->GetName() + "::" + _sdf->Get<std::string>("chassis");
    this->impl_->chassisLink = this->impl_->model->GetLink(chassisLinkName);
    if (!this->impl_->chassisLink)
    {
      std::cerr << "could not find chassis link" << std::endl;
      return;
    }

    std::string handWheelJointName = this->impl_->model->GetName() + "::" + _sdf->Get<std::string>("steering_wheel");
    this->impl_->handWheelJoint =
        this->impl_->model->GetJoint(handWheelJointName);
    if (!this->impl_->handWheelJoint)
    {
      std::cerr << "could not find steering wheel joint" << std::endl;
      return;
    }

    std::string flWheelJointName = this->impl_->model->GetName() + "::" + _sdf->Get<std::string>("front_left_wheel");
    this->impl_->flWheelJoint =
        this->impl_->model->GetJoint(flWheelJointName);
    if (!this->impl_->flWheelJoint)
    {
      std::cerr << "could not find front left wheel joint" << std::endl;
      return;
    }

    std::string frWheelJointName = this->impl_->model->GetName() + "::" + _sdf->Get<std::string>("front_right_wheel");
    this->impl_->frWheelJoint =
        this->impl_->model->GetJoint(frWheelJointName);
    if (!this->impl_->frWheelJoint)
    {
      std::cerr << "could not find front right wheel joint" << std::endl;
      return;
    }

    std::string blWheelJointName = this->impl_->model->GetName() + "::" + _sdf->Get<std::string>("back_left_wheel");
    this->impl_->blWheelJoint =
        this->impl_->model->GetJoint(blWheelJointName);
    if (!this->impl_->blWheelJoint)
    {
      std::cerr << "could not find back left wheel joint" << std::endl;
      return;
    }

    std::string brWheelJointName = this->impl_->model->GetName() + "::" + _sdf->Get<std::string>("back_right_wheel");
    this->impl_->brWheelJoint =
        this->impl_->model->GetJoint(brWheelJointName);
    if (!this->impl_->brWheelJoint)
    {
      std::cerr << "could not find back right wheel joint" << std::endl;
      return;
    }

    std::string flWheelSteeringJointName = this->impl_->model->GetName() + "::" + _sdf->Get<std::string>("front_left_wheel_steering");
    this->impl_->flWheelSteeringJoint =
        this->impl_->model->GetJoint(flWheelSteeringJointName);
    if (!this->impl_->flWheelSteeringJoint)
    {
      std::cerr << "could not find front left steering joint" << std::endl;
      return;
    }

    std::string frWheelSteeringJointName = this->impl_->model->GetName() + "::" + _sdf->Get<std::string>("front_right_wheel_steering");
    this->impl_->frWheelSteeringJoint =
        this->impl_->model->GetJoint(frWheelSteeringJointName);
    if (!this->impl_->frWheelSteeringJoint)
    {
      std::cerr << "could not find front right steering joint" << std::endl;
      return;
    }

    std::string paramName;
    double paramDefault;

    paramName = "chassis_aero_force_gain";
    paramDefault = 1;
    if (_sdf->HasElement(paramName))
      this->impl_->chassisAeroForceGain = _sdf->Get<double>(paramName);
    else
      this->impl_->chassisAeroForceGain = paramDefault;

    paramName = "front_torque";
    paramDefault = 0;
    if (_sdf->HasElement(paramName))
      this->impl_->frontTorque = _sdf->Get<double>(paramName);
    else
      this->impl_->frontTorque = paramDefault;

    paramName = "back_torque";
    paramDefault = 2000;
    if (_sdf->HasElement(paramName))
      this->impl_->backTorque = _sdf->Get<double>(paramName);
    else
      this->impl_->backTorque = paramDefault;

    paramName = "front_brake_torque";
    paramDefault = 2000;
    if (_sdf->HasElement(paramName))
      this->impl_->frontBrakeTorque = _sdf->Get<double>(paramName);
    else
      this->impl_->frontBrakeTorque = paramDefault;

    paramName = "back_brake_torque";
    paramDefault = 2000;
    if (_sdf->HasElement(paramName))
      this->impl_->backBrakeTorque = _sdf->Get<double>(paramName);
    else
      this->impl_->backBrakeTorque = paramDefault;

    paramName = "battery_charge_watt_hours";
    paramDefault = 280;
    if (_sdf->HasElement(paramName))
      this->impl_->batteryChargeWattHours = _sdf->Get<double>(paramName);
    else
      this->impl_->batteryChargeWattHours = paramDefault;

    paramName = "battery_discharge_watt_hours";
    paramDefault = 260;
    if (_sdf->HasElement(paramName))
      this->impl_->batteryDischargeWattHours = _sdf->Get<double>(paramName);
    else
      this->impl_->batteryDischargeWattHours = paramDefault;

    paramName = "gas_efficiency";
    paramDefault = 0.37;
    if (_sdf->HasElement(paramName))
      this->impl_->gasEfficiency = _sdf->Get<double>(paramName);
    else
      this->impl_->gasEfficiency = paramDefault;

    paramName = "min_gas_flow";
    paramDefault = 1e-4;
    if (_sdf->HasElement(paramName))
      this->impl_->minGasFlow = _sdf->Get<double>(paramName);
    else
      this->impl_->minGasFlow = paramDefault;

    paramName = "max_speed";
    paramDefault = 10;
    if (_sdf->HasElement(paramName))
      this->impl_->maxSpeed = _sdf->Get<double>(paramName);
    else
      this->impl_->maxSpeed = paramDefault;

    paramName = "max_steer";
    paramDefault = 0.6;
    if (_sdf->HasElement(paramName))
      this->impl_->maxSteer = _sdf->Get<double>(paramName);
    else
      this->impl_->maxSteer = paramDefault;

    paramName = "flwheel_steering_p_gain";
    paramDefault = 0;
    if (_sdf->HasElement(paramName))
      this->impl_->flWheelSteeringPID.SetPGain(_sdf->Get<double>(paramName));
    else
      this->impl_->flWheelSteeringPID.SetPGain(paramDefault);

    paramName = "frwheel_steering_p_gain";
    paramDefault = 0;
    if (_sdf->HasElement(paramName))
      this->impl_->frWheelSteeringPID.SetPGain(_sdf->Get<double>(paramName));
    else
      this->impl_->frWheelSteeringPID.SetPGain(paramDefault);

    paramName = "flwheel_steering_i_gain";
    paramDefault = 0;
    if (_sdf->HasElement(paramName))
      this->impl_->flWheelSteeringPID.SetIGain(_sdf->Get<double>(paramName));
    else
      this->impl_->flWheelSteeringPID.SetIGain(paramDefault);

    paramName = "frwheel_steering_i_gain";
    paramDefault = 0;
    if (_sdf->HasElement(paramName))
      this->impl_->frWheelSteeringPID.SetIGain(_sdf->Get<double>(paramName));
    else
      this->impl_->frWheelSteeringPID.SetIGain(paramDefault);

    paramName = "flwheel_steering_d_gain";
    paramDefault = 0;
    if (_sdf->HasElement(paramName))
      this->impl_->flWheelSteeringPID.SetDGain(_sdf->Get<double>(paramName));
    else
      this->impl_->flWheelSteeringPID.SetDGain(paramDefault);

    paramName = "frwheel_steering_d_gain";
    paramDefault = 0;
    if (_sdf->HasElement(paramName))
      this->impl_->frWheelSteeringPID.SetDGain(_sdf->Get<double>(paramName));
    else
      this->impl_->frWheelSteeringPID.SetDGain(paramDefault);

    this->UpdateHandWheelRatio();

    // Update wheel radius for each wheel from SDF collision objects
    //  assumes that wheel link is child of joint (and not parent of joint)
    //  assumes that wheel link has only one collision
    unsigned int id = 0;
    this->impl_->flWheelRadius = this->CollisionRadius(
        this->impl_->flWheelJoint->GetChild()->GetCollision(id));
    this->impl_->frWheelRadius = this->CollisionRadius(
        this->impl_->frWheelJoint->GetChild()->GetCollision(id));
    this->impl_->blWheelRadius = this->CollisionRadius(
        this->impl_->blWheelJoint->GetChild()->GetCollision(id));
    this->impl_->brWheelRadius = this->CollisionRadius(
        this->impl_->brWheelJoint->GetChild()->GetCollision(id));

    // Get initial joint friction and add it to braking friction
    this->impl_->flJointFriction = this->impl_->flWheelJoint->GetParam("friction", 0);
    this->impl_->frJointFriction = this->impl_->frWheelJoint->GetParam("friction", 0);
    this->impl_->blJointFriction = this->impl_->blWheelJoint->GetParam("friction", 0);
    this->impl_->brJointFriction = this->impl_->brWheelJoint->GetParam("friction", 0);

    // Compute wheelbase, frontTrackWidth, and rearTrackWidth
    //  first compute the positions of the 4 wheel centers
    //  again assumes wheel link is child of joint and has only one collision
    ignition::math::Vector3d flCenterPos =
        this->impl_->flWheelJoint->GetChild()->GetCollision(id)->WorldPose().Pos();
    ignition::math::Vector3d frCenterPos =
        this->impl_->frWheelJoint->GetChild()->GetCollision(id)->WorldPose().Pos();
    ignition::math::Vector3d blCenterPos =
        this->impl_->blWheelJoint->GetChild()->GetCollision(id)->WorldPose().Pos();
    ignition::math::Vector3d brCenterPos =
        this->impl_->brWheelJoint->GetChild()->GetCollision(id)->WorldPose().Pos();

    // track widths are computed first
    ignition::math::Vector3d vec3 = flCenterPos - frCenterPos;
    this->impl_->frontTrackWidth = vec3.Length();
    vec3 = flCenterPos - frCenterPos;
    this->impl_->backTrackWidth = vec3.Length();
    // to compute wheelbase, first position of axle centers are computed
    ignition::math::Vector3d frontAxlePos = (flCenterPos + frCenterPos) / 2;
    ignition::math::Vector3d backAxlePos = (blCenterPos + brCenterPos) / 2;
    // then the wheelbase is the distance between the axle centers
    vec3 = frontAxlePos - backAxlePos;
    this->impl_->wheelbaseLength = vec3.Length();

    // gzerr << "wheel base length and track width: "
    //   << this->impl_->wheelbaseLength << " "
    //   << this->impl_->frontTrackWidth
    //   << " " << this->impl_->backTrackWidth << std::endl;

    // Max force that can be applied to hand steering wheel
    double handWheelForce = 10;
    this->impl_->handWheelPID.Init(100, 0, 10, 0, 0,
                                   handWheelForce, -handWheelForce);

    // Max force that can be applied to wheel steering joints
    double kMaxSteeringForceMagnitude = 5000;

    this->impl_->flWheelSteeringPID.SetCmdMax(kMaxSteeringForceMagnitude);
    this->impl_->flWheelSteeringPID.SetCmdMin(-kMaxSteeringForceMagnitude);

    this->impl_->frWheelSteeringPID.SetCmdMax(kMaxSteeringForceMagnitude);
    this->impl_->frWheelSteeringPID.SetCmdMin(-kMaxSteeringForceMagnitude);

    // Create a connection so the OnUpdate function is called at every simulation
    // iteration. Remove this call, the connection and the callback if not needed.
    this->impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&PriusHybridPlugin::OnUpdate, this));

    this->impl_->keyboardSub = this->impl_->gznode->Subscribe(
        "~/keyboard/keypress", &PriusHybridPlugin::OnKeyPress, this, true);
    this->impl_->worldControlPub =
        this->impl_->gznode->Advertise<gazebo::msgs::WorldControl>("~/world_control");

    this->impl_->node.Subscribe("/keypress", &PriusHybridPlugin::OnKeyPressIgn,
                                this);
  }

  void PriusHybridPlugin::OnUpdate()
  {
    // shortcut to this->impl_
    PriusHybridPluginPrivate *dPtr = this->impl_.get();

    std::lock_guard<std::mutex> lock(this->impl_->mutex);

    gazebo::common::Time curTime = this->impl_->world->SimTime();
    double dt = (curTime - this->impl_->lastSimTime).Double();
    if (dt < 0)
    {
      this->Reset();
      return;
    }
    else if (ignition::math::equal(dt, 0.0))
    {
      return;
    }

    dPtr->handWheelAngle = dPtr->handWheelJoint->Position();
    dPtr->flSteeringAngle = dPtr->flWheelSteeringJoint->Position();
    dPtr->frSteeringAngle = dPtr->frWheelSteeringJoint->Position();

    dPtr->flWheelAngularVelocity = dPtr->flWheelJoint->GetVelocity(0);
    dPtr->frWheelAngularVelocity = dPtr->frWheelJoint->GetVelocity(0);
    dPtr->blWheelAngularVelocity = dPtr->blWheelJoint->GetVelocity(0);
    dPtr->brWheelAngularVelocity = dPtr->brWheelJoint->GetVelocity(0);

    dPtr->chassisLinearVelocity = dPtr->chassisLink->WorldCoGLinearVel();
    // Convert meter/sec to miles/hour
    double linearVel = dPtr->chassisLinearVelocity.Length() * 2.23694;

    // Distance traveled in miles.
    this->impl_->odom += (fabs(linearVel) * dt / 3600.0);

    bool neutral = dPtr->directionState == PriusHybridPluginPrivate::NEUTRAL;

    this->impl_->lastSimTime = curTime;

    // Aero-dynamic drag on chassis
    // F: force in world frame, applied at center of mass
    // V: velocity in world frame of chassis center of mass
    // C: drag coefficient based on straight-ahead driving [N / (m/s)^2]
    // |V|: speed
    // V_hat: velocity unit vector
    // F = -C |V|^2 V_hat
    auto dragForce = -dPtr->chassisAeroForceGain *
                     dPtr->chassisLinearVelocity.SquaredLength() *
                     dPtr->chassisLinearVelocity.Normalized();
    dPtr->chassisLink->AddForce(dragForce);

    // PID (position) steering
    this->impl_->handWheelCmd =
        ignition::math::clamp(this->impl_->handWheelCmd,
                              -this->impl_->maxSteer / this->impl_->steeringRatio,
                              this->impl_->maxSteer / this->impl_->steeringRatio);
    double steerError =
        this->impl_->handWheelAngle - this->impl_->handWheelCmd;
    double steerCmd = this->impl_->handWheelPID.Update(steerError, dt);
    this->impl_->handWheelJoint->SetForce(0, steerCmd);
    // this->impl_->handWheelJoint->SetPosition(0, this->impl_->handWheelCmd);
    // this->impl_->handWheelJoint->SetLowStop(0, this->impl_->handWheelCmd);
    // this->impl_->handWheelJoint->SetHighStop(0, this->impl_->handWheelCmd);

    // PID (position) steering joints based on steering position
    // Ackermann steering geometry here
    //  \TODO provide documentation for these equations
    double tanSteer =
        tan(this->impl_->handWheelCmd * this->impl_->steeringRatio);
    this->impl_->flWheelSteeringCmd = atan2(tanSteer,
                                            1 - this->impl_->frontTrackWidth / 2 / this->impl_->wheelbaseLength *
                                                    tanSteer);
    this->impl_->frWheelSteeringCmd = atan2(tanSteer,
                                            1 + this->impl_->frontTrackWidth / 2 / this->impl_->wheelbaseLength *
                                                    tanSteer);
    // this->flWheelSteeringCmd = this->handWheelAngle * this->steeringRatio;
    // this->frWheelSteeringCmd = this->handWheelAngle * this->steeringRatio;

    double flwsError =
        this->impl_->flSteeringAngle - this->impl_->flWheelSteeringCmd;
    double flwsCmd = this->impl_->flWheelSteeringPID.Update(flwsError, dt);
    this->impl_->flWheelSteeringJoint->SetForce(0, flwsCmd);
    // this->impl_->flWheelSteeringJoint->SetPosition(0,
    // this->impl_->flWheelSteeringCmd);
    // this->impl_->flWheelSteeringJoint->SetLowStop(0,
    // this->impl_->flWheelSteeringCmd);
    // this->impl_->flWheelSteeringJoint->SetHighStop(0,
    // this->impl_->flWheelSteeringCmd);

    double frwsError =
        this->impl_->frSteeringAngle - this->impl_->frWheelSteeringCmd;
    double frwsCmd = this->impl_->frWheelSteeringPID.Update(frwsError, dt);
    this->impl_->frWheelSteeringJoint->SetForce(0, frwsCmd);
    // this->impl_->frWheelSteeringJoint->SetPosition(0,
    // this->impl_->frWheelSteeringCmd);
    // this->impl_->frWheelSteeringJoint->SetLowStop(0,
    // this->impl_->frWheelSteeringCmd);
    // this->impl_->frWheelSteeringJoint->SetHighStop(0,
    // this->impl_->frWheelSteeringCmd);

    // static common::Time lastErrorPrintTime = 0.0;
    // if (curTime - lastErrorPrintTime > 0.01 || curTime < lastErrorPrintTime)
    //{
    //   lastErrorPrintTime = curTime;
    //   double maxSteerError =
    //     std::abs(frwsError) > std::abs(flwsError) ? frwsError : flwsError;
    //   double maxSteerErrPer = maxSteerError / this->impl_->maxSteer * 100.0;
    //   std::cerr << std::fixed << "Max steering error: " << maxSteerErrPer
    //     << std::endl;
    // }

    // Model low-speed caaaareep and high-speed regen braking
    // with term added to gas/brake
    // Cross-over speed is 7 miles/hour
    // 10% throttle at 0 speed
    // max 2.5% braking at higher speeds
    double creepPercent;
    if (std::abs(linearVel) <= 7)
    {
      creepPercent = 0.1 * (1 - std::abs(linearVel) / 7);
    }
    else
    {
      creepPercent = 0.025 * (7 - std::abs(linearVel));
    }
    creepPercent = ignition::math::clamp(creepPercent, -0.025, 0.1);

    // Gas pedal torque.
    // Map gas torques to individual wheels.
    // Cut off gas torque at a given wheel if max speed is exceeded.
    // Use directionState to determine direction of that can be applied torque.
    // Note that definition of DirectionType allows multiplication to determine
    // torque direction.
    // also, make sure gas pedal is at least as large as the creepPercent.
    double gasPercent = std::max(this->impl_->gasPedalPercent, creepPercent);
    double gasMultiplier = this->GasTorqueMultiplier();
    double flGasTorque = 0, frGasTorque = 0, blGasTorque = 0, brGasTorque = 0;
    // Apply equal torque at left and right wheels, which is an implicit model
    // of the differential.
    if (fabs(dPtr->flWheelAngularVelocity * dPtr->flWheelRadius) < dPtr->maxSpeed &&
        fabs(dPtr->frWheelAngularVelocity * dPtr->frWheelRadius) < dPtr->maxSpeed)
    {
      flGasTorque = gasPercent * dPtr->frontTorque * gasMultiplier;
      frGasTorque = gasPercent * dPtr->frontTorque * gasMultiplier;
    }
    if (fabs(dPtr->blWheelAngularVelocity * dPtr->blWheelRadius) < dPtr->maxSpeed &&
        fabs(dPtr->brWheelAngularVelocity * dPtr->brWheelRadius) < dPtr->maxSpeed)
    {
      blGasTorque = gasPercent * dPtr->backTorque * gasMultiplier;
      brGasTorque = gasPercent * dPtr->backTorque * gasMultiplier;
    }
    double throttlePower =
        std::abs(flGasTorque * dPtr->flWheelAngularVelocity) +
        std::abs(frGasTorque * dPtr->frWheelAngularVelocity) +
        std::abs(blGasTorque * dPtr->blWheelAngularVelocity) +
        std::abs(brGasTorque * dPtr->brWheelAngularVelocity);

    // auto release handbrake as soon as the gas pedal is depressed
    if (this->impl_->gasPedalPercent > 0)
      this->impl_->handbrakePercent = 0.0;

    double brakePercent = this->impl_->brakePedalPercent + this->impl_->handbrakePercent;
    // use creep braking if not in Neutral
    if (!neutral)
    {
      brakePercent = std::max(brakePercent,
                              -creepPercent - this->impl_->gasPedalPercent);
    }

    brakePercent = ignition::math::clamp(brakePercent, 0.0, 1.0);
    dPtr->flWheelJoint->SetParam("friction", 0,
                                 dPtr->flJointFriction + brakePercent * dPtr->frontBrakeTorque);
    dPtr->frWheelJoint->SetParam("friction", 0,
                                 dPtr->frJointFriction + brakePercent * dPtr->frontBrakeTorque);
    dPtr->blWheelJoint->SetParam("friction", 0,
                                 dPtr->blJointFriction + brakePercent * dPtr->backBrakeTorque);
    dPtr->brWheelJoint->SetParam("friction", 0,
                                 dPtr->brJointFriction + brakePercent * dPtr->backBrakeTorque);

    this->impl_->flWheelJoint->SetForce(0, flGasTorque);
    this->impl_->frWheelJoint->SetForce(0, frGasTorque);
    this->impl_->blWheelJoint->SetForce(0, blGasTorque);
    this->impl_->brWheelJoint->SetForce(0, brGasTorque);

    // gzerr << "gas and brake torque " << flGasTorque << " "
    //       << flBrakeTorque << std::endl;

    // Battery

    // Speed x throttle regions
    //
    //    throttle |
    //             |
    //        high |____
    //             |    |
    //      medium |____|_____
    //             |    |     |
    //         low |____|_____|_________
    //              low  med   high    speed

    bool engineOn;
    bool regen = !neutral;
    double batteryChargePower = 0;
    double batteryDischargePower = 0;

    // Battery is below threshold
    if (this->impl_->batteryCharge < this->impl_->batteryLowThreshold)
    {
      // Gas engine is on and recharing battery
      engineOn = true;
      this->impl_->evMode = false;
      batteryChargePower = dPtr->kLowBatteryChargePower;
      throttlePower += dPtr->kLowBatteryChargePower;
    }
    // Neutral and battery not low
    else if (neutral)
    {
      // Gas engine is off, battery not recharged
      engineOn = false;
    }
    // Speed below medium-high threshold, throttle below low-medium threshold
    else if (linearVel < this->impl_->speedMediumHigh &&
             this->impl_->gasPedalPercent <= this->impl_->kGasPedalLowMedium)
    {
      // Gas engine is off, running on battery
      engineOn = false;
      batteryDischargePower = throttlePower;
    }
    // EV mode, speed below low-medium threshold, throttle below medium-high
    // threshold
    else if (this->impl_->evMode && linearVel < this->impl_->speedLowMedium && this->impl_->gasPedalPercent <= this->impl_->kGasPedalMediumHigh)
    {
      // Gas engine is off, running on battery
      engineOn = false;
      batteryDischargePower = throttlePower;
    }
    else
    {
      // Gas engine is on
      engineOn = true;
      this->impl_->evMode = false;
    }

    if (regen)
    {
      // regen max torque at same level as throttle limit in EV mode
      // but only at the front wheels
      batteryChargePower +=
          std::min(this->impl_->kGasPedalMediumHigh, brakePercent) * (dPtr->frontBrakeTorque * std::abs(dPtr->flWheelAngularVelocity) +
                                                                      dPtr->frontBrakeTorque * std::abs(dPtr->frWheelAngularVelocity) +
                                                                      dPtr->backBrakeTorque * std::abs(dPtr->blWheelAngularVelocity) +
                                                                      dPtr->backBrakeTorque * std::abs(dPtr->brWheelAngularVelocity));
    }
    dPtr->batteryCharge += dt / 3600 * (batteryChargePower / dPtr->batteryChargeWattHours - batteryDischargePower / dPtr->batteryDischargeWattHours);
    if (dPtr->batteryCharge > 1)
    {
      dPtr->batteryCharge = 1;
    }

    // engine has minimum gas flow if the throttle is pressed at all
    if (engineOn && throttlePower > 0)
    {
      dPtr->gasConsumption += dt * (dPtr->minGasFlow + throttlePower / dPtr->gasEfficiency / dPtr->kGasEnergyDensity);
    }

    // Accumulated mpg since last reset
    // max value: 999.9
    // double mpg = std::min(999.9,
    //                       dPtr->odom / std::max(dPtr->gasConsumption, 1e-6));

    if ((curTime - this->impl_->lastMsgTime) > .5)
    {
      this->impl_->posePub.Publish(
          ignition::msgs::Convert(this->impl_->model->WorldPose()));

      ignition::msgs::Double_V consoleMsg;

      // linearVel (meter/sec) = (2*PI*r) * (rad/sec).
      double linearVel = (2.0 * IGN_PI * this->impl_->flWheelRadius) *
                         ((this->impl_->flWheelAngularVelocity +
                           this->impl_->frWheelAngularVelocity) *
                          0.5);

      // Convert meter/sec to miles/hour
      linearVel *= 2.23694;

      // Distance traveled in miles.
      this->impl_->odom += (fabs(linearVel) * dt / 3600);

      // \todo: Actually compute MPG
      double mpg = 1.0 / std::max(linearVel, 0.0);

      // Gear information: 1=drive, 2=reverse, 3=neutral
      if (this->impl_->directionState == PriusHybridPluginPrivate::FORWARD)
        consoleMsg.add_data(1.0);
      else if (this->impl_->directionState == PriusHybridPluginPrivate::REVERSE)
        consoleMsg.add_data(2.0);
      else if (this->impl_->directionState == PriusHybridPluginPrivate::NEUTRAL)
        consoleMsg.add_data(3.0);

      // MPH. A speedometer does not go negative.
      consoleMsg.add_data(std::max(linearVel, 0.0));

      // MPG
      consoleMsg.add_data(mpg);

      // Miles
      consoleMsg.add_data(this->impl_->odom);

      // EV mode
      this->impl_->evMode ? consoleMsg.add_data(1.0) : consoleMsg.add_data(0.0);

      // Battery state
      consoleMsg.add_data(this->impl_->batteryCharge);

      this->impl_->consolePub.Publish(consoleMsg);

      // Output prius car data.
      this->impl_->posePub.Publish(
          ignition::msgs::Convert(this->impl_->model->WorldPose()));

      this->impl_->lastMsgTime = curTime;
    }

    // reset if last command is more than x sec ago
    if ((curTime - this->impl_->lastPedalCmdTime).Double() > 0.3)
    {
      this->impl_->gasPedalPercent = 0.0;
      this->impl_->brakePedalPercent = 0.0;
    }

    if ((curTime - this->impl_->lastSteeringCmdTime).Double() > 0.3)
    {
      this->impl_->handWheelCmd = 0;
    }
  }

  /////////////////////////////////////////////////
  void PriusHybridPlugin::OnReset(const ignition::msgs::Any & /*_msg*/)
  {
    gazebo::msgs::WorldControl msg;
    msg.mutable_reset()->set_all(true);

    this->impl_->worldControlPub->Publish(msg);
  }

  /////////////////////////////////////////////////
  void PriusHybridPlugin::OnStop(const ignition::msgs::Any & /*_msg*/)
  {
    ignition::msgs::StringMsg req;
    ignition::msgs::StringMsg rep;
    bool result = false;
    unsigned int timeout = 5000;
    bool executed = this->impl_->node.Request("/priuscup/upload",
                                              req, timeout, rep, result);
    if (executed)
    {
      std::cerr << "Result: " << result << std::endl;
      std::cerr << rep.data() << std::endl;
    }
    else
    {
      std::cerr << "Service call timed out" << std::endl;
    }
  }

  void PriusHybridPlugin::OnCmdVel(const ignition::msgs::Pose &_msg)
  {
    std::lock_guard<std::mutex> lock(this->impl_->mutex);

    this->impl_->gasPedalPercent = std::min(_msg.position().x(), 1.0);
    this->impl_->handWheelCmd = _msg.position().y();
    this->impl_->brakePedalPercent = _msg.position().z();

    this->impl_->lastPedalCmdTime = this->impl_->world->SimTime();
    this->impl_->lastSteeringCmdTime = this->impl_->world->SimTime();
  }
  /////////////////////////////////////////////////
  void PriusHybridPlugin::OnCmdGear(const ignition::msgs::Int32 &_msg)
  {
    std::lock_guard<std::mutex> lock(this->impl_->mutex);

    // -1 reverse, 0 neutral, 1 forward
    int state = static_cast<int>(this->impl_->directionState);
    state += _msg.data();
    state = ignition::math::clamp(state, -1, 1);
    this->impl_->directionState =
        static_cast<PriusHybridPluginPrivate::DirectionType>(state);
  }

  /////////////////////////////////////////////////
  void PriusHybridPlugin::OnCmdMode(const ignition::msgs::Boolean & /*_msg*/)
  {
    // toggle ev mode
    std::lock_guard<std::mutex> lock(this->impl_->mutex);
    this->impl_->evMode = !this->impl_->evMode;
  }

  void PriusHybridPlugin::UpdateHandWheelRatio()
  {
    // The total range the steering wheel can rotate
    this->impl_->handWheelHigh = 7.85;
    this->impl_->handWheelLow = -7.85;
    double handWheelRange =
        this->impl_->handWheelHigh - this->impl_->handWheelLow;
    double high = 0.8727;
    high = std::min(high, this->impl_->maxSteer);
    double low = -0.8727;
    low = std::max(low, -this->impl_->maxSteer);
    double tireAngleRange = high - low;

    // Compute the angle ratio between the steering wheel and the tires
    this->impl_->steeringRatio = tireAngleRange / handWheelRange;
  }

  double PriusHybridPlugin::CollisionRadius(gazebo::physics::CollisionPtr _coll)
  {
    if (!_coll || !(_coll->GetShape()))
      return 0;
    if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE))
    {
      gazebo::physics::CylinderShape *cyl =
          static_cast<gazebo::physics::CylinderShape *>(_coll->GetShape().get());
      return cyl->GetRadius();
    }
    else if (_coll->GetShape()->HasType(gazebo::physics::Base::SPHERE_SHAPE))
    {
      gazebo::physics::SphereShape *sph =
          static_cast<gazebo::physics::SphereShape *>(_coll->GetShape().get());
      return sph->GetRadius();
    }
    return 0;
  }

  /////////////////////////////////////////////////
  void PriusHybridPlugin::OnKeyPress(ConstAnyPtr &_msg)
  {
    RCLCPP_INFO(this->impl_->ros_node_->get_logger(), "Got key [%d]", *_msg->string_value().c_str());
    this->KeyControl(_msg->int_value());
  }
  /////////////////////////////////////////////////
  void PriusHybridPlugin::OnKeyPressIgn(const ignition::msgs::Any &_msg)
  {
    this->KeyControl(_msg.int_value());
  }
  /////////////////////////////////////////////////
  void PriusHybridPlugin::KeyControlTypeA(const int _key)
  {
    std::lock_guard<std::mutex> lock(this->impl_->mutex);

    switch (_key)
    {
    // e - gas pedal
    case 69:
    case 101:
    {
      this->impl_->brakePedalPercent = 0.0;
      this->impl_->gasPedalPercent += 0.1;
      this->impl_->gasPedalPercent =
          std::min(this->impl_->gasPedalPercent, 1.0);
      this->impl_->lastPedalCmdTime = this->impl_->world->SimTime();
      break;
    }
    // w - release pedals
    case 87:
    case 119:
    {
      this->impl_->brakePedalPercent = 0.0;
      this->impl_->gasPedalPercent = 0.0;
      this->impl_->lastPedalCmdTime = this->impl_->world->SimTime();
      break;
    }
    // q - brake
    case 113:
    {
      this->impl_->gasPedalPercent = 0.0;
      this->impl_->brakePedalPercent += 0.1;
      this->impl_->brakePedalPercent =
          std::min(this->impl_->brakePedalPercent, 1.0);
      this->impl_->lastPedalCmdTime = this->impl_->world->SimTime();
      break;
    }
    // a - steer left
    case 65:
    case 97:
    {
      this->impl_->handWheelCmd += 0.25;
      this->impl_->handWheelCmd = std::min(this->impl_->handWheelCmd,
                                           this->impl_->handWheelHigh);
      this->impl_->lastSteeringCmdTime = this->impl_->world->SimTime();
      break;
    }
    // d - steer right
    case 68:
    case 100:
    {
      this->impl_->handWheelCmd -= 0.25;
      this->impl_->handWheelCmd = std::max(this->impl_->handWheelCmd,
                                           this->impl_->handWheelLow);
      this->impl_->lastSteeringCmdTime = this->impl_->world->SimTime();
      break;
    }
    // s - center steering
    case 83:
    case 115:
    {
      this->impl_->handWheelCmd = 0;
      this->impl_->lastSteeringCmdTime = this->impl_->world->SimTime();
      break;
    }
    // z reverse
    case 90:
    case 122:
    {
      this->impl_->directionState = PriusHybridPluginPrivate::REVERSE;
      break;
    }
    // x neutral
    case 88:
    case 120:
    {
      this->impl_->directionState = PriusHybridPluginPrivate::NEUTRAL;
      break;
    }
    // c forward
    case 67:
    case 99:
    {
      this->impl_->directionState = PriusHybridPluginPrivate::FORWARD;
      break;
    }

    default:
    {
      this->impl_->brakePedalPercent = 0;
      this->impl_->gasPedalPercent = 0;
      break;
    }
    }
  }

  /////////////////////////////////////////////////
  void PriusHybridPlugin::KeyControlTypeB(const int _key)
  {
    std::lock_guard<std::mutex> lock(this->impl_->mutex);

    switch (_key)
    {
    // w - accelerate forward
    case 87:
    case 119:
    {
      // this->impl_->brakePedalPercent = 0.0;
      // this->impl_->gasPedalPercent += 0.1;
      // this->impl_->gasPedalPercent =
      //     std::min(this->impl_->gasPedalPercent, 1.0);
      // this->impl_->directionState = PriusHybridPluginPrivate::FORWARD;
      // this->impl_->lastPedalCmdTime = this->impl_->world->SimTime();
      break;
    }
    // a - steer left
    case 65:
    case 97:
    {
      // this->impl_->handWheelCmd += 0.25;
      // this->impl_->handWheelCmd = std::min(this->impl_->handWheelCmd,
      //                                      this->impl_->handWheelHigh);
      // this->impl_->lastSteeringCmdTime = this->impl_->world->SimTime();
      break;
    }
    // s - reverse
    case 83:
    case 115:
    {
      // this->impl_->brakePedalPercent = 0.0;
      // if (this->impl_->directionState != PriusHybridPluginPrivate::REVERSE)
      //   this->impl_->gasPedalPercent = 0.0;
      // this->impl_->gasPedalPercent += 0.1;
      // this->impl_->gasPedalPercent =
      //     std::min(this->impl_->gasPedalPercent, 1.0);
      // this->impl_->directionState = PriusHybridPluginPrivate::REVERSE;
      // this->impl_->lastPedalCmdTime = this->impl_->world->SimTime();
      break;
    }
    // d - steer right
    case 68:
    case 100:
    {
      // this->impl_->handWheelCmd -= 0.25;
      // this->impl_->handWheelCmd = std::max(this->impl_->handWheelCmd,
      //                                      this->impl_->handWheelLow);
      // this->impl_->lastSteeringCmdTime = this->impl_->world->SimTime();
      break;
    }
    // e brake
    case 69:
    case 101:
    {
      // this->impl_->brakePedalPercent = 1.0;
      // this->impl_->gasPedalPercent = 0.0;
      // this->impl_->lastPedalCmdTime = this->impl_->world->SimTime();
      break;
    }
    // x neutral
    case 88:
    case 120:
    {
      // this->impl_->directionState = PriusHybridPluginPrivate::NEUTRAL;
      break;
    }
    // q - EV mode
    case 81:
    case 113:
    {
      // // avoid rapid mode changes due to repeated key press
      // gazebo::common::Time now = this->impl_->world->SimTime();
      // if ((now - this->impl_->lastModeCmdTime).Double() > 0.3)
      // {
      //   this->impl_->evMode = !this->impl_->evMode;
      //   this->impl_->lastModeCmdTime = now;
      // }
      break;
    }
    default:
    {
      break;
    }
    }
  }

  /////////////////////////////////////////////////
  void PriusHybridPlugin::KeyControl(const int _key)
  {
    if (this->impl_->keyControl == 0)
      this->KeyControlTypeA(_key);
    else if (this->impl_->keyControl == 1)
      this->KeyControlTypeB(_key);
  }

  /////////////////////////////////////////////////
  double PriusHybridPlugin::GasTorqueMultiplier()
  {
    // if (this->impl_->keyState == ON)
    {
      if (this->impl_->directionState == PriusHybridPluginPrivate::FORWARD)
        return 1.0;
      else if (this->impl_->directionState == PriusHybridPluginPrivate::REVERSE)
        return -1.0;
    }
    return 0;
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PriusHybridPlugin)
} // namespace gazebo_plugins