#include "RobotControl.hpp"

#include <cmath>

namespace
{
constexpr uint8_t kYawChannel = 0;
constexpr uint8_t kForwardChannel = 1;
constexpr uint8_t kSpeedChannel = 2;
constexpr uint8_t kMainModeChannel = 4;
constexpr uint8_t kSubModeChannel = 7;
constexpr uint8_t kSafetyChannel = 8;
constexpr uint8_t kStepChannel = 9;
constexpr uint32_t kSbusTimeoutMs = 250;
constexpr uint32_t kUsbTimeoutMs = 150;
constexpr uint32_t kSafetyRecoveryMs = 150;
constexpr uint32_t kObstacleWheelStopMs = 200;
constexpr float kMoveEnterDeadband = 0.25F;
constexpr float kMoveExitDeadband = 0.12F;
constexpr uint32_t kWheelReverseNeutralMs = 200;

float Axis(int16_t value) { return RCDog::Clamp(value / 100.0F, -1.0F, 1.0F); }
}

RobotControl::RobotControl(LibXR::HardwareContainer&,
                           LibXR::ApplicationManager& app, SbusReceiver& sbus,
                           DogMotor& dog_motor, WheelMotor& wheel_motor,
                           ObstacleController& obstacle, HostLink& host_link,
                           uint32_t stack_size)
    : sbus_(sbus),
      dog_(dog_motor),
      wheel_(wheel_motor),
      obstacle_(obstacle),
      host_(host_link)
{
  status_.schema_version = RCDog::kSchemaVersion;
  app.Register(*this);
  thread_.Create(this, ThreadEntry, "robot_control", stack_size,
                 LibXR::Thread::Priority::REALTIME);
}

void RobotControl::OnMonitor() {}
RCDog::RobotStatusV1 RobotControl::Status() const
{
  RCDog::RobotStatusV1 status{};
  taskENTER_CRITICAL();
  status = published_status_;
  taskEXIT_CRITICAL();
  return status;
}
void RobotControl::ThreadEntry(RobotControl* self) { self->Run(); }

void RobotControl::Run()
{
  LibXR::MillisecondTimestamp wake(LibXR::Thread::GetTime());
  while (true)
  {
    Tick(LibXR::Thread::GetTime());
    LibXR::Thread::SleepUntil(wake, 1);
  }
}

void RobotControl::Tick(uint32_t now_ms)
{
  const auto sbus = sbus_.Snapshot();
  UpdateSafety(sbus, now_ms);
  const Input input = SelectInput(sbus, now_ms);
  if (safety_latched_ || !input.valid)
  {
    StopAll(safety_latched_);
  }
  else
  {
    Execute(input, now_ms);
  }
  obstacle_.Tick(now_ms);
  PublishStatus(input, now_ms);
}

RobotControl::Input RobotControl::SelectInput(const RCDog::SbusSample& sbus,
                                              uint32_t now_ms)
{
  Input input{};
  if (!sbus_.IsFresh(now_ms, kSbusTimeoutMs))
  {
    RevokeUsb();
    return input;
  }
  const uint8_t main = SbusReceiver::Switch3(sbus, kMainModeChannel);
  const uint8_t sub = SbusReceiver::Switch3(sbus, kSubModeChannel);
  const bool physical_authorizes_usb = main == 0 && sub == 0 &&
      std::abs(sbus.normalized[kYawChannel]) <= 12 &&
      std::abs(sbus.normalized[kForwardChannel]) <= 12;

  RCDog::ControlCommandV1 latest{};
  uint32_t rx_ms = 0;
  const bool fresh_packet = host_.ReadCommand(latest, rx_ms, usb_generation_);
  if (!physical_authorizes_usb)
  {
    RevokeUsb();
  }
  else if (fresh_packet)
  {
    const bool safe_zero = latest.mode == 0 && latest.flags == 0 &&
        latest.yaw == 0 && latest.forward == 0 && latest.speed == 0;
    if (!usb_active_)
    {
      if (latest.session_id != blocked_session_ && safe_zero &&
          (safe_zero_count_ == 0 || latest.session_id == usb_session_) &&
          (safe_zero_count_ == 0 || CounterForward(latest.command_counter,
                                                   last_usb_counter_)))
      {
        usb_session_ = latest.session_id;
        last_usb_counter_ = latest.command_counter;
        last_accepted_usb_counter_ = latest.command_counter;
        usb_rx_ms_ = rx_ms;
        usb_command_ = latest;
        if (++safe_zero_count_ >= 3)
        {
          usb_active_ = true;
          usb_timeout_latched_ = false;
        }
      }
      else
      {
        safe_zero_count_ = 0;
        usb_session_ = 0;
      }
    }
    else if (latest.session_id == usb_session_ &&
             CounterForward(latest.command_counter, last_usb_counter_))
    {
      last_usb_counter_ = latest.command_counter;
      last_accepted_usb_counter_ = latest.command_counter;
      usb_rx_ms_ = rx_ms;
      usb_command_ = latest;
    }
  }

  if (usb_active_ && now_ms - usb_rx_ms_ <= kUsbTimeoutMs)
  {
    input.source = RCDog::ControlSource::USB;
    // USB mode 8 is deliberately a second stand-hold mode.
    input.mode = usb_command_.mode == 8 ? RCDog::RobotMode::STAND_HOLD_ALT
                                       : static_cast<RCDog::RobotMode>(usb_command_.mode);
    input.requested_mode = usb_command_.mode;
    input.yaw = RCDog::Clamp(usb_command_.yaw / 1000.0F, -1.0F, 1.0F);
    input.forward = RCDog::Clamp(usb_command_.forward / 1000.0F, -1.0F, 1.0F);
    input.speed = RCDog::Clamp(usb_command_.speed / 1000.0F, 0.0F, 1.0F);
    input.flags = usb_command_.flags;
    input.valid = true;
    return input;
  }
  if (usb_active_)
  {
    usb_timeout_latched_ = true;
    RevokeUsb();
    return input;
  }

  input.source = RCDog::ControlSource::SBUS;
  input.mode = static_cast<RCDog::RobotMode>(main * 3U + sub);
  input.requested_mode = static_cast<uint8_t>(input.mode);
  input.yaw = Axis(sbus.normalized[kYawChannel]);
  input.forward = Axis(sbus.normalized[kForwardChannel]);
  input.speed = RCDog::Clamp((sbus.normalized[kSpeedChannel] + 100.0F) / 200.0F,
                            0.0F, 1.0F);
  input.flags = RCDog::DEADMAN | RCDog::MOTION_ENABLE;
  input.valid = true;
  return input;
}

void RobotControl::UpdateSafety(const RCDog::SbusSample& sbus, uint32_t now_ms)
{
  const bool fresh = sbus_.IsFresh(now_ms, kSbusTimeoutMs);
  const bool trigger = !fresh || sbus.normalized[kSafetyChannel] >= 50;
  if (trigger)
  {
    safety_latched_ = true;
    release_since_ms_ = 0;
    RevokeUsb();
    obstacle_.SafetyAbort();
    return;
  }
  const bool released = sbus.normalized[kSafetyChannel] <= 20;
  const bool low_low = SbusReceiver::Switch3(sbus, kMainModeChannel) == 0 &&
                       SbusReceiver::Switch3(sbus, kSubModeChannel) == 0;
  if (safety_latched_ && released && low_low)
  {
    if (release_since_ms_ == 0)
    {
      release_since_ms_ = now_ms;
    }
    if (now_ms - release_since_ms_ >= kSafetyRecoveryMs)
    {
      safety_latched_ = false;
      release_since_ms_ = 0;
    }
  }
  else if (safety_latched_)
  {
    release_since_ms_ = 0;
  }
}

void RobotControl::Execute(const Input& input, uint32_t now_ms)
{
  status_.requested_mode = input.requested_mode;
  status_.entry_state = static_cast<uint8_t>(RCDog::EntryState::ENTERING);
  status_.block_reason = static_cast<uint8_t>(RCDog::BlockReason::NONE);
  const bool motion_allowed = input.source == RCDog::ControlSource::SBUS ||
      ((input.flags & (RCDog::DEADMAN | RCDog::MOTION_ENABLE)) ==
       (RCDog::DEADMAN | RCDog::MOTION_ENABLE));
  const bool smooth_stop = (input.flags & RCDog::SMOOTH_STOP) != 0;

  if (input.mode != RCDog::RobotMode::OBSTACLE)
  {
    obstacle_.SetRequested(false);
    obstacle_started_ = false;
    obstacle_stop_since_ms_ = 0;
    step_input_initialized_ = false;
    step_stable_frames_ = 0;
    if (!obstacle_.CanExit())
    {
      wheel_.SetMode(WheelMotor::Mode::HOLD);
      status_.active_mode = static_cast<uint8_t>(RCDog::RobotMode::OBSTACLE);
      status_.entry_state = static_cast<uint8_t>(RCDog::EntryState::WAIT_OBSTACLE);
      status_.block_reason = static_cast<uint8_t>(RCDog::BlockReason::OBSTACLE);
      return;
    }
    obstacle_profile_locked_ = false;
  }
  switch (input.mode)
  {
    case RCDog::RobotMode::MOTOR_CHECK:
      dog_.SafeStop();
      wheel_.StopAndLock();
      break;
    case RCDog::RobotMode::LOW_WHEEL:
    case RCDog::RobotMode::LOW_WHEEL_REVERSE:
    {
      const bool reverse_pose = input.mode == RCDog::RobotMode::LOW_WHEEL_REVERSE;
      dog_.RequestMechanicalPose(reverse_pose);
      if (!dog_.IsMechanicalPoseReady(reverse_pose))
      {
        wheel_.SetMode(WheelMotor::Mode::HOLD);
        status_.block_reason = static_cast<uint8_t>(
            RCDog::BlockReason::MECHANICAL_PERMIT);
        break;
      }
      if (!wheel_.TryClearLock())
      {
        status_.block_reason = static_cast<uint8_t>(
            wheel_.IsHealthy() ? RCDog::BlockReason::WHEEL_STOP
                               : RCDog::BlockReason::WHEEL_FAULT);
        break;
      }
      wheel_.SetMode(WheelMotor::Mode::DRIVE);
      wheel_.SetMotion(motion_allowed ? LowWheelForward(input.forward, now_ms) : 0.0F,
                       motion_allowed ? input.yaw : 0.0F, 40.0F + 160.0F * input.speed);
      break;
    }
    case RCDog::RobotMode::STAND_HOLD:
    case RCDog::RobotMode::STAND_HOLD_ALT:
      dog_.RequestStand();
      wheel_.SetMode(WheelMotor::Mode::HOLD);
      break;
    case RCDog::RobotMode::STAND_WHEEL:
      dog_.RequestStand();
      if (!dog_.IsStanding())
      {
        wheel_.SetMode(WheelMotor::Mode::HOLD);
        status_.block_reason = static_cast<uint8_t>(RCDog::BlockReason::STAND);
        break;
      }
      if (!wheel_.TryClearLock())
      {
        wheel_.SetMode(WheelMotor::Mode::HOLD);
        status_.block_reason = static_cast<uint8_t>(
            wheel_.IsHealthy() ? RCDog::BlockReason::WHEEL_STOP
                               : RCDog::BlockReason::WHEEL_FAULT);
        break;
      }
      wheel_.SetMode(WheelMotor::Mode::DRIVE);
      wheel_.SetMotion(motion_allowed ? input.forward : 0.0F,
                       motion_allowed ? input.yaw : 0.0F, 40.0F + 160.0F * input.speed);
      break;
    case RCDog::RobotMode::GAIT_ONLY:
    case RCDog::RobotMode::GAIT_WHEEL:
      if (!dog_.IsStanding() && dog_.GetState() != DogMotor::State::GAIT)
      {
        dog_.RequestStand();
        wheel_.SetMode(WheelMotor::Mode::HOLD);
        status_.block_reason = static_cast<uint8_t>(RCDog::BlockReason::STAND);
        break;
      }
      dog_.SetGait(input.forward, input.yaw, input.speed, motion_allowed, smooth_stop);
      if (input.mode == RCDog::RobotMode::GAIT_WHEEL && wheel_.IsHealthy())
      {
        if (!wheel_.TryClearLock())
        {
          wheel_.SetMode(WheelMotor::Mode::HOLD);
          status_.block_reason = static_cast<uint8_t>(
              RCDog::BlockReason::WHEEL_STOP);
          break;
        }
        wheel_.SetMode(WheelMotor::Mode::DRIVE);
        wheel_.SetMotion(motion_allowed && !smooth_stop ? input.forward : 0.0F,
                         motion_allowed && !smooth_stop ? input.yaw : 0.0F,
                         30.0F + 70.0F * input.speed);
      }
      else
      {
        wheel_.SetMode(WheelMotor::Mode::HOLD);
      }
      break;
    case RCDog::RobotMode::OBSTACLE:
    {
      wheel_.SetMode(WheelMotor::Mode::HOLD);
      const auto sample = sbus_.Snapshot();
      if (!obstacle_profile_locked_)
      {
        const int16_t profile = sample.normalized[kSpeedChannel];
        obstacle_.SelectProfile(profile < -33 ? RCDog::StairProfile::LOW :
                                (profile > 33 ? RCDog::StairProfile::HIGH :
                                                RCDog::StairProfile::MID));
        obstacle_profile_locked_ = true;
      }
      // A partially completed sequence owns the current foot pose. Re-authorize
      // it directly instead of commanding a stand transition first.
      if (!obstacle_.CanExit())
      {
        obstacle_.SetRequested(true);
        obstacle_started_ = true;
        break;
      }
      dog_.RequestStand();
      if (!dog_.IsStanding())
      {
        status_.block_reason = static_cast<uint8_t>(RCDog::BlockReason::STAND);
        break;
      }
      if (!wheel_.IsStopped())
      {
        obstacle_stop_since_ms_ = 0;
        status_.block_reason = static_cast<uint8_t>(RCDog::BlockReason::WHEEL_STOP);
        break;
      }
      if (obstacle_stop_since_ms_ == 0)
      {
        obstacle_stop_since_ms_ = now_ms;
      }
      if (now_ms - obstacle_stop_since_ms_ < kObstacleWheelStopMs)
      {
        status_.block_reason = static_cast<uint8_t>(RCDog::BlockReason::WHEEL_STOP);
        break;
      }
      obstacle_.SetRequested(true);
      if (sample.frame_counter != last_step_frame_)
      {
        last_step_frame_ = sample.frame_counter;
        const bool candidate = sample.channel[kStepChannel] > 1200 ? true :
            (sample.channel[kStepChannel] < 800 ? false : step_candidate_high_);
        if (!step_input_initialized_)
        {
          step_input_initialized_ = true;
          step_input_high_ = candidate;
          step_candidate_high_ = candidate;
          step_stable_frames_ = 0;
        }
        else if (candidate != step_candidate_high_)
        {
          step_candidate_high_ = candidate;
          step_stable_frames_ = 1;
        }
        else if (candidate != step_input_high_ && step_stable_frames_ < 8)
        {
          ++step_stable_frames_;
          if (step_stable_frames_ >= 8)
          {
            step_input_high_ = candidate;
            step_stable_frames_ = 0;
            if (!obstacle_started_)
            {
              obstacle_started_ = obstacle_.RequestStep();
            }
          }
        }
      }
      break;
    }
  }
  status_.active_mode = static_cast<uint8_t>(input.mode);
  status_.entry_state = status_.block_reason == 0
      ? static_cast<uint8_t>(RCDog::EntryState::ACTIVE)
      : static_cast<uint8_t>(RCDog::EntryState::BLOCKED);
}

void RobotControl::StopAll(bool safety)
{
  dog_.SafeStop();
  wheel_.StopAndLock();
  obstacle_.SetRequested(false);
  low_wheel_direction_ = 0;
  low_wheel_reverse_rearm_ = false;
  low_wheel_neutral_since_ms_ = 0;
  status_.entry_state = static_cast<uint8_t>(RCDog::EntryState::BLOCKED);
  status_.block_reason = static_cast<uint8_t>(safety ? RCDog::BlockReason::SAFETY_LATCH
                                                     : RCDog::BlockReason::INPUT_TIMEOUT);
}

void RobotControl::PublishStatus(const Input& input, uint32_t now_ms)
{
  const RCDog::ObstacleFault obstacle_fault = obstacle_.Fault();
  status_.schema_version = RCDog::kSchemaVersion;
  status_.control_source = static_cast<uint8_t>(input.source);
  status_.safety_latched = safety_latched_ ? 1 : 0;
  status_.obstacle_state = static_cast<uint8_t>(obstacle_.State());
  status_.obstacle_fault = static_cast<uint8_t>(obstacle_fault);
  status_.leg_online_mask = dog_.OnlineMask();
  status_.wheel_online_mask = wheel_.OnlineMask();
  status_.reserved = 0;
  status_.fault_bits = dog_.FaultBits() | wheel_.FaultBits();
  if (!sbus_.IsFresh(now_ms)) status_.fault_bits |= RCDog::FAULT_SBUS_LOST;
  if (usb_timeout_latched_)
    status_.fault_bits |= RCDog::FAULT_USB_LOST;
  if (safety_latched_) status_.fault_bits |= RCDog::FAULT_SAFETY_LATCHED;
  if (obstacle_fault != RCDog::ObstacleFault::NONE)
    status_.fault_bits |= RCDog::FAULT_OBSTACLE;
  if (host_.ProtocolErrors() != 0) status_.fault_bits |= RCDog::FAULT_USB_PROTOCOL;
  status_.last_command_counter = last_accepted_usb_counter_;
  status_.uptime_ms = now_ms;
  const RCDog::RobotStatusV1 snapshot = status_;
  taskENTER_CRITICAL();
  published_status_ = snapshot;
  taskEXIT_CRITICAL();
  host_.SetStatus(snapshot);
}

void RobotControl::RevokeUsb()
{
  if (usb_active_)
  {
    blocked_session_ = usb_session_;
  }
  usb_active_ = false;
  safe_zero_count_ = 0;
  usb_session_ = 0;
  last_usb_counter_ = 0;
}

bool RobotControl::CounterForward(uint32_t value, uint32_t previous)
{
  return static_cast<int32_t>(value - previous) > 0;
}

float RobotControl::LowWheelForward(float requested, uint32_t now_ms)
{
  const bool neutral = std::fabs(requested) <= kMoveExitDeadband;
  int8_t direction = low_wheel_direction_;
  if (requested > kMoveEnterDeadband)
  {
    direction = 1;
  }
  else if (requested < -kMoveEnterDeadband)
  {
    direction = -1;
  }
  else if (neutral)
  {
    direction = 0;
  }
  if (direction != 0 && low_wheel_direction_ != 0 &&
      direction != low_wheel_direction_)
  {
    low_wheel_reverse_rearm_ = true;
    low_wheel_neutral_since_ms_ = 0;
    direction = 0;
  }
  if (low_wheel_reverse_rearm_)
  {
    direction = 0;
    if (neutral)
    {
      if (low_wheel_neutral_since_ms_ == 0)
      {
        low_wheel_neutral_since_ms_ = now_ms;
      }
      else if (now_ms - low_wheel_neutral_since_ms_ >= kWheelReverseNeutralMs)
      {
        low_wheel_reverse_rearm_ = false;
        low_wheel_neutral_since_ms_ = 0;
      }
    }
    else
    {
      low_wheel_neutral_since_ms_ = 0;
    }
  }
  low_wheel_direction_ = direction;
  return static_cast<float>(direction);
}
