#include "ObstacleController.hpp"

#include <cstring>

namespace
{
constexpr uint8_t kLF = 1U << 0;
constexpr uint8_t kRF = 1U << 1;
constexpr uint8_t kLB = 1U << 2;
constexpr uint8_t kRB = 1U << 3;
constexpr uint8_t kAllLegs = kLF | kRF | kLB | kRB;
constexpr float kFrontZ = 300.0F;
constexpr float kRearZ = 315.0F;
constexpr float kCompactX = 108.0F;
constexpr float kForwardX = 192.0F;
constexpr float kRearShiftX = -192.0F;

void SetPose(RCDog::FootTarget target[4], float front_x, float front_z,
             float rear_x, float rear_z)
{
  target[0] = {front_x, front_z};
  target[1] = {front_x, front_z};
  target[2] = {rear_x, rear_z};
  target[3] = {rear_x, rear_z};
}
}  // namespace

ObstacleController::ObstacleController(LibXR::HardwareContainer&,
                                       LibXR::ApplicationManager& app,
                                       DogMotor& dog_motor,
                                       WheelMotor& wheel_motor)
    : dog_(dog_motor), wheel_(wheel_motor)
{
  app.Register(*this);
}

void ObstacleController::OnMonitor() {}

void ObstacleController::SetRequested(bool requested)
{
  if (requested && !requested_ && CanExit())
  {
    fault_ = RCDog::ObstacleFault::NONE;
    state_ = RCDog::ObstacleState::DISABLED;
  }
  requested_ = requested;
  if (!requested && CanExit())
  {
    state_ = RCDog::ObstacleState::DISABLED;
    step_pending_ = false;
  }
}

void ObstacleController::SelectProfile(RCDog::StairProfile profile)
{
  if (state_ != RCDog::ObstacleState::MOVING)
  {
    profile_ = profile;
  }
}

bool ObstacleController::RequestStep()
{
  if (!requested_ || state_ == RCDog::ObstacleState::MOVING ||
      state_ == RCDog::ObstacleState::FAULT)
  {
    return false;
  }
  step_pending_ = true;
  state_ = RCDog::ObstacleState::PRECHECK;
  return true;
}

void ObstacleController::SafetyAbort()
{
  dog_.SafeStop();
  wheel_.StopAndLock();
  const bool unfinished_sequence = sequence_count_ != 0 &&
                                   phase_ < sequence_count_;
  if (requested_ || unfinished_sequence ||
      state_ == RCDog::ObstacleState::PRECHECK ||
      state_ == RCDog::ObstacleState::MOVING)
  {
    Fail(RCDog::ObstacleFault::SAFETY);
  }
}

void ObstacleController::Tick(uint32_t now_ms)
{
  if (!requested_)
  {
    if (state_ == RCDog::ObstacleState::MOVING && segment_started_ &&
        dog_.MotionComplete())
    {
      ++phase_;
      segment_started_ = false;
      dwell_started_ms_ = 0;
      state_ = RCDog::ObstacleState::READY;
    }
    return;
  }
  if (state_ == RCDog::ObstacleState::DISABLED)
  {
    state_ = RCDog::ObstacleState::READY;
  }
  if (!step_pending_ && state_ != RCDog::ObstacleState::MOVING)
  {
    if (sequence_count_ == 0 || phase_ >= sequence_count_)
    {
      return;
    }
    state_ = RCDog::ObstacleState::MOVING;
  }
  if (state_ == RCDog::ObstacleState::PRECHECK)
  {
    wheel_.SetMode(WheelMotor::Mode::HOLD);
    if (!dog_.IsHealthy() || !wheel_.IsHealthy())
    {
      Fail(RCDog::ObstacleFault::PRECHECK);
      return;
    }
    if (!dog_.IsStanding() || !wheel_.IsStopped())
    {
      return;
    }
    BuildSequence();
    phase_ = 0;
    segment_started_ = false;
    step_pending_ = false;
    state_ = RCDog::ObstacleState::MOVING;
  }
  if (state_ != RCDog::ObstacleState::MOVING)
  {
    return;
  }
  if (!dog_.IsHealthy() || !wheel_.IsHealthy() || !wheel_.IsStopped())
  {
    Fail(RCDog::ObstacleFault::MOTION_RUNTIME);
    return;
  }
  if (phase_ >= sequence_count_)
  {
    state_ = RCDog::ObstacleState::COMPLETE;
    return;
  }
  auto& segment = sequence_[phase_];
  if (!segment_started_)
  {
    if (!dog_.StartFootMotion(segment.target, segment.mask, segment.duration_ms,
                              segment.clearance_mm))
    {
      Fail(RCDog::ObstacleFault::MOTION_START);
      return;
    }
    segment_started_ = true;
    dwell_started_ms_ = 0;
    return;
  }
  if (!dog_.MotionComplete())
  {
    return;
  }
  if (dwell_started_ms_ == 0)
  {
    dwell_started_ms_ = now_ms;
  }
  if (now_ms - dwell_started_ms_ >= segment.dwell_ms)
  {
    ++phase_;
    segment_started_ = false;
  }
}

RCDog::ObstacleState ObstacleController::State() const
{
  return state_;
}
RCDog::ObstacleFault ObstacleController::Fault() const
{
  return fault_;
}
bool ObstacleController::CanExit() const
{
  return !segment_started_ &&
         (sequence_count_ == 0 || phase_ >= sequence_count_ ||
          state_ == RCDog::ObstacleState::FAULT);
}

void ObstacleController::BuildSequence()
{
  sequence_count_ = 0;
  const float landing = LandingHeight();
  const float raise = profile_ == RCDog::StairProfile::HIGH ? 30.0F : 0.0F;
  const float front_support = kFrontZ + raise;
  const float rear_support = kRearZ + raise;
  const float front_landing = front_support - landing;
  const float rear_landing = rear_support - landing;

  auto add = [&](const RCDog::FootTarget pose[4], uint8_t mask, uint32_t duration,
                 float clearance = 0.0F, uint32_t dwell = 300U)
  {
    auto& segment = sequence_[sequence_count_++];
    std::memcpy(segment.target, pose, sizeof(segment.target));
    segment.mask = mask;
    segment.duration_ms = duration;
    segment.clearance_mm = clearance;
    segment.dwell_ms = dwell;
  };

  RCDog::FootTarget pose[4]{};
  if (raise > 0.0F)
  {
    SetPose(pose, 0, front_support, 0, rear_support);
    add(pose, kAllLegs, 1500, 0, 0);
  }
  SetPose(pose, -kCompactX, front_support, -kCompactX, rear_support);
  add(pose, kAllLegs, 1500, 0, 0);
  pose[2] = {kCompactX, rear_support}; add(pose, kLB, 1500, 30);
  pose[3] = {kCompactX, rear_support}; add(pose, kRB, 1500, 30);

  const float front_swing = front_landing - 70.0F;
  pose[0] = {-kCompactX, front_swing}; add(pose, kLF, 1000, 0, 0);
  pose[0] = {kForwardX, front_swing}; add(pose, kLF, 1500, 0, 0);
  pose[0].z_mm = front_landing; add(pose, kLF, 1800);
  pose[1] = {-kCompactX, front_swing}; add(pose, kRF, 1000, 0, 0);
  pose[1] = {kForwardX, front_swing}; add(pose, kRF, 1500, 0, 0);
  pose[1].z_mm = front_landing; add(pose, kRF, 1800);

  SetPose(pose, -kCompactX, front_landing, kRearShiftX, rear_support);
  add(pose, kAllLegs, 2000, 0, 0);
  const float rear_swing = rear_landing - 70.0F;
  pose[2] = {kRearShiftX, rear_swing}; add(pose, kLB, 1000, 0, 0);
  pose[2] = {kCompactX, rear_swing}; add(pose, kLB, 1500, 0, 0);
  pose[2].z_mm = rear_landing; add(pose, kLB, 1800);
  pose[3] = {kRearShiftX, rear_swing}; add(pose, kRB, 1000, 0, 0);
  pose[3] = {kCompactX, rear_swing}; add(pose, kRB, 1500, 0, 0);
  pose[3].z_mm = rear_landing; add(pose, kRB, 1800);

  SetPose(pose, -kCompactX, kFrontZ, kCompactX, kRearZ);
  add(pose, kAllLegs, 1800);
  pose[0] = {kForwardX, kFrontZ}; add(pose, kLF, 1500, 30);
  pose[1] = {kForwardX, kFrontZ}; add(pose, kRF, 1500, 30);
  SetPose(pose, -kCompactX, kFrontZ, kRearShiftX, kRearZ);
  add(pose, kAllLegs, 2000, 0, 0);
  pose[2] = {kCompactX, kRearZ}; add(pose, kLB, 1500, 30);
  pose[3] = {kCompactX, kRearZ}; add(pose, kRB, 1500, 30);
  SetPose(pose, -2.0F * kCompactX, kFrontZ, 0, kRearZ);
  add(pose, kAllLegs, 1500, 0, 0);
  pose[0] = {0, kFrontZ}; add(pose, kLF, 1500, 30);
  pose[1] = {0, kFrontZ}; add(pose, kRF, 1500, 30);
}

void ObstacleController::Fail(RCDog::ObstacleFault fault)
{
  fault_ = fault;
  state_ = RCDog::ObstacleState::FAULT;
  segment_started_ = false;
  step_pending_ = false;
  sequence_count_ = 0;
  phase_ = 0;
  dwell_started_ms_ = 0;
  wheel_.StopAndLock();
  dog_.SafeStop();
}

float ObstacleController::LandingHeight() const
{
  switch (profile_)
  {
    case RCDog::StairProfile::LOW: return 50.0F;
    case RCDog::StairProfile::HIGH: return 150.0F;
    case RCDog::StairProfile::MID: default: return 100.0F;
  }
}
