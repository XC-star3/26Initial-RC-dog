#include "DogMotor.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

namespace
{
constexpr float kPi = 3.14159265358979323846F;
constexpr float kRadToDeg = 180.0F / kPi;
constexpr float kDegToRad = kPi / 180.0F;
constexpr float kStandKp = 1.7F;
constexpr float kStandKi = 0.0F;
constexpr float kStandKd = 0.017F;
constexpr float kCurrentLimitA = 18.0F;
constexpr float kTorqueNmPerA = 1.2F;
constexpr float kThighMm = 210.0F;
constexpr float kShankMm = 250.0F;
constexpr float kFrontStandMm = 300.0F;
constexpr float kRearStandMm = 315.0F;
constexpr float kFrontStartMm = 150.0F;
constexpr float kRearStartMm = 165.0F;
constexpr uint32_t kHeartbeatTimeoutMs = 250;
constexpr uint32_t kEncoderTimeoutMs = 120;
constexpr uint32_t kPeriodMs = 2;
constexpr uint32_t kIdleRetryMs = 100;
constexpr uint32_t kConfigurationTimeoutMs = 3000;
constexpr uint8_t kTrajectorySamples = 32;
constexpr uint8_t kAxisStateClosedLoop = 8;
constexpr float kTrotHz = 2.0F;
constexpr float kSwingClearanceMm = 70.0F;
constexpr float kMechanicalHipDeg = -15.0F;
constexpr float kMechanicalReverseHipDeg = 5.0F;
constexpr float kMechanicalStepDeg = 20.0F * kPeriodMs / 800.0F;

constexpr DogMotor::MotorConfig kMotors[8] = {
    {0, 0, 0, 2, 1.0F, -1.0F, 8.0F, -120.0F, 120.0F},
    {0, 1, 0, 1, -1.0F, 1.0F, 8.0F, -120.0F, 143.0F},
    {1, 0, 0, 4, -1.0F, 1.0F, 8.0F, -120.0F, 120.0F},
    {1, 1, 0, 3, 1.0F, 1.0F, 8.0F, -120.0F, 143.0F},
    {2, 0, 1, 2, 1.0F, -1.0F, 8.0F, -120.0F, 120.0F},
    {2, 1, 1, 1, -1.0F, -1.0F, 8.0F, -120.0F, 143.0F},
    {3, 0, 1, 4, -1.0F, -1.0F, 8.0F, -120.0F, 120.0F},
    {3, 1, 1, 3, 1.0F, 1.0F, 8.0F, -120.0F, 143.0F},
};

uint16_t EncodeUnsigned(float value, float low, float high, uint8_t bits)
{
  value = RCDog::Clamp(value, low, high);
  const uint32_t scale = (1U << bits) - 1U;
  return static_cast<uint16_t>((value - low) * static_cast<float>(scale) /
                               (high - low));
}

float ReadFloatLe(const uint8_t* data)
{
  float value = 0.0F;
  std::memcpy(&value, data, sizeof(value));
  return value;
}
}  // namespace

DogMotor::DogMotor(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                   uint32_t stack_size)
    : front_(*hw.FindOrExit<LibXR::FDCAN>({"fdcan_front"})),
      rear_(*hw.FindOrExit<LibXR::FDCAN>({"fdcan_rear"})),
      front_callback_(LibXR::CAN::Callback::Create(CanRxFront, this)),
      rear_callback_(LibXR::CAN::Callback::Create(CanRxRear, this))
{
  for (uint8_t leg = 0; leg < 4; ++leg)
  {
    current_foot_[leg] = {0.0F, leg >= 2 ? kRearStartMm : kFrontStartMm};
    published_foot_[leg] = current_foot_[leg];
  }
  front_.Register(front_callback_, LibXR::CAN::Type::STANDARD,
                  LibXR::CAN::FilterMode::ID_RANGE, 0, 0x7FF);
  rear_.Register(rear_callback_, LibXR::CAN::Type::STANDARD,
                 LibXR::CAN::FilterMode::ID_RANGE, 0, 0x7FF);
  app.Register(*this);
  thread_.Create(this, ThreadEntry, "dog_motor", stack_size,
                 LibXR::Thread::Priority::HIGH);
}

void DogMotor::OnMonitor()
{
  const uint32_t now = LibXR::Thread::GetTime();
  online_mask_.store(CalculateOnlineMask(now), std::memory_order_release);
}

void DogMotor::SafeStop()
{
  reconfigure_required_.store(true, std::memory_order_release);
  taskENTER_CRITICAL();
  if (pending_command_.requested_state != State::SAFE ||
      GetState() != State::SAFE)
  {
    pending_command_.requested_state = State::SAFE;
    pending_command_.gait_enabled = false;
    pending_command_.generation++;
    command_generation_.store(pending_command_.generation, std::memory_order_release);
  }
  taskEXIT_CRITICAL();
}

void DogMotor::RequestStand()
{
  taskENTER_CRITICAL();
  const State state = GetState();
  if (pending_command_.requested_state != State::STANDING ||
      (state != State::CONFIGURING && state != State::STANDING))
  {
    pending_command_.requested_state = State::STANDING;
    pending_command_.gait_enabled = false;
    pending_command_.generation++;
    command_generation_.store(pending_command_.generation, std::memory_order_release);
  }
  taskEXIT_CRITICAL();
}

void DogMotor::RequestLower()
{
  taskENTER_CRITICAL();
  const State state = GetState();
  if (pending_command_.requested_state != State::LOWERING ||
      (state != State::LOWERING && state != State::SAFE))
  {
    pending_command_.requested_state = State::LOWERING;
    pending_command_.gait_enabled = false;
    pending_command_.generation++;
    command_generation_.store(pending_command_.generation, std::memory_order_release);
  }
  taskEXIT_CRITICAL();
}

void DogMotor::RequestMechanicalPose(bool reverse)
{
  taskENTER_CRITICAL();
  const State state = GetState();
  if (pending_command_.requested_state != State::MECHANICAL ||
      pending_command_.mechanical_reverse != reverse ||
      (state != State::CONFIGURING && state != State::MECHANICAL))
  {
    pending_command_.requested_state = State::MECHANICAL;
    pending_command_.mechanical_reverse = reverse;
    pending_command_.gait_enabled = false;
    pending_command_.generation++;
    command_generation_.store(pending_command_.generation, std::memory_order_release);
  }
  taskEXIT_CRITICAL();
}

bool DogMotor::IsMechanicalPoseReady(bool reverse) const
{
  return GetState() == State::MECHANICAL &&
         mechanical_ready_.load(std::memory_order_acquire) &&
         mechanical_reverse_.load(std::memory_order_acquire) == reverse;
}

void DogMotor::SetGait(float forward, float yaw, float speed, bool enabled,
                       bool smooth_stop)
{
  if (!enabled || smooth_stop)
  {
    RequestStand();
    return;
  }
  forward = RCDog::Clamp(forward, -1.0F, 1.0F);
  yaw = RCDog::Clamp(yaw, -1.0F, 1.0F);
  speed = RCDog::Clamp(speed, 0.0F, 1.0F);
  taskENTER_CRITICAL();
  if (pending_command_.requested_state != State::GAIT ||
      pending_command_.forward != forward || pending_command_.yaw != yaw ||
      pending_command_.speed != speed ||
      pending_command_.smooth_stop != smooth_stop)
  {
    pending_command_.requested_state = State::GAIT;
    pending_command_.forward = forward;
    pending_command_.yaw = yaw;
    pending_command_.speed = speed;
    pending_command_.gait_enabled = true;
    pending_command_.smooth_stop = smooth_stop;
    pending_command_.generation++;
    command_generation_.store(pending_command_.generation,
                              std::memory_order_release);
  }
  taskEXIT_CRITICAL();
}

bool DogMotor::StartFootMotion(const RCDog::FootTarget target[4], uint8_t leg_mask,
                               uint32_t duration_ms, float clearance_mm)
{
  if (target == nullptr || leg_mask == 0 || duration_ms < kPeriodMs ||
      !IsStanding())
  {
    return false;
  }
  leg_mask &= 0x0F;
  clearance_mm = std::max(0.0F, clearance_mm);
  if (!ValidateFootMotion(target, leg_mask, clearance_mm))
  {
    return false;
  }
  taskENTER_CRITICAL();
  std::memcpy(pending_command_.foot_target, target,
              sizeof(pending_command_.foot_target));
  pending_command_.foot_mask = leg_mask;
  pending_command_.foot_duration_ms = duration_ms;
  pending_command_.foot_clearance_mm = clearance_mm;
  pending_command_.requested_state = State::FOOT_MOTION;
  motion_complete_.store(false, std::memory_order_release);
  pending_command_.generation++;
  command_generation_.store(pending_command_.generation, std::memory_order_release);
  taskEXIT_CRITICAL();
  return true;
}

bool DogMotor::MotionComplete() const
{
  return motion_complete_.load(std::memory_order_acquire);
}
bool DogMotor::IsStanding() const
{
  return GetState() == State::STANDING;
}
bool DogMotor::IsSafe() const { return GetState() == State::SAFE; }
bool DogMotor::IsHealthy() const
{
  return OnlineMask() == 0xFF && FaultBits() == RCDog::FAULT_NONE;
}
uint8_t DogMotor::OnlineMask() const
{
  return static_cast<uint8_t>(online_mask_.load(std::memory_order_acquire));
}
uint32_t DogMotor::FaultBits() const
{
  return fault_bits_.load(std::memory_order_acquire);
}
DogMotor::State DogMotor::GetState() const
{
  return static_cast<State>(published_state_.load(std::memory_order_acquire));
}

void DogMotor::ThreadEntry(DogMotor* self) { self->Run(); }

void DogMotor::CanRxFront(bool in_isr, DogMotor* self,
                          const LibXR::CAN::ClassicPack& pack)
{
  if (in_isr)
  {
    const UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
    self->HandleCan(pack, 0);
    taskEXIT_CRITICAL_FROM_ISR(mask);
  }
  else
  {
    taskENTER_CRITICAL();
    self->HandleCan(pack, 0);
    taskEXIT_CRITICAL();
  }
}

void DogMotor::CanRxRear(bool in_isr, DogMotor* self,
                         const LibXR::CAN::ClassicPack& pack)
{
  if (in_isr)
  {
    const UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
    self->HandleCan(pack, 1);
    taskEXIT_CRITICAL_FROM_ISR(mask);
  }
  else
  {
    taskENTER_CRITICAL();
    self->HandleCan(pack, 1);
    taskEXIT_CRITICAL();
  }
}

void DogMotor::Run()
{
  LibXR::MillisecondTimestamp wake(LibXR::Thread::GetTime());
  while (true)
  {
    ControlTick(LibXR::Thread::GetTime());
    LibXR::Thread::SleepUntil(wake, kPeriodMs);
  }
}

void DogMotor::HandleCan(const LibXR::CAN::ClassicPack& pack, uint8_t bus)
{
  if (pack.dlc != 8 || pack.type != LibXR::CAN::Type::STANDARD)
  {
    return;
  }
  const uint8_t node = static_cast<uint8_t>(pack.id >> 5U);
  const uint8_t command = static_cast<uint8_t>(pack.id & 0x1FU);
  const uint32_t now = LibXR::Thread::GetTime();
  for (uint8_t i = 0; i < 8; ++i)
  {
    if (kMotors[i].node_id != node || kMotors[i].bus != bus)
    {
      continue;
    }
    if (command == 0x01)
    {
      std::memcpy(&feedback_[i].axis_error, pack.data, sizeof(uint32_t));
      feedback_[i].axis_state = pack.data[4];
      feedback_[i].fault_flags = pack.data[5] & 0x0FU;
      feedback_[i].last_heartbeat_ms = now;
    }
    else if (command == 0x09)
    {
      feedback_[i].encoder_turn = ReadFloatLe(pack.data);
      feedback_[i].velocity_turn_s = ReadFloatLe(pack.data + 4);
      feedback_[i].last_encoder_ms = now;
      if (!feedback_[i].zero_valid)
      {
        feedback_[i].zero_turn = feedback_[i].encoder_turn;
        feedback_[i].zero_valid = true;
      }
    }
  }
}

void DogMotor::ControlTick(uint32_t now_ms)
{
  if (command_generation_.load(std::memory_order_acquire) != applied_generation_)
  {
    Command snapshot{};
    taskENTER_CRITICAL();
    snapshot = pending_command_;
    taskEXIT_CRITICAL();
    ApplyCommand(snapshot, now_ms);
    applied_generation_ = snapshot.generation;
  }

  MotorFeedback feedback[8]{};
  taskENTER_CRITICAL();
  std::memcpy(feedback, feedback_, sizeof(feedback));
  taskEXIT_CRITICAL();
  uint8_t online = 0;
  uint8_t closed_loop = 0;
  uint8_t idle = 0;
  for (uint8_t i = 0; i < 8; ++i)
  {
    if (feedback[i].last_heartbeat_ms != 0 && feedback[i].last_encoder_ms != 0 &&
        now_ms - feedback[i].last_heartbeat_ms <= kHeartbeatTimeoutMs &&
        now_ms - feedback[i].last_encoder_ms <= kEncoderTimeoutMs)
    {
      online |= static_cast<uint8_t>(1U << i);
      if (feedback[i].axis_state == kAxisStateClosedLoop)
      {
        closed_loop |= static_cast<uint8_t>(1U << i);
      }
      else if (feedback[i].axis_state == 1U)
      {
        idle |= static_cast<uint8_t>(1U << i);
      }
    }
  }
  online_mask_.store(online, std::memory_order_release);
  closed_loop_mask_.store(closed_loop, std::memory_order_release);
  uint32_t faults = RCDog::FAULT_NONE;
  if (online != 0xFF)
  {
    faults |= RCDog::FAULT_LEG_OFFLINE;
  }
  for (const auto& fb : feedback)
  {
    if (fb.axis_error != 0 || fb.fault_flags != 0)
    {
      faults |= RCDog::FAULT_LEG_DRIVE;
    }
  }
  if (now_ms - last_bus_check_ms_ >= 100U)
  {
    LibXR::CAN::ErrorState front_error{};
    LibXR::CAN::ErrorState rear_error{};
    const bool front_ok = front_.GetErrorState(front_error) == LibXR::ErrorCode::OK;
    const bool rear_ok = rear_.GetErrorState(rear_error) == LibXR::ErrorCode::OK;
    bus_off_ = (front_ok && front_error.bus_off) ||
               (rear_ok && rear_error.bus_off);
    last_bus_check_ms_ = now_ms;
  }
  if (bus_off_)
  {
    faults |= RCDog::FAULT_CAN_BUS_OFF;
  }
  if (motion_fault_)
  {
    faults |= RCDog::FAULT_LEG_DRIVE;
  }
  const State state_before_fault = GetState();
  if (state_before_fault != State::SAFE &&
      state_before_fault != State::CONFIGURING && closed_loop != 0xFF)
  {
    closed_loop_fault_ = true;
  }
  if (state_before_fault == State::CONFIGURING &&
      configuration_started_ms_ != 0 &&
      now_ms - configuration_started_ms_ > kConfigurationTimeoutMs &&
      (configuration_wait_idle_ || closed_loop != 0xFF))
  {
    closed_loop_fault_ = true;
  }
  if (closed_loop_fault_)
  {
    faults |= RCDog::FAULT_LEG_DRIVE;
  }
  fault_bits_.store(faults, std::memory_order_release);

  const bool configuring_offline = state_before_fault == State::CONFIGURING &&
      (faults == RCDog::FAULT_NONE || faults == RCDog::FAULT_LEG_OFFLINE);
  if (faults != RCDog::FAULT_NONE && state_before_fault != State::SAFE &&
      state_before_fault != State::FAULT && !configuring_offline)
  {
    published_state_.store(static_cast<uint32_t>(State::FAULT),
                           std::memory_order_release);
  }

  const State control_state = GetState();
  const bool inactive = control_state == State::SAFE || control_state == State::FAULT;
  if (inactive && (last_control_state_ != control_state ||
                   now_ms - last_idle_ms_ >= kIdleRetryMs))
  {
    for (uint8_t i = 0; i < 8; ++i)
    {
      SendIdle(i);
    }
    last_idle_ms_ = now_ms;
  }
  last_control_state_ = control_state;

  UpdateTargets(now_ms);
  for (uint8_t i = 0; i < 8; ++i)
  {
    const State send_state = GetState();
    if ((closed_loop & (1U << i)) != 0 && send_state != State::SAFE &&
        send_state != State::FAULT &&
        (send_state != State::CONFIGURING ||
         (!configuration_wait_idle_ && closed_loop == 0xFF)))
    {
      SendMotor(i, target_deg_[i], 0.002F);
    }
  }
  SendQuery(query_cursor_);
  query_cursor_ = static_cast<uint8_t>((query_cursor_ + 1U) % 8U);
  if (control_state == State::CONFIGURING &&
      now_ms - last_setup_ms_ >= 25U)
  {
    if (configuration_wait_idle_ && idle == 0xFF)
    {
      configuration_wait_idle_ = false;
    }
    for (uint8_t attempt = 0;
         attempt < 8 &&
         (configuration_wait_idle_ || closed_loop != 0xFF);
         ++attempt)
    {
      const uint8_t index = static_cast<uint8_t>(setup_cursor_++ % 8U);
      if (configuration_wait_idle_ && (idle & (1U << index)) == 0)
      {
        SendIdle(index);
        break;
      }
      if (!configuration_wait_idle_ &&
          (closed_loop & (1U << index)) == 0)
      {
        SendSetup(index);
        break;
      }
    }
    last_setup_ms_ = now_ms;
  }
}

void DogMotor::ApplyCommand(const Command& command, uint32_t now_ms)
{
  active_command_ = command;
  mechanical_ready_.store(false, std::memory_order_release);
  if (command.requested_state == State::SAFE)
  {
    motion_fault_ = false;
    closed_loop_fault_ = false;
    motion_.active = false;
    motion_.complete = false;
    motion_complete_.store(false, std::memory_order_release);
    configuration_started_ms_ = 0;
    configuration_wait_idle_ = false;
    reconfigure_required_.store(true, std::memory_order_release);
  }
  if (command.requested_state == State::FOOT_MOTION)
  {
    if (!ValidateFootMotionFrom(current_foot_, command.foot_target,
                                command.foot_mask,
                                command.foot_clearance_mm))
    {
      motion_fault_ = true;
      motion_.active = false;
      motion_.complete = false;
      motion_complete_.store(false, std::memory_order_release);
      published_state_.store(static_cast<uint32_t>(State::FAULT),
                             std::memory_order_release);
      return;
    }
    std::memcpy(motion_.start, current_foot_, sizeof(motion_.start));
    std::memcpy(motion_.target, command.foot_target, sizeof(motion_.target));
    motion_.mask = command.foot_mask;
    motion_.start_ms = now_ms;
    motion_.duration_ms = command.foot_duration_ms;
    motion_.clearance_mm = command.foot_clearance_mm;
    motion_.active = true;
    motion_.complete = false;
    motion_complete_.store(false, std::memory_order_release);
  }
  else if (command.requested_state == State::GAIT && gait_epoch_ms_ == 0)
  {
    gait_epoch_ms_ = now_ms;
  }
  else if (command.requested_state != State::GAIT)
  {
    gait_epoch_ms_ = 0;
  }
  const bool force_reconfigure = command.requested_state != State::SAFE &&
      reconfigure_required_.exchange(false, std::memory_order_acq_rel);
  const bool needs_configuration = command.requested_state != State::SAFE &&
      command.requested_state != State::FAULT &&
      (force_reconfigure ||
       closed_loop_mask_.load(std::memory_order_acquire) != 0xFF);
  const State state = command.requested_state == State::STANDING ||
                              needs_configuration
                          ? State::CONFIGURING
                          : command.requested_state;
  if (state == State::CONFIGURING && GetState() != State::CONFIGURING)
  {
    configuration_started_ms_ = now_ms;
    configuration_wait_idle_ = force_reconfigure;
    if (configuration_wait_idle_)
    {
      for (uint8_t i = 0; i < 8; ++i)
      {
        SendIdle(i);
      }
    }
  }
  else if (state != State::CONFIGURING)
  {
    configuration_started_ms_ = 0;
  }
  published_state_.store(static_cast<uint32_t>(state),
                         std::memory_order_release);
}

void DogMotor::UpdateTargets(uint32_t now_ms)
{
  const State state = GetState();
  if (state == State::SAFE || state == State::FAULT)
  {
    std::fill(std::begin(integral_), std::end(integral_), 0.0F);
    return;
  }

  if (state == State::FOOT_MOTION)
  {
    UpdateFootTargets(now_ms);
  }
  else if (state == State::LOWERING || state == State::STANDING ||
           state == State::CONFIGURING)
  {
    const bool all_closed = !configuration_wait_idle_ &&
        closed_loop_mask_.load(std::memory_order_acquire) == 0xFF;
    const float end_front = state == State::LOWERING ? kFrontStartMm : kFrontStandMm;
    const float end_rear = state == State::LOWERING ? kRearStartMm : kRearStandMm;
    for (uint8_t leg = 0; leg < 4; ++leg)
    {
      const float target = leg >= 2 ? end_rear : end_front;
      if (state != State::CONFIGURING || all_closed)
      {
        const float delta =
            RCDog::Clamp(target - current_foot_[leg].z_mm, -0.4F, 0.4F);
        current_foot_[leg].x_mm +=
            RCDog::Clamp(-current_foot_[leg].x_mm, -0.4F, 0.4F);
        current_foot_[leg].z_mm += delta;
      }
    }
    if (state == State::LOWERING)
    {
      bool done = true;
      for (uint8_t leg = 0; leg < 4; ++leg)
      {
        const float target = leg >= 2 ? kRearStartMm : kFrontStartMm;
        done &= std::fabs(current_foot_[leg].z_mm - target) < 1.0F;
      }
      if (done)
      {
        published_state_.store(static_cast<uint32_t>(State::SAFE),
                               std::memory_order_release);
      }
    }
    else if (state == State::CONFIGURING)
    {
      bool done = all_closed &&
                  online_mask_.load(std::memory_order_acquire) == 0xFF;
      MotorFeedback feedback[8]{};
      taskENTER_CRITICAL();
      std::memcpy(feedback, feedback_, sizeof(feedback));
      taskEXIT_CRITICAL();
      for (uint8_t leg = 0; leg < 4; ++leg)
      {
        const float target = leg >= 2 ? kRearStandMm : kFrontStandMm;
        done &= std::fabs(current_foot_[leg].z_mm - target) < 1.0F;
      }
      for (const auto& fb : feedback)
      {
        done &= fb.axis_state == kAxisStateClosedLoop;
      }
      if (done)
      {
        configuration_started_ms_ = 0;
        const State next = active_command_.requested_state == State::STANDING
                               ? State::STANDING
                               : active_command_.requested_state;
        published_state_.store(static_cast<uint32_t>(next),
                               std::memory_order_release);
      }
    }
  }
  else if (state == State::MECHANICAL)
  {
    const float hip = active_command_.mechanical_reverse
                          ? kMechanicalReverseHipDeg
                          : kMechanicalHipDeg;
    for (uint8_t leg = 0; leg < 4; ++leg)
    {
      const uint8_t hip_index = leg * 2;
      const uint8_t knee_index = hip_index + 1;
      const float hip_delta = RCDog::Clamp(hip - target_deg_[hip_index],
                                           -kMechanicalStepDeg,
                                           kMechanicalStepDeg);
      const float knee_delta = RCDog::Clamp(-target_deg_[knee_index],
                                            -kMechanicalStepDeg,
                                            kMechanicalStepDeg);
      target_deg_[hip_index] += hip_delta;
      target_deg_[knee_index] += knee_delta;
    }
    bool ready = true;
    for (uint8_t leg = 0; leg < 4; ++leg)
    {
      ready &= std::fabs(target_deg_[leg * 2] - hip) <= 0.5F;
      ready &= std::fabs(target_deg_[leg * 2 + 1]) <= 0.5F;
    }
    mechanical_reverse_.store(active_command_.mechanical_reverse,
                              std::memory_order_release);
    mechanical_ready_.store(ready, std::memory_order_release);
    return;
  }
  else if (state == State::GAIT)
  {
    const float frequency = 1.2F + active_command_.speed * (kTrotHz - 1.2F);
    const float cycle_ms = 1000.0F / frequency;
    const float phase = std::fmod(static_cast<float>(now_ms - gait_epoch_ms_), cycle_ms) /
                        cycle_ms;
    const float stride = 20.0F + 45.0F * active_command_.speed;
    for (uint8_t leg = 0; leg < 4; ++leg)
    {
      const bool pair_a = leg == 0 || leg == 3;
      float p = std::fmod(phase + (pair_a ? 0.0F : 0.5F), 1.0F);
      const bool swing = p < 0.5F;
      const float q = swing ? p * 2.0F : (p - 0.5F) * 2.0F;
      const float side = (leg == 0 || leg == 2) ? 1.0F : -1.0F;
      const float drive = active_command_.forward + active_command_.yaw * side;
      current_foot_[leg].x_mm = drive * stride * (swing ? (-0.5F + q) : (0.5F - q));
      const float stand_z = leg >= 2 ? kRearStandMm : kFrontStandMm;
      current_foot_[leg].z_mm = stand_z -
          (swing ? kSwingClearanceMm * std::sin(kPi * q) : 0.0F);
    }
  }

  for (uint8_t leg = 0; leg < 4; ++leg)
  {
    float hip = 0.0F;
    float knee = 0.0F;
    if (InverseKinematics(current_foot_[leg].x_mm, current_foot_[leg].z_mm,
                          hip, knee))
    {
      target_deg_[leg * 2] = hip;
      target_deg_[leg * 2 + 1] = knee;
    }
    else
    {
      motion_fault_ = true;
      fault_bits_.fetch_or(RCDog::FAULT_LEG_DRIVE, std::memory_order_release);
      motion_.active = false;
      motion_.complete = false;
      motion_complete_.store(false, std::memory_order_release);
      published_state_.store(static_cast<uint32_t>(State::FAULT),
                             std::memory_order_release);
      return;
    }
  }
  taskENTER_CRITICAL();
  std::memcpy(published_foot_, current_foot_, sizeof(published_foot_));
  taskEXIT_CRITICAL();
}

void DogMotor::UpdateFootTargets(uint32_t now_ms)
{
  if (!motion_.active)
  {
    return;
  }
  const float p = RCDog::Clamp(static_cast<float>(now_ms - motion_.start_ms) /
                                  static_cast<float>(motion_.duration_ms),
                              0.0F, 1.0F);
  const float s = Smooth(p);
  for (uint8_t leg = 0; leg < 4; ++leg)
  {
    if ((motion_.mask & (1U << leg)) == 0)
    {
      continue;
    }
    current_foot_[leg].x_mm = motion_.start[leg].x_mm +
        (motion_.target[leg].x_mm - motion_.start[leg].x_mm) * s;
    current_foot_[leg].z_mm = motion_.start[leg].z_mm +
        (motion_.target[leg].z_mm - motion_.start[leg].z_mm) * s -
        motion_.clearance_mm * std::sin(kPi * p);
  }
  if (p >= 1.0F)
  {
    motion_.active = false;
    motion_.complete = true;
    motion_complete_.store(true, std::memory_order_release);
    published_state_.store(static_cast<uint32_t>(State::STANDING),
                           std::memory_order_release);
  }
}

void DogMotor::SendMotor(uint8_t index, float target_deg, float dt_s)
{
  const auto& cfg = kMotors[index];
  MotorFeedback fb{};
  taskENTER_CRITICAL();
  fb = feedback_[index];
  taskEXIT_CRITICAL();
  const float user_deg = ((fb.encoder_turn - fb.zero_turn) * 360.0F * cfg.direction) /
                         cfg.ratio;
  target_deg = RCDog::Clamp(target_deg, cfg.min_deg, cfg.max_deg);
  const float error = target_deg - user_deg;
  integral_[index] = RCDog::Clamp(integral_[index] + error * dt_s, -2.0F, 2.0F);
  const float derivative = (error - previous_error_[index]) / dt_s;
  previous_error_[index] = error;
  const float current = RCDog::Clamp(kStandKp * error + kStandKi * integral_[index] +
                                         kStandKd * derivative,
                                     -kCurrentLimitA, kCurrentLimitA);
  const float motor_position = (fb.zero_turn +
      target_deg * cfg.ratio / (360.0F * cfg.direction)) * 2.0F * kPi;
  const float torque = current * kTorqueNmPerA * cfg.torque_direction;
  const uint16_t pos = EncodeUnsigned(motor_position, -12.5F, 12.5F, 16);
  const uint16_t vel = EncodeUnsigned(0.0F, -65.0F, 65.0F, 12);
  const uint16_t kp = EncodeUnsigned(0.0F, 0.0F, 500.0F, 12);
  const uint16_t kd = EncodeUnsigned(0.0F, 0.0F, 5.0F, 12);
  const uint16_t tq = EncodeUnsigned(torque, -50.0F, 50.0F, 12);
  LibXR::CAN::ClassicPack pack{static_cast<uint32_t>((cfg.node_id << 5U) | 0x08U),
                               LibXR::CAN::Type::STANDARD, 8, {}};
  pack.data[0] = static_cast<uint8_t>(pos >> 8U);
  pack.data[1] = static_cast<uint8_t>(pos);
  pack.data[2] = static_cast<uint8_t>(vel >> 4U);
  pack.data[3] = static_cast<uint8_t>((vel << 4U) | (kp >> 8U));
  pack.data[4] = static_cast<uint8_t>(kp);
  pack.data[5] = static_cast<uint8_t>(kd >> 4U);
  pack.data[6] = static_cast<uint8_t>((kd << 4U) | (tq >> 8U));
  pack.data[7] = static_cast<uint8_t>(tq);
  (void)(cfg.bus == 0 ? front_ : rear_).AddMessage(pack);
}

void DogMotor::SendIdle(uint8_t index)
{
  const auto& cfg = kMotors[index];
  LibXR::CAN::ClassicPack pack{static_cast<uint32_t>((cfg.node_id << 5U) | 0x07U),
                               LibXR::CAN::Type::STANDARD, 8, {}};
  constexpr uint32_t idle = 1;
  std::memcpy(pack.data, &idle, sizeof(idle));
  (void)(cfg.bus == 0 ? front_ : rear_).AddMessage(pack);
}

void DogMotor::SendQuery(uint8_t index)
{
  const auto& cfg = kMotors[index];
  LibXR::CAN::ClassicPack pack{static_cast<uint32_t>((cfg.node_id << 5U) | 0x09U),
                               LibXR::CAN::Type::STANDARD, 8, {}};
  (void)(cfg.bus == 0 ? front_ : rear_).AddMessage(pack);
}

void DogMotor::SendSetup(uint8_t index)
{
  const auto& cfg = kMotors[index];
  LibXR::CAN::ClassicPack pack{static_cast<uint32_t>((cfg.node_id << 5U) | 0x0FU),
                               LibXR::CAN::Type::STANDARD, 8, {}};
  const float velocity_limit = 30.0F;
  std::memcpy(pack.data, &velocity_limit, 4);
  std::memcpy(pack.data + 4, &kCurrentLimitA, 4);
  auto& bus = cfg.bus == 0 ? front_ : rear_;
  (void)bus.AddMessage(pack);
  pack.id = static_cast<uint32_t>((cfg.node_id << 5U) | 0x0BU);
  const uint32_t torque_mode = 1;
  const uint32_t mit_input = 9;
  std::memcpy(pack.data, &torque_mode, 4);
  std::memcpy(pack.data + 4, &mit_input, 4);
  (void)bus.AddMessage(pack);
  pack.id = static_cast<uint32_t>((cfg.node_id << 5U) | 0x07U);
  const uint32_t closed_loop = 8;
  std::memset(pack.data, 0, sizeof(pack.data));
  std::memcpy(pack.data, &closed_loop, 4);
  (void)bus.AddMessage(pack);
}

bool DogMotor::InverseKinematics(float x_mm, float z_mm, float& hip_deg,
                                 float& knee_deg) const
{
  const float length = std::sqrt(x_mm * x_mm + z_mm * z_mm);
  if (length < std::fabs(kShankMm - kThighMm) ||
      length > kShankMm + kThighMm)
  {
    return false;
  }
  const float range = std::asin(RCDog::Clamp(x_mm / length, -1.0F, 1.0F));
  const float separate = std::acos(RCDog::Clamp(
      (length * length + kThighMm * kThighMm - kShankMm * kShankMm) /
          (2.0F * kThighMm * length),
      -1.0F, 1.0F));
  // Calibration points (-33,33)->(0,135.6), (0,0)->(80,110).
  constexpr float hip_zero = 85.69F * kDegToRad;
  constexpr float knee_zero = 86.31F * kDegToRad;
  hip_deg = (hip_zero - (separate - range)) * kRadToDeg;
  knee_deg = (knee_zero - (separate + range)) * kRadToDeg;
  return true;
}

bool DogMotor::ValidateFootMotion(const RCDog::FootTarget target[4],
                                  uint8_t leg_mask, float clearance_mm) const
{
  RCDog::FootTarget start[4]{};
  taskENTER_CRITICAL();
  std::memcpy(start, published_foot_, sizeof(start));
  taskEXIT_CRITICAL();
  return ValidateFootMotionFrom(start, target, leg_mask, clearance_mm);
}

bool DogMotor::ValidateFootMotionFrom(const RCDog::FootTarget start[4],
                                      const RCDog::FootTarget target[4],
                                      uint8_t leg_mask,
                                      float clearance_mm) const
{
  for (uint8_t leg = 0; leg < 4; ++leg)
  {
    if ((leg_mask & (1U << leg)) == 0)
    {
      continue;
    }
    for (uint8_t sample = 0; sample <= kTrajectorySamples; ++sample)
    {
      const float p = static_cast<float>(sample) /
                      static_cast<float>(kTrajectorySamples);
      const float s = Smooth(p);
      const float x = start[leg].x_mm +
                      (target[leg].x_mm - start[leg].x_mm) * s;
      const float z = start[leg].z_mm +
                      (target[leg].z_mm - start[leg].z_mm) * s -
                      clearance_mm * std::sin(kPi * p);
      float hip = 0.0F;
      float knee = 0.0F;
      if (!InverseKinematics(x, z, hip, knee) ||
          hip < kMotors[leg * 2].min_deg ||
          hip > kMotors[leg * 2].max_deg ||
          knee < kMotors[leg * 2 + 1].min_deg ||
          knee > kMotors[leg * 2 + 1].max_deg)
      {
        return false;
      }
    }
  }
  return true;
}

uint8_t DogMotor::CalculateOnlineMask(uint32_t now_ms) const
{
  MotorFeedback feedback[8]{};
  taskENTER_CRITICAL();
  std::memcpy(feedback, feedback_, sizeof(feedback));
  taskEXIT_CRITICAL();
  uint8_t mask = 0;
  for (uint8_t i = 0; i < 8; ++i)
  {
    if (feedback[i].last_heartbeat_ms != 0 && feedback[i].last_encoder_ms != 0 &&
        now_ms - feedback[i].last_heartbeat_ms <= kHeartbeatTimeoutMs &&
        now_ms - feedback[i].last_encoder_ms <= kEncoderTimeoutMs)
    {
      mask |= static_cast<uint8_t>(1U << i);
    }
  }
  return mask;
}

float DogMotor::Smooth(float x) { return x * x * (3.0F - 2.0F * x); }
