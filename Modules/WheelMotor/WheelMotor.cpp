#include "WheelMotor.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

namespace
{
constexpr float kPi = 3.14159265358979323846F;
constexpr float kRpmToRadS = 2.0F * kPi / 60.0F;
constexpr float kGearRatio = 268.0F / 17.0F;
constexpr float kForwardSign[4] = {-1.0F, 1.0F, 1.0F, -1.0F};
constexpr float kKp = 0.015F;
constexpr float kKi = 0.25F;
constexpr float kAccel = 30.0F;
constexpr float kDecel = 50.0F;
constexpr float kBrake = 60.0F;
constexpr int32_t kContinuousRaw = static_cast<int32_t>(6.0F / 20.0F * 16384.0F);
constexpr int32_t kPeakRaw = static_cast<int32_t>(10.0F / 20.0F * 16384.0F);
constexpr uint32_t kPeakMs = 500;
constexpr uint8_t kDerateC = 70;
constexpr uint8_t kCutoffC = 85;
constexpr uint8_t kRecoverC = 65;
constexpr uint32_t kCommandTimeoutMs = 100;
constexpr uint32_t kFeedbackTimeoutMs = 250;
constexpr uint32_t kPeriodMs = 2;
constexpr float kQuickTurnFull = 0.05F;
constexpr float kQuickTurnEnd = 0.25F;
constexpr float kStrongTurnStart = 0.45F;
constexpr float kStrongTurnFull = 0.85F;
constexpr float kStrongTurnForwardScale = 0.5F;
constexpr float kZeroEpsilon = 0.0001F;

float Approach(float current, float target, float delta)
{
  if (current < target)
  {
    return std::min(current + delta, target);
  }
  return std::max(current - delta, target);
}

float SmoothStep(float value)
{
  value = RCDog::Clamp(value, 0.0F, 1.0F);
  return value * value * (3.0F - 2.0F * value);
}
}  // namespace

WheelMotor::WheelMotor(LibXR::HardwareContainer& hw,
                       LibXR::ApplicationManager& app, uint32_t stack_size)
    : can_(*hw.FindOrExit<LibXR::FDCAN>({"fdcan_wheel"})),
      can_callback_(LibXR::CAN::Callback::Create(CanRx, this))
{
  can_.Register(can_callback_, LibXR::CAN::Type::STANDARD,
                LibXR::CAN::FilterMode::ID_RANGE, 0x201, 0x204);
  app.Register(*this);
  thread_.Create(this, ThreadEntry, "wheel_motor", stack_size,
                 LibXR::Thread::Priority::HIGH);
}

void WheelMotor::OnMonitor()
{
  online_mask_.store(CalculateOnlineMask(LibXR::Thread::GetTime()),
                     std::memory_order_release);
}

void WheelMotor::SetMotion(float forward, float yaw, float max_rpm)
{
  forward = RCDog::Clamp(forward, -1.0F, 1.0F);
  yaw = RCDog::Clamp(yaw, -1.0F, 1.0F);
  max_rpm = RCDog::Clamp(max_rpm, 0.0F, 200.0F);
  const float speed = std::fabs(forward);
  float quick_turn = 0.0F;
  if (speed <= kQuickTurnFull)
  {
    quick_turn = 1.0F;
  }
  else if (speed < kQuickTurnEnd)
  {
    quick_turn = (kQuickTurnEnd - speed) / (kQuickTurnEnd - kQuickTurnFull);
  }
  const float yaw_scale = speed + quick_turn * (1.0F - speed);
  const float strong_turn = SmoothStep((std::fabs(yaw) - kStrongTurnStart) /
                                       (kStrongTurnFull - kStrongTurnStart));
  const float turn_scale = yaw_scale + strong_turn * (1.0F - yaw_scale);
  const float forward_scale = 1.0F - strong_turn *
      (1.0F - kStrongTurnForwardScale);
  float left = forward * forward_scale + yaw * turn_scale;
  float right = forward * forward_scale - yaw * turn_scale;
  const float mix = std::max(1.0F, std::max(std::fabs(left), std::fabs(right)));
  left = left / mix * max_rpm;
  right = right / mix * max_rpm;
  const float targets[4] = {left, right, right, left};
  const float scale[4] = {1.0F, 1.0F, 1.0F, 1.0F};
  SetTargets(targets, scale);
}

void WheelMotor::SetTargets(const float rpm[4], const float scale[4])
{
  if (rpm == nullptr || scale == nullptr)
  {
    return;
  }
  taskENTER_CRITICAL();
  for (uint8_t i = 0; i < 4; ++i)
  {
    command_.target_rpm[i] = RCDog::Clamp(rpm[i], -200.0F, 200.0F);
    command_.scale[i] = RCDog::Clamp(scale[i], 0.0F, 1.0F);
  }
  command_.generation++;
  command_generation_.store(command_.generation, std::memory_order_release);
  last_command_ms_ = LibXR::Thread::GetTime();
  taskEXIT_CRITICAL();
}

void WheelMotor::SetMode(Mode mode)
{
  taskENTER_CRITICAL();
  if (command_.mode != mode)
  {
    command_.mode = mode;
    if (mode != Mode::DRIVE)
    {
      std::fill(std::begin(command_.target_rpm), std::end(command_.target_rpm),
                0.0F);
    }
    command_.generation++;
    command_generation_.store(command_.generation, std::memory_order_release);
  }
  last_command_ms_ = LibXR::Thread::GetTime();
  taskEXIT_CRITICAL();
}

void WheelMotor::StopAndLock()
{
  taskENTER_CRITICAL();
  if (command_.mode != Mode::OFF || !command_.locked)
  {
    command_.mode = Mode::OFF;
    command_.locked = true;
    std::fill(std::begin(command_.target_rpm), std::end(command_.target_rpm),
              0.0F);
    command_.generation++;
    command_generation_.store(command_.generation, std::memory_order_release);
  }
  taskEXIT_CRITICAL();
}

bool WheelMotor::TryClearLock()
{
  if (OnlineMask() != 0x0F || !IsStopped() || FaultBits() != RCDog::FAULT_NONE)
  {
    return false;
  }
  taskENTER_CRITICAL();
  if (!command_.locked)
  {
    last_command_ms_ = LibXR::Thread::GetTime();
    taskEXIT_CRITICAL();
    return true;
  }
  command_.locked = false;
  command_.mode = Mode::HOLD;
  command_.generation++;
  command_generation_.store(command_.generation, std::memory_order_release);
  last_command_ms_ = LibXR::Thread::GetTime();
  taskEXIT_CRITICAL();
  return true;
}

bool WheelMotor::IsStopped() const
{
  Feedback feedback[4]{};
  taskENTER_CRITICAL();
  std::memcpy(feedback, feedback_, sizeof(feedback));
  taskEXIT_CRITICAL();
  for (const auto& fb : feedback)
  {
    if (std::fabs(static_cast<float>(fb.speed_rpm) / kGearRatio * kRpmToRadS) > 0.5F)
    {
      return false;
    }
  }
  return true;
}
bool WheelMotor::IsHealthy() const
{
  return OnlineMask() == 0x0F && FaultBits() == RCDog::FAULT_NONE;
}
uint8_t WheelMotor::OnlineMask() const
{
  return static_cast<uint8_t>(online_mask_.load(std::memory_order_acquire));
}
uint8_t WheelMotor::ThermalDeratedMask() const
{
  return static_cast<uint8_t>(derated_mask_.load(std::memory_order_acquire));
}
uint32_t WheelMotor::FaultBits() const
{
  return fault_bits_.load(std::memory_order_acquire);
}

void WheelMotor::ThreadEntry(WheelMotor* self) { self->Run(); }

void WheelMotor::CanRx(bool in_isr, WheelMotor* self,
                       const LibXR::CAN::ClassicPack& pack)
{
  if (in_isr)
  {
    const UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
    self->HandleCan(pack);
    taskEXIT_CRITICAL_FROM_ISR(mask);
  }
  else
  {
    taskENTER_CRITICAL();
    self->HandleCan(pack);
    taskEXIT_CRITICAL();
  }
}

void WheelMotor::Run()
{
  LibXR::MillisecondTimestamp wake(LibXR::Thread::GetTime());
  while (true)
  {
    Tick(LibXR::Thread::GetTime());
    LibXR::Thread::SleepUntil(wake, kPeriodMs);
  }
}

void WheelMotor::HandleCan(const LibXR::CAN::ClassicPack& pack)
{
  if (pack.dlc != 8 || pack.id < 0x201 || pack.id > 0x204)
  {
    return;
  }
  const uint8_t index = static_cast<uint8_t>(pack.id - 0x201);
  feedback_[index].encoder = static_cast<uint16_t>(pack.data[0] << 8U | pack.data[1]);
  feedback_[index].speed_rpm = static_cast<int16_t>(pack.data[2] << 8U | pack.data[3]);
  feedback_[index].current_raw = static_cast<int16_t>(pack.data[4] << 8U | pack.data[5]);
  feedback_[index].temperature_c = pack.data[6];
  feedback_[index].last_update_ms = LibXR::Thread::GetTime();
}

void WheelMotor::Tick(uint32_t now_ms)
{
  Command command{};
  uint32_t last_command_ms = 0;
  taskENTER_CRITICAL();
  command = command_;
  last_command_ms = last_command_ms_;
  taskEXIT_CRITICAL();
  applied_generation_ = command.generation;

  Feedback feedback[4]{};
  taskENTER_CRITICAL();
  std::memcpy(feedback, feedback_, sizeof(feedback));
  taskEXIT_CRITICAL();

  uint8_t online = 0;
  for (uint8_t i = 0; i < 4; ++i)
  {
    if (feedback[i].last_update_ms != 0 &&
        now_ms - feedback[i].last_update_ms <= kFeedbackTimeoutMs)
    {
      online |= static_cast<uint8_t>(1U << i);
    }
  }
  online_mask_.store(online, std::memory_order_release);
  uint32_t faults = online == 0x0F ? RCDog::FAULT_NONE : RCDog::FAULT_WHEEL_OFFLINE;
  uint8_t derated = 0;
  bool overtemp = false;
  for (uint8_t i = 0; i < 4; ++i)
  {
    if (feedback[i].temperature_c >= kCutoffC)
    {
      overtemp_latched_[i] = true;
    }
    else if (feedback[i].temperature_c <= kRecoverC)
    {
      overtemp_latched_[i] = false;
    }
    if (feedback[i].temperature_c > kDerateC)
    {
      derated |= static_cast<uint8_t>(1U << i);
    }
    overtemp |= overtemp_latched_[i];
  }
  if (overtemp)
  {
    faults |= RCDog::FAULT_WHEEL_OVERTEMP;
  }
  derated_mask_.store(derated, std::memory_order_release);

  if (now_ms - last_bus_check_ms_ >= 100U)
  {
    LibXR::CAN::ErrorState error{};
    if (can_.GetErrorState(error) == LibXR::ErrorCode::OK && error.bus_off)
    {
      bus_off_ = true;
      ++bus_off_count_;
    }
    else
    {
      bus_off_ = false;
    }
    last_bus_check_ms_ = now_ms;
  }
  if (bus_off_)
  {
    faults |= RCDog::FAULT_CAN_BUS_OFF;
  }

  const bool timed_out = command.mode != Mode::OFF &&
                         now_ms - last_command_ms > kCommandTimeoutMs;
  if (timed_out || faults != RCDog::FAULT_NONE || command.locked ||
      command.mode == Mode::OFF)
  {
    std::fill(std::begin(integral_), std::end(integral_), 0.0F);
    std::fill(std::begin(ramped_rad_s_), std::end(ramped_rad_s_), 0.0F);
    int16_t zero[4]{};
    SendCurrents(zero);
    fault_bits_.store(faults, std::memory_order_release);
    return;
  }

  int16_t currents[4]{};
  for (uint8_t i = 0; i < 4; ++i)
  {
    float target = command.target_rpm[i] * command.scale[i] * kRpmToRadS;
    if (ramped_rad_s_[i] * target < 0.0F &&
        std::fabs(ramped_rad_s_[i]) > kZeroEpsilon)
    {
      target = 0.0F;
    }
    const float rate = command.mode == Mode::HOLD ? kBrake :
        (std::fabs(target) > std::fabs(ramped_rad_s_[i]) ? kAccel : kDecel);
    ramped_rad_s_[i] = Approach(ramped_rad_s_[i], target, rate * 0.002F);
    const float measured = static_cast<float>(feedback[i].speed_rpm) /
                           kGearRatio * kRpmToRadS;
    const float error = ramped_rad_s_[i] * kForwardSign[i] - measured;
    integral_[i] += kKi * error * 0.002F;
    float demand = kKp * error + integral_[i];
    const bool peak = std::fabs(demand * 10000.0F) > kContinuousRaw;
    peak_budget_ms_[i] = RCDog::Clamp(peak_budget_ms_[i] + (peak ? 2.0F : -1.0F),
                                     0.0F, static_cast<float>(kPeakMs));
    int32_t limit = peak_budget_ms_[i] >= kPeakMs ? kContinuousRaw : kPeakRaw;
    if (feedback[i].temperature_c > kDerateC)
    {
      const float ratio = static_cast<float>(feedback[i].temperature_c - kDerateC) /
                          static_cast<float>(kCutoffC - kDerateC);
      limit = static_cast<int32_t>(kPeakRaw -
          RCDog::Clamp(ratio, 0.0F, 1.0F) * (kPeakRaw - kContinuousRaw));
    }
    const float output_limit = static_cast<float>(limit) / 10000.0F;
    demand = RCDog::Clamp(demand, -output_limit, output_limit);
    integral_[i] = RCDog::Clamp(integral_[i], -output_limit, output_limit);
    currents[i] = static_cast<int16_t>(demand * 10000.0F);
  }
  SendCurrents(currents);
  fault_bits_.store(faults, std::memory_order_release);
}

void WheelMotor::SendCurrents(const int16_t currents[4])
{
  LibXR::CAN::ClassicPack pack{0x200, LibXR::CAN::Type::STANDARD, 8, {}};
  for (uint8_t i = 0; i < 4; ++i)
  {
    pack.data[i * 2] = static_cast<uint8_t>(currents[i] >> 8U);
    pack.data[i * 2 + 1] = static_cast<uint8_t>(currents[i]);
  }
  (void)can_.AddMessage(pack);
}

uint8_t WheelMotor::CalculateOnlineMask(uint32_t now_ms) const
{
  Feedback feedback[4]{};
  taskENTER_CRITICAL();
  std::memcpy(feedback, feedback_, sizeof(feedback));
  taskEXIT_CRITICAL();
  uint8_t mask = 0;
  for (uint8_t i = 0; i < 4; ++i)
  {
    if (feedback[i].last_update_ms != 0 &&
        now_ms - feedback[i].last_update_ms <= kFeedbackTimeoutMs)
    {
      mask |= static_cast<uint8_t>(1U << i);
    }
  }
  return mask;
}
