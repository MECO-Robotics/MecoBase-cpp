#pragma once
#include <cstdint>
#include <functional>
#include <string_view>
#include <wpi/json.h>
namespace base::replay
{
// Desktop FRC_REPLAY_LOG enables input substitution. Active() initializes a
// requested replay before IO selection; live logging begins on explicit initialization.
// Initialize early when replay frame processing or output logging is required.
void InitializeFromEnvironment();
bool Active();
bool FrameActive();
uint64_t FrameIndex();
wpi::json ProcessInputs(std::string_view key, std::function<wpi::json()> live);
void RecordOutput(std::string_view key, const wpi::json &value);
void BeginFrame();
void EndFrame();
// EOF only: request the asynchronous writer flush and wait for its completion
// marker on disk before stopping the logger. Never called in the robot loop.
void FlushOutput();
// Drain/join the recording worker before stopping DataLogManager. Lifecycle only.
void ShutdownRecording();
// Runs existing robot callbacks once per recorded timestamp, installing HAL DS
// data and FPGA time first. Returns only after EOF; no physics is constructed.
void RunReplay(const std::function<void()> &robotInit, const std::function<void()> &loop);
} // namespace base::replay
