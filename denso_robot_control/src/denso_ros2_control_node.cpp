/**
 * Software License Agreement (MIT License)
 *
 * @copyright Copyright (c) 2015 DENSO WAVE INCORPORATED
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <chrono>
#include <cerrno>
#include <cstring>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <pthread.h>
#include <sched.h>
#include <sstream>
#include <sys/resource.h>
#include <thread>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr std::chrono::nanoseconds kDefaultControlPeriod = std::chrono::milliseconds(8);
constexpr double kCycleOverrunThresholdMs = 20.0;
constexpr int kRealtimePriority = 80;
constexpr int kFallbackNice = -20;

rclcpp::Duration GetFixedPeriod(const controller_manager::ControllerManager & cm)
{
  const auto update_rate = cm.get_update_rate();
  if (update_rate > 0) {
    return rclcpp::Duration::from_nanoseconds(1'000'000'000LL / update_rate);
  }

  return rclcpp::Duration::from_nanoseconds(kDefaultControlPeriod.count());
}

void RaiseControlThreadPriority(const rclcpp::Logger & logger)
{
  sched_param param{};
  param.sched_priority = kRealtimePriority;
  const int sched_error = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  if (sched_error == 0) {
    RCLCPP_INFO(logger, "SCHED_FIFO(prio=%d) applied", kRealtimePriority);
    return;
  }

  errno = 0;
  const int nice_result = setpriority(PRIO_PROCESS, 0, kFallbackNice);
  const int nice_error = errno;
  if (nice_result == 0) {
    RCLCPP_INFO(
      logger, "nice(%d) applied (SCHED_FIFO failed: %d/%s)", kFallbackNice, sched_error,
      std::strerror(sched_error));
    return;
  }

  RCLCPP_WARN(
    logger, "Failed to raise priority (SCHED_FIFO: %d/%s, nice: %d/%s)", sched_error,
    std::strerror(sched_error), nice_error, std::strerror(nice_error));
}

timespec GetThreadCpuTime()
{
  timespec ts{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
  return ts;
}

double ToMilliseconds(const timespec & start, const timespec & end)
{
  constexpr double kNanosecondsPerMillisecond = 1.0e6;
  const auto seconds = static_cast<int64_t>(end.tv_sec) - static_cast<int64_t>(start.tv_sec);
  const auto nanoseconds = static_cast<int64_t>(end.tv_nsec) - static_cast<int64_t>(start.tv_nsec);
  return static_cast<double>(seconds * 1'000'000'000LL + nanoseconds) /
         kNanosecondsPerMillisecond;
}

double ToMilliseconds(const std::chrono::steady_clock::time_point & start,
  const std::chrono::steady_clock::time_point & end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

void PrintCycleOverrun(double read_wall_ms, double update_wall_ms, double write_wall_ms,
  double read_cpu_ms, double update_cpu_ms, double write_cpu_ms)
{
  const double total_wall_ms = read_wall_ms + update_wall_ms + write_wall_ms;
  if (total_wall_ms <= kCycleOverrunThresholdMs) {
    return;
  }

  const double total_cpu_ms = read_cpu_ms + update_cpu_ms + write_cpu_ms;
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << "[CYCLE_OVERRUN] wall=" << total_wall_ms << "ms (R=" << read_wall_ms
         << " U=" << update_wall_ms << " W=" << write_wall_ms << ") cpu=" << total_cpu_ms
         << "ms (R=" << read_cpu_ms << " U=" << update_cpu_ms << " W=" << write_cpu_ms
         << ")";
  std::cout << stream.str() << std::endl;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto cm = std::make_shared<controller_manager::ControllerManager>(
    executor, "controller_manager");

  const auto fixed_period = GetFixedPeriod(*cm);
  // Anchor the virtual clock to real system time (once at startup) so that
  // /joint_states header stamps share the same clock/epoch as move_group,
  // while still advancing in strict fixed_period (8 ms) steps below.
  auto virtual_time = cm->now();

  executor->add_node(cm);
  std::thread executor_thread([executor]() { executor->spin(); });

  RCLCPP_INFO(
    cm->get_logger(), "Using fixed virtual control period of %.3f ms",
    static_cast<double>(fixed_period.nanoseconds()) / 1.0e6);
  RaiseControlThreadPriority(cm->get_logger());

  // Safety principle: never add sleep(), WallRate, or any other real-time pacing here.
  // The DENSO controller's SYNC write() blocking is the only valid pacing source; introducing
  // a second software clock risks buffer depletion and a robot stop.
  //
  // Startup buffer fill is intentionally left to the existing natural fast loop when write()
  // is not yet blocking. Dedicated underrun mitigation is a separate phase-2 task and is
  // intentionally out of scope for this node.
  while (rclcpp::ok()) {
    const auto read_wall_start = std::chrono::steady_clock::now();
    const auto read_cpu_start = GetThreadCpuTime();
    cm->read(virtual_time, fixed_period);
    const auto read_cpu_end = GetThreadCpuTime();
    const auto read_wall_end = std::chrono::steady_clock::now();

    const auto update_wall_start = std::chrono::steady_clock::now();
    const auto update_cpu_start = GetThreadCpuTime();
    cm->update(virtual_time, fixed_period);
    const auto update_cpu_end = GetThreadCpuTime();
    const auto update_wall_end = std::chrono::steady_clock::now();

    const auto write_wall_start = std::chrono::steady_clock::now();
    const auto write_cpu_start = GetThreadCpuTime();
    cm->write(virtual_time, fixed_period);
    const auto write_cpu_end = GetThreadCpuTime();
    const auto write_wall_end = std::chrono::steady_clock::now();

    PrintCycleOverrun(
      ToMilliseconds(read_wall_start, read_wall_end),
      ToMilliseconds(update_wall_start, update_wall_end),
      ToMilliseconds(write_wall_start, write_wall_end),
      ToMilliseconds(read_cpu_start, read_cpu_end),
      ToMilliseconds(update_cpu_start, update_cpu_end),
      ToMilliseconds(write_cpu_start, write_cpu_end));

    virtual_time = virtual_time + fixed_period;
  }

  executor->cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
