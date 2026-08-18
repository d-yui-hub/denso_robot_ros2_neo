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
#include <memory>
#include <thread>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr std::chrono::nanoseconds kDefaultControlPeriod = std::chrono::milliseconds(8);

rclcpp::Duration GetFixedPeriod(const controller_manager::ControllerManager & cm)
{
  rclcpp::Parameter update_rate_parameter;
  if (cm.get_parameter("update_rate", update_rate_parameter)) {
    const auto update_rate = update_rate_parameter.as_int();
    if (update_rate > 0) {
      return rclcpp::Duration::from_nanoseconds(1000000000LL / update_rate);
    }
  }

  return rclcpp::Duration::from_nanoseconds(kDefaultControlPeriod.count());
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto cm = std::make_shared<controller_manager::ControllerManager>(
    executor, "controller_manager");

  const auto fixed_period = GetFixedPeriod(*cm);
  auto virtual_time = rclcpp::Time(0, 0, RCL_ROS_TIME);

  executor->add_node(cm);
  std::thread executor_thread([executor]() { executor->spin(); });

  RCLCPP_INFO(
    cm->get_logger(), "Using fixed virtual control period of %.3f ms",
    static_cast<double>(fixed_period.nanoseconds()) / 1.0e6);

  // Safety principle: never add sleep(), WallRate, or any other real-time pacing here.
  // The DENSO controller's SYNC write() blocking is the only valid pacing source; introducing
  // a second software clock risks buffer depletion and a robot stop.
  //
  // Startup buffer fill is intentionally left to the existing natural fast loop when write()
  // is not yet blocking. Dedicated underrun mitigation is a separate phase-2 task and is
  // intentionally out of scope for this node.
  while (rclcpp::ok()) {
    cm->read(virtual_time, fixed_period);
    cm->update(virtual_time, fixed_period);
    cm->write(virtual_time, fixed_period);
    virtual_time = virtual_time + fixed_period;
  }

  executor->cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
