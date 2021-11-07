//
// MIT License
//
// Copyright (c) 2020 Pathfinder for Autonomous Navigation (PAN)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

/** @file psim/fc/orbit_controller.cpp
 *  @author Govind Chari
 */

#include <psim/fc/attitude_controller.hpp>
#include <gnc/attitude_controller.hpp>
#include <psim/fc/attitude_estimator.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>
#include <lin/math.hpp>



namespace psim {

void AttitudeController::add_fields(State &state) {
  this->Super::add_fields(state);
}

void AttitudeController::step() {
  this->Super::step();

  float Kp = 0.0f;
  float Kd = 0.000000026037f;

  auto const &attitude_valid = fc_satellite_attitude_is_valid->get();
  auto const &q_body_eci = fc_satellite_attitude_q_body_eci->get();
  auto const &w = fc_satellite_attitude_w->get();
  auto &wheels_t = truth_satellite_wheels_t->get();

  lin::Vector3f q_vec = {q_body_eci(1),q_body_eci(2),q_body_eci(3)};
  lin::Vector3f torque;

  if (attitude_valid)  {
    torque =  -(Kp * q_vec + Kd * w);
  } else {
    torque = {0.0f, 0.0f, 0.0f};
  }
  
  wheels_t = torque;

}
    
} // namespace psim