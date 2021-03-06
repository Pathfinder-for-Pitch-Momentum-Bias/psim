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

#include <psim/truth/transform_velocity.hpp>

#include <gnc/utilities.hpp>
#include <lin/core.hpp>

namespace psim {

TransformVelocityEcef::TransformVelocityEcef(RandomsGenerator &randoms,
    Configuration const &config, std::string const &satellite,
    std::string const &vector)
  : Super(randoms, config, satellite, vector) { }

Vector3 TransformVelocityEcef::vector_ecef() const {
  return vector->get();
}

Vector3 TransformVelocityEcef::vector_eci() const {
  auto const &v_ecef = vector->get();
  auto const &q_eci_ecef = truth_earth_q_eci_ecef->get();
  auto const &r_ecef = truth_satellite_orbit_r_ecef->get();
  auto const &w_earth = truth_earth_w->get();

  Vector3 v_eci;
  gnc::utl::rotate_frame(q_eci_ecef, (v_ecef + lin::cross(w_earth, r_ecef)).eval(), v_eci);
  return v_eci;
}

TransformVelocityEci::TransformVelocityEci(RandomsGenerator &randoms,
    Configuration const &config, std::string const &satellite,
    std::string const &vector)
  : Super(randoms, config, satellite, vector) { }

Vector3 TransformVelocityEci::vector_ecef() const {
  auto const &v_eci = vector->get();
  auto const &q_ecef_eci = truth_earth_q_ecef_eci->get();
  auto const &r_ecef = truth_satellite_orbit_r_ecef->get();
  auto const &w_earth = truth_earth_w->get();

  Vector3 v;
  gnc::utl::rotate_frame(q_ecef_eci, v_eci, v);
  return v - lin::cross(w_earth, r_ecef);
}

Vector3 TransformVelocityEci::vector_eci() const {
  return vector->get();
}
}  // namespace psim
