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

#include <psim/truth/transform_position.hpp>

#include <gnc/utilities.hpp>

namespace psim {

Vector3 TransformPositionEcef::vector_ecef() const {
  return vector->get();
}

Vector3 TransformPositionEcef::vector_eci() const {
  auto const &r_ecef = vector->get();
  auto const &q_eci_ecef = truth_earth_q_eci_ecef->get();

  Vector3 r_eci;
  gnc::utl::rotate_frame(q_eci_ecef, r_ecef, r_eci);
  return r_eci;
}

Vector3 TransformPositionEci::vector_ecef() const {
  auto const &r_eci = vector->get();
  auto const &q_ecef_eci = truth_earth_q_ecef_eci->get();

  Vector3 r_ecef;
  gnc::utl::rotate_frame(q_ecef_eci, r_eci, r_ecef);
  return r_ecef;
}

Vector3 TransformPositionEci::vector_eci() const {
  return vector->get();
}
}  // namespace psim
