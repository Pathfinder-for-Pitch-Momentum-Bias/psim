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

/** @file psim/sensors/sun_sensors.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_SENSORS_SUN_SENSORS_HPP_
#define PSIM_SENSORS_SUN_SENSORS_HPP_

#include <psim/sensors/sun_sensors.yml.hpp>

namespace psim {

class SunSensors : public SunSensorsInterface<SunSensors> {
 private:
  typedef SunSensorsInterface<SunSensors> Super;

 public:
  using Super::SunSensorsInterface;

  SunSensors() = delete;
  virtual ~SunSensors() = default;

  Boolean sensors_satellite_sun_sensors_valid() const;
  Vector3 sensors_satellite_sun_sensors_s() const;
  Vector3 sensors_satellite_sun_sensors_s_error() const;
};
} // namespace psim

#endif
