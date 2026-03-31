/*! @file missions/waypoint_v2_main.cpp
 *  @version 4.0.0
 *  @date Mar 07 2019
 *
 *  @brief
 *  main for Waypoint Missions V2 API usage in a Linux environment.
 *  Shows example usage of the Waypoint Missions through
 *  the Mission Manager API.
 *
 *  @Copyright (c) 2019 DJI
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
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "waypoint_v2_sample.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <cstdint>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

// Generic parser
template <typename T>
bool getCmdOptionString(int argc, char* argv[], const std::string& option, T& value)
{
  for (int i = 1; i < argc - 1; ++i)
  {
    if (std::string(argv[i]) == option)
    {
      std::istringstream iss(argv[i + 1]);
      return (iss >> value) ? true : false;
    }
  }
  return false;
}

// Unified parser: single value OR CSV
template <typename T>
bool getCmdOptionFlexible(int argc, char* argv[],
                     const std::string& option,
                     std::vector<T>& values)
{
  std::string raw;
  if (!getCmdOptionString(argc, argv, option, raw))
    return false;

  values.clear();

  std::stringstream ss(raw);
  std::string item;

  // Detect CSV by comma
  if (raw.find(',') != std::string::npos)
  {
    while (std::getline(ss, item, ','))
    {
      std::istringstream iss(item);
      T val;
      if (!(iss >> val))
        return false;

      values.push_back(val);
    }
  }
  else
  {
    // Single value → still return as vector of size 1
    std::istringstream iss(raw);
    T val;
    if (!(iss >> val))
      return false;

    values.push_back(val);
  }

  return true;
}


int
main(int argc, char** argv)
{
  /*! Initialize variables*/


  std::vector<float32_t> damp ;
  std::vector<uint16_t> n_points ;
  std::vector<uint16_t> wp_type ;
  std::vector<uint16_t> wp_type_first ;
  std::vector<uint16_t> wp_type_last  ;
  std::vector<float> speed  ;
  std::vector<float> step ;
  std::vector<float> angle_deg ;


  getCmdOptionFlexible(argc, argv, "--damp", damp);
  getCmdOptionFlexible(argc, argv, "--n", n_points);
  getCmdOptionFlexible(argc, argv, "--wp", wp_type);
  getCmdOptionFlexible(argc, argv, "--wp_first", wp_type_first);
  getCmdOptionFlexible(argc, argv, "--wp_last", wp_type_last);
  getCmdOptionFlexible(argc, argv, "--speed", speed);
  getCmdOptionFlexible(argc, argv, "--step", step);
  getCmdOptionFlexible(argc, argv, "--angle_deg", angle_deg);

  std::cout << "damp          = " << damp[0] << std::endl;
  std::cout << "n_points      = " << n_points[0] << std::endl;
  std::cout << "wp_type       = " << wp_type[0] << std::endl;
  std::cout << "wp_type_first = " << wp_type_first[0] << std::endl;
  std::cout << "wp_type_last  = " << wp_type_last[0] << std::endl;
  std::cout << "speed         = " << speed[0] << std::endl;
  std::cout << "step          = " << step[0] << std::endl;
  std::cout << "angle_deg     = " << angle_deg[0] << std::endl;

  int functionTimeout = 1;
  /*! Setup OSDK.*/
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  int responseTimeout = 1;

  /*! Obtain Control Authority*/
  vehicle->control->obtainCtrlAuthority(functionTimeout);

  /*! Initialize a new WaypointV2 mission sample*/
  auto *sample = new WaypointV2MissionSample(vehicle);

  /*! run a new WaypointV2 mission sample*/
  sample->runWaypointV2Mission(
    damp, speed, step, angle_deg, n_points, wp_type, wp_type_first, wp_type_last);

  delete(sample);

  // Mission will continue when we exit here
  return 0;
}