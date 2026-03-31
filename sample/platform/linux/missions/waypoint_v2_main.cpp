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
#include "lrs_waypoint_types.hpp"
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


  std::vector<float32_t> damp_ ;
  std::vector<uint16_t> n_points_ ;
  std::vector<uint16_t> wp_type_ ;
  std::vector<uint16_t> wp_type_first_ ;
  std::vector<uint16_t> wp_type_last_  ;
  std::vector<float> speed_  ;
  std::vector<float> step_ ;
  std::vector<float> angle_deg_ ;

  getCmdOptionFlexible(argc, argv, "--damp", damp_);
  getCmdOptionFlexible(argc, argv, "--n", n_points_);
  getCmdOptionFlexible(argc, argv, "--wp", wp_type_);
  getCmdOptionFlexible(argc, argv, "--wp_first", wp_type_first_);
  getCmdOptionFlexible(argc, argv, "--wp_last", wp_type_last_);
  getCmdOptionFlexible(argc, argv, "--speed", speed_);
  getCmdOptionFlexible(argc, argv, "--step", step_);
  getCmdOptionFlexible(argc, argv, "--angle_deg", angle_deg_);

  GenParams params{};

  getCmdOptionFlexible(argc, argv, "--damps", params.damps);
  getCmdOptionFlexible(argc, argv, "--types", params.types);
  getCmdOptionFlexible(argc, argv, "--speeds", params.speeds);
  getCmdOptionFlexible(argc, argv, "--steps", params.steps);
  getCmdOptionFlexible(argc, argv, "--angles_deg", params.angles_deg);



  params.damp = damp_[0];
  params.n_points = n_points_[0] ;
  params.wp_type = wp_type_[0];
  params.wp_type_first = wp_type_first_[0];
  params.wp_type_last =wp_type_last_[0];
  params.speed = speed_[0];
  params.step = step_[0];
  params.angle_deg = angle_deg_[0];


  params.timeout = 1;

  std::cout << "damp          = " << params.damp << std::endl;
  std::cout << "n_points      = " << params.n_points << std::endl;
  std::cout << "wp_type       = " << params.wp_type << std::endl;
  std::cout << "wp_type_first = " << params.wp_type_first << std::endl;
  std::cout << "wp_type_last  = " << params.wp_type_last << std::endl;
  std::cout << "speed         = " << params.speed << std::endl;
  std::cout << "step          = " << params.step << std::endl;
  std::cout << "angle_deg     = " << params.angle_deg << std::endl;

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
  sample->runWaypointV2Mission(params);

  delete(sample);

  // Mission will continue when we exit here
  return 0;
}