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
bool getCmdOption(int argc, char* argv[], const std::string& option, T& value)
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


int
main(int argc, char** argv)
{
  /*! Initialize variables*/


  uint16_t damp = 0;
  float speed = 5.0f;
  float step = 10.0f;
  float angle_deg = 0.0f;

  getCmdOption(argc, argv, "--damp", damp);
  getCmdOption(argc, argv, "--speed", speed);
  getCmdOption(argc, argv, "--step", step);
  getCmdOption(argc, argv, "--angle_deg", angle_deg);

  std::cout << "damp       = " << damp << std::endl;
  std::cout << "speed      = " << speed << std::endl;
  std::cout << "step       = " << step << std::endl;
  std::cout << "angle_deg  = " << angle_deg << std::endl;

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
  sample->runWaypointV2Mission(damp, speed, step, angle_deg);

  delete(sample);

  // Mission will continue when we exit here
  return 0;
}