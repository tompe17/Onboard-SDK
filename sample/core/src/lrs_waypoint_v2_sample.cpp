/*! @file waypoint_v2_sample.cpp
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

#include "lrs_waypoint_v2_sample.hpp"
#include "dji_waypoint_v2_action.hpp"
#include "memory"
#include <ctime>
#include <stdlib.h>
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

// 10HZ push ;1HZ print
E_OsdkStat
updateMissionState(T_CmdHandle*     cmdHandle,
                   const T_CmdInfo* cmdInfo,
                   const uint8_t*   cmdData,
                   void*            userData)
{

  if (cmdInfo)
  {
    if (userData)
    {
      auto* wp2Ptr              = (WaypointV2MissionOperator*)userData;
      auto* missionStatePushAck = (DJI::OSDK::MissionStatePushAck*)cmdData;

      wp2Ptr->setCurrentState(wp2Ptr->getCurrentState());
      wp2Ptr->setCurrentState(
        (DJI::OSDK::DJIWaypointV2MissionState)missionStatePushAck->data.state);
      static uint32_t curMs = 0;
      static uint32_t preMs = 0;
      OsdkOsal_GetTimeMs(&curMs);
      if (curMs - preMs >= 1000)
      {
        preMs = curMs;
        DSTATUS("missionStatePushAck ->commonDataVersion:%d commonDataLen:%d",
                missionStatePushAck->commonDataVersion,
                missionStatePushAck->commonDataLen);
        DSTATUS("->missionStatePushAck->data: state:0x%x curWaypointIndex:%d "
                "velocity:%d",
                missionStatePushAck->data.state,
                missionStatePushAck->data.curWaypointIndex,
                missionStatePushAck->data.velocity);

        //        DSTATUS("missionStatePushAck->commonDataVersion:%d",
        //                missionStatePushAck->commonDataVersion);
        //        DSTATUS("missionStatePushAck->commonDataLen:%d",
        //                missionStatePushAck->commonDataLen);
        //        DSTATUS("missionStatePushAck->data.state:0x%x",
        //                missionStatePushAck->data.state);
        //        DSTATUS("missionStatePushAck->data.curWaypointIndex:%d",
        //                missionStatePushAck->data.curWaypointIndex);
        //        DSTATUS("missionStatePushAck->data.velocity:%d",
        //                missionStatePushAck->data.velocity);
      }
    }
    else
    {
      DERROR("cmdInfo is a null value");
    }
    return OSDK_STAT_OK;
  }
  return OSDK_STAT_ERR_ALLOC;
}

/*! only push 0x00,0x10,0x11 event*/
E_OsdkStat
updateMissionEvent(T_CmdHandle*     cmdHandle,
                   const T_CmdInfo* cmdInfo,
                   const uint8_t*   cmdData,
                   void*            userData)
{

  if (cmdInfo)
  {
    if (userData)
    {
      auto* MissionEventPushAck = (DJI::OSDK::MissionEventPushAck*)cmdData;

      DSTATUS("MissionEventPushAck->event ID :0x%x",
              MissionEventPushAck->event);

      if (MissionEventPushAck->event == 0x01)
        DSTATUS("interruptReason:0x%x",
                MissionEventPushAck->data.interruptReason);
      if (MissionEventPushAck->event == 0x02)
        DSTATUS("recoverProcess:0x%x",
                MissionEventPushAck->data.recoverProcess);
      if (MissionEventPushAck->event == 0x03)
        DSTATUS("finishReason:0x%x", MissionEventPushAck->data.finishReason);

      if (MissionEventPushAck->event == 0x10)
        DSTATUS("current waypointIndex:%d",
                MissionEventPushAck->data.waypointIndex);

      if (MissionEventPushAck->event == 0x11)
      {
        DSTATUS(
          "currentMissionExecNum:%d",
          MissionEventPushAck->data.MissionExecEvent.currentMissionExecNum);
      }

      return OSDK_STAT_OK;
    }
  }
  return OSDK_STAT_SYS_ERR;
}

WaypointV2MissionSample::WaypointV2MissionSample(Vehicle* vehicle)
  : vehiclePtr(vehicle)
{
  vehiclePtr->waypointV2Mission->RegisterMissionEventCallback(
    vehicle->waypointV2Mission, updateMissionEvent);
  vehiclePtr->waypointV2Mission->RegisterMissionStateCallback(
    vehicle->waypointV2Mission, updateMissionState);
}

WaypointV2MissionSample::~WaypointV2MissionSample() = default;

bool
WaypointV2MissionSample::setUpSubscription(int timeout)
{
  // Telemetry: Verify the subscription
  ACK::ErrorCode subscribeStatus;

  subscribeStatus = vehiclePtr->subscribe->verify(timeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    return false;
  }
  // TOPIC_ALTITUDE_OF_HOMEPOINT, TOPIC_GPS_POSITION
  //  Telemetry: Subscribe to flight status and mode at freq 10 Hz
  int       freq            = 1;
  TopicName topicList10Hz[] = { TOPIC_GPS_FUSED };
  int       numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  bool      enableTimestamp = false;

  bool pkgStatus = vehiclePtr->subscribe->initPackageFromTopicList(
    DEFAULT_PACKAGE_INDEX, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  usleep(5000);
  // Start listening to the telemetry data
  subscribeStatus =
    vehiclePtr->subscribe->startPackage(DEFAULT_PACKAGE_INDEX, timeout);
  usleep(5000);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup
    ACK::ErrorCode ack =
      vehiclePtr->subscribe->removePackage(DEFAULT_PACKAGE_INDEX, timeout);
    if (ACK::getError(ack))
    {
      DSTATUS("Error unsubscribing; please restart the drone/FC to get "
              "back to a clean state.");
    }
    return false;
  }
  return true;
}

bool
WaypointV2MissionSample::teardownSubscription(const int pkgIndex, int timeout)
{
  ACK::ErrorCode ack = vehiclePtr->subscribe->removePackage(pkgIndex, timeout);
  if (ACK::getError(ack))
  {
    DSTATUS("Error unsubscribing; please restart the drone/FC to get back "
            "to a clean state.");
    return false;
  }
  return true;
}

ErrorCode::ErrorCodeType
WaypointV2MissionSample::runWaypointV2Mission(float32_t damping,
                                              float     speed,
                                              float     step,
                                              float     angle_deg,
                                              uint16_t  n_points,
                                              uint16_t  wp_type,
                                              uint16_t  wp_type_first,
                                              uint16_t  wp_type_last)
{
  if (!vehiclePtr->isM300())
  {
    DSTATUS("This sample only supports M300!");
    return false;
  }

  int                      timeout      = 1;
  GetRemainRamAck          actionMemory = { 0 };
  ErrorCode::ErrorCodeType ret;

  if (!setUpSubscription(timeout))
  {
    DERROR("Failed to set up subscription!");
    return -1;
  }
  else
  {
    DSTATUS("Set up subscription successfully!");
  }
  /*! wait for subscription data come*/
  sleep(timeout);
  //  sleep(timeout);
  //  sleep(5);
  /*! init mission */
  ret = initMissionSetting(timeout,
                           damping,
                           speed,
                           step,
                           angle_deg,
                           n_points,
                           wp_type,
                           wp_type_first,
                           wp_type_last);
  if (ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(timeout);

  /*! upload mission */
  /*! upload mission's timeout need to be longer than 2s*/
  int uploadMissionTimeOut = 3;
  ret                      = uploadWaypointMission(uploadMissionTimeOut);
  if (ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(timeout);

  /*! download mission */
  std::vector<WaypointV2> mission;
  ret = downloadWaypointMission(mission, timeout);
  if (ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(timeout);

  /*! upload  actions */
  /*! check action memory */
  ret = getActionRemainMemory(actionMemory, timeout);
  if (actionMemory.remainMemory <= 0)
  {
    DSTATUS("action memory is not enough.Can not upload more action!");
    return ErrorCode::SysCommonErr::UndefinedError;
  }

  ret = uploadWapointActions(timeout);
  if (ret != ErrorCode::SysCommonErr::Success)
    return ret;

  ret = getActionRemainMemory(actionMemory, timeout);
  sleep(timeout);

  /*! start mission */
  ret = startWaypointMission(timeout);
  if (ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(20);

  /*! set global cruise speed */
  // setGlobalCruiseSpeed(8.0, timeout);
  // sleep(timeout);

  /*! get global cruise speed */
  getGlobalCruiseSpeed(8.0);

  sleep(timeout);

  /*! pause the mission*/
#if 0  
  ret = pauseWaypointMission(timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
  sleep(5);
#endif

  /*! resume the mission*/
#if 0  
  ret = resumeWaypointMission(timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
    return ret;
#endif
  sleep(20);
  /*! Set up telemetry subscription*/
  if (!teardownSubscription(DEFAULT_PACKAGE_INDEX, timeout))
  {
    std::cout << "Failed to tear down Subscription!" << std::endl;
    return ErrorCode::SysCommonErr::UndefinedError;
  }

  return ErrorCode::SysCommonErr::Success;
}

// Earth radius in meters
// constexpr double EARTH_RADIUS = 6378137.0;

// Convert degrees to radians
double
deg2rad(double deg)
{
  return deg * M_PI / 180.0;
}




WaypointV2MissionSample::Vec3 WaypointV2MissionSample::toLocalXYZ(const WaypointV2& ref, const WaypointV2& p)
{
  double dLat = p.latitude  - ref.latitude;
  double dLon = p.longitude - ref.longitude;
  double dAlt = p.relativeHeight - ref.relativeHeight; // or altitude field

  double x = dLon * EARTH_RADIUS * cos(ref.latitude);
  double y = dLat * EARTH_RADIUS;
  double z = dAlt;

  return {x, y, z};
}
double WaypointV2MissionSample::computeAngleDeg3D(const WaypointV2& A,
                  const WaypointV2& B,
                  const WaypointV2& C)
{
  Vec3 BA = toLocalXYZ(B, A);
  Vec3 BC = toLocalXYZ(B, C);

  // Dot product
  double dot = BA.x * BC.x + BA.y * BC.y + BA.z * BC.z;

  // Magnitudes
  double mag1 = sqrt(BA.x*BA.x + BA.y*BA.y + BA.z*BA.z);
  double mag2 = sqrt(BC.x*BC.x + BC.y*BC.y + BC.z*BC.z);

  if (mag1 == 0 || mag2 == 0)
    return 0.0;

  double cosAngle = dot / (mag1 * mag2);

  // Clamp for safety
  cosAngle = std::max(-1.0, std::min(1.0, cosAngle));

  double angleRad = acos(cosAngle);
  return angleRad * 180.0 / M_PI;
}

// Haversine distance (meters)
double
WaypointV2MissionSample::calculateDistance(const WaypointV2& wp1,
                                           const WaypointV2& wp2)
{
  double lat1 = (wp1.latitude);
  double lon1 = (wp1.longitude);
  double lat2 = (wp2.latitude);
  double lon2 = (wp2.longitude);

  //  printf(
  //    "lat1: %f lon1: %f   ----  lat2: %f, lon2: %f", lat1, lon1, lat2, lon2);

  double dLat = lat2 - lat1;
  double dLon = lon2 - lon1;

  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);

  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return EARTH_RADIUS * c;
}

void
WaypointV2MissionSample::printWaypointDistances(
  const std::vector<WaypointV2>& waypointList)
{
  for (size_t i = 1; i < waypointList.size(); ++i)
  {
    double dist = calculateDistance3D(waypointList[i - 1], waypointList[i]);

    printf("Distance WP[%zu] -> WP[%zu]: %.2f meters\n", i - 1, i, dist);
  }
}

void WaypointV2MissionSample::printWaypointAngles3D(
  const std::vector<WaypointV2>& waypointList)
{
  for (size_t i = 1; i < waypointList.size() - 1; ++i)
  {
    double angle = computeAngleDeg3D(
      waypointList[i - 1],
      waypointList[i],
      waypointList[i + 1]);

    printf("Angle at WP[%zu]: %.2f deg\n", i, angle);
  }
}

double
WaypointV2MissionSample::calculateDistance3D(const WaypointV2& wp1,
                                             const WaypointV2& wp2)
{
  double horizontal = calculateDistance(wp1, wp2);
  double vertical   = wp2.relativeHeight - wp1.relativeHeight;

  return sqrt(horizontal * horizontal + vertical * vertical);
}

ErrorCode::ErrorCodeType
WaypointV2MissionSample::initMissionSetting(int       timeout,
                                            float32_t damping,
                                            float     speed,
                                            float     step,
                                            float     angle_deg,
                                            uint16_t  n_points,
                                            uint16_t  wp_type,
                                            uint16_t  wp_type_first,
                                            uint16_t  wp_type_last)
{

  // uint16_t polygonNum = 6;
  // float32_t radius = 6;

  // uint16_t actionNum = 5;
  srand(int(time(0)));

  /*! Generate waypoints*/

  /*! Generate actions*/
  /// this->actions = generateWaypointActions(actionNum);

  /*! Init waypoint settings*/
  WayPointV2InitSettings missionInitSettings;
  missionInitSettings.missionID      = rand();
  missionInitSettings.repeatTimes    = 1;
  missionInitSettings.finishedAction = DJIWaypointV2MissionFinishedNoAction;
  //  missionInitSettings.finishedAction =
  //  DJIWaypointV2MissionFinishedGoToFirstWaypoint;
  missionInitSettings.maxFlightSpeed            = 12.0;
  missionInitSettings.autoFlightSpeed           = 7.0;
  missionInitSettings.exitMissionOnRCSignalLost = 1;
  missionInitSettings.gotoFirstWaypointMode =
    DJIWaypointV2MissionGotoFirstWaypointModePointToPoint;
  //  missionInitSettings.gotoFirstWaypointMode =
  //    DJIWaypointV2MissionGotoFirstWaypointModeSafely;

  // missionInitSettings.mission =  generatePolygonWaypoints(radius,
  // polygonNum); missionInitSettings.mission =  generateLineWaypoints(10.0, 8);
  // missionInitSettings.mission =  generateStairWaypoints(20.0, 6);
  missionInitSettings.mission      = generateAngleWaypoints(step,
                                                       angle_deg,
                                                       n_points,
                                                       damping,
                                                       speed,
                                                       wp_type,
                                                       wp_type_first,
                                                       wp_type_last);
  missionInitSettings.missTotalLen = missionInitSettings.mission.size();

  printWaypointDistances(missionInitSettings.mission);
  printWaypointAngles3D(missionInitSettings.mission);
  int i = 0;
  for (auto& wp : missionInitSettings.mission)
  {
    printWpInfo(wp, std::to_string(i++));
  }

  //  printWaypointDistances(missionInitSettings.mission);

  ErrorCode::ErrorCodeType ret =
    vehiclePtr->waypointV2Mission->init(&missionInitSettings, timeout);
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Init mission setting ErrorCode:0x%lX", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Init mission setting successfully!");
  }
  return ret;
}

ErrorCode::ErrorCodeType
WaypointV2MissionSample::uploadWaypointMission(int timeout)
{

  //  ErrorCode::ErrorCodeType ret =
  //  vehiclePtr->waypointV2Mission->uploadMission(this->mission,timeout);
  ErrorCode::ErrorCodeType ret =
    vehiclePtr->waypointV2Mission->uploadMission(timeout);
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Upload waypoint v2 mission ErrorCode:0x%lX", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Upload waypoint v2 mission successfully!");
  }
  return ret;
}

ErrorCode::ErrorCodeType
WaypointV2MissionSample::downloadWaypointMission(
  std::vector<WaypointV2>& mission,
  int                      timeout)
{
  ErrorCode::ErrorCodeType ret =
    vehiclePtr->waypointV2Mission->downloadMission(mission, timeout);
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Download waypoint v2 mission ErrorCode:0x%lX", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Download waypoint v2 mission successfully!");
  }
  return ret;
}

ErrorCode::ErrorCodeType
WaypointV2MissionSample::uploadWapointActions(int timeout)
{
  ErrorCode::ErrorCodeType ret =
    vehiclePtr->waypointV2Mission->uploadAction(actions, timeout);
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Upload waypoint v2 actions ErrorCode:0x%lX", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Upload waypoint v2 actions successfully!");
  }
  return ret;
}

ErrorCode::ErrorCodeType
WaypointV2MissionSample::startWaypointMission(int timeout)
{
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->start(timeout);
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Start waypoint v2 mission ErrorCode:0x%lX", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Start waypoint v2 mission successfully!");
  }
  return ret;
}

ErrorCode::ErrorCodeType
WaypointV2MissionSample::stopWaypointMission(int timeout)
{
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->stop(timeout);
  return ret;
}

ErrorCode::ErrorCodeType
WaypointV2MissionSample::pauseWaypointMission(int timeout)
{
  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->pause(timeout);
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Pause waypoint v2 mission ErrorCode:0x%lX", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Pause waypoint v2 mission successfully!");
  }
  sleep(5);
  return ret;
}

ErrorCode::ErrorCodeType
WaypointV2MissionSample::resumeWaypointMission(int timeout)
{

  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->resume(timeout);
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Resume Waypoint v2 mission ErrorCode:0x%lX", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("Resume Waypoint v2 mission successfully!");
  }
  return ret;
}

void
WaypointV2MissionSample::getGlobalCruiseSpeed(int timeout)
{
  GlobalCruiseSpeed        cruiseSpeed = 0;
  ErrorCode::ErrorCodeType ret =
    vehiclePtr->waypointV2Mission->getGlobalCruiseSpeed(cruiseSpeed, timeout);
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Get glogal cruise speed failed ErrorCode:0x%lX", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return;
  }
  DSTATUS("Current cruise speed is: %f m/s", cruiseSpeed);
}

void
WaypointV2MissionSample::setGlobalCruiseSpeed(
  const GlobalCruiseSpeed& cruiseSpeed,
  int                      timeout)
{
  ErrorCode::ErrorCodeType ret =
    vehiclePtr->waypointV2Mission->setGlobalCruiseSpeed(cruiseSpeed, timeout);
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Set glogal cruise speed %f m/s failed ErrorCode:0x%lX",
           cruiseSpeed,
           ret);
    ErrorCode::printErrorCodeMsg(ret);
    return;
  }
  DSTATUS("Current cruise speed is: %f m/s", cruiseSpeed);
}

WaypointV2
WaypointV2MissionSample::xyzToWaypointV2(double            x,
                                         double            y,
                                         double            z,
                                         const WaypointV2& startWp,
                                         WaypointV2&       wp)
{
  //  WaypointV2 wp;

  wp.latitude  = x / EARTH_RADIUS + startWp.latitude;
  wp.longitude = y / (EARTH_RADIUS * cos(startWp.latitude)) + startWp.longitude;
  wp.relativeHeight = startWp.relativeHeight;
  return wp;
}

std::vector<WaypointV2>
WaypointV2MissionSample::generatePolygonWaypoints(float32_t radius,
                                                  uint16_t  polygonNum)
{
  // Let's create a vector to store our waypoints in.
  std::vector<WaypointV2> waypointList;
  WaypointV2              startPoint;
  WaypointV2              waypointV2;

  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition =
    vehiclePtr->subscribe->getValue<TOPIC_GPS_FUSED>();
  startPoint.latitude       = subscribeGPosition.latitude;
  startPoint.longitude      = subscribeGPosition.longitude;
  startPoint.relativeHeight = 15;
  setWaypointV2Defaults(startPoint);
  waypointList.push_back(startPoint);

  // Iterative algorithm
  for (int i = 0; i < polygonNum; i++)
  {
    float32_t angler_rad = i * 2 * M_PI / polygonNum;
    setWaypointV2Defaults(waypointV2);
    float32_t x = radius * cos(angler_rad);
    float32_t y = radius * sin(angler_rad);

    xyzToWaypointV2(x, y, 0, startPoint, waypointV2);
    //    waypointV2.latitude = x / EARTH_RADIUS + startPoint.latitude;
    //    waypointV2.longitude =
    //      y / (EARTH_RADIUS * cos(startPoint.latitude)) +
    //      startPoint.longitude;
    //    waypointV2.relativeHeight = startPoint.relativeHeight;
    waypointList.push_back(waypointV2);
  }
  waypointList.push_back(startPoint);
  return waypointList;
}

std::vector<WaypointV2>
WaypointV2MissionSample::generateLineWaypoints(float32_t step,
                                               uint16_t  n_points)
{
  // Let's create a vector to store our waypoints in.
  std::vector<WaypointV2> waypointList;
  WaypointV2              startPoint;
  WaypointV2              waypointV2;

  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition =
    vehiclePtr->subscribe->getValue<TOPIC_GPS_FUSED>();
  startPoint.latitude       = subscribeGPosition.latitude;
  startPoint.longitude      = subscribeGPosition.longitude;
  startPoint.relativeHeight = 15;
  setWaypointV2Defaults(startPoint);
  startPoint.waypointType    = DJIWaypointV2FlightPathModeGoToPointAlongACurve;
  startPoint.dampingDistance = 0.0;
  /// waypointList.push_back(startPoint);

  // Iterative algorithm
  for (int i = 0; i < n_points; i++)
  {
    setWaypointV2Defaults(waypointV2);
    float32_t X                = step + (i + 3) * step;
    float32_t Y                = 0.0;
    waypointV2.dampingDistance = 0.0;
    waypointV2.waypointType = DJIWaypointV2FlightPathModeGoToPointAlongACurve;
    if (i == (n_points - 1))
    {
      waypointV2.waypointType =
        DJIWaypointV2FlightPathModeGoToPointAlongACurveAndStop;
      waypointV2.dampingDistance = 6.0;
    }
    waypointV2.latitude = X / EARTH_RADIUS + startPoint.latitude;
    waypointV2.longitude =
      Y / (EARTH_RADIUS * cos(startPoint.latitude)) + startPoint.longitude;
    waypointV2.relativeHeight = startPoint.relativeHeight;
    waypointList.push_back(waypointV2);
  }
  /// waypointList.push_back(startPoint);
  return waypointList;
}

std::vector<WaypointV2>
WaypointV2MissionSample::generateStairWaypoints(float32_t step,
                                                uint16_t  n_points)
{
  // Let's create a vector to store our waypoints in.
  std::vector<WaypointV2> waypointList;
  WaypointV2              startPoint;
  WaypointV2              waypointV2;

  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition =
    vehiclePtr->subscribe->getValue<TOPIC_GPS_FUSED>();
  startPoint.latitude       = subscribeGPosition.latitude;
  startPoint.longitude      = subscribeGPosition.longitude;
  startPoint.relativeHeight = 15;
  setWaypointV2Defaults(startPoint);
  startPoint.waypointType    = DJIWaypointV2FlightPathModeGoToPointAlongACurve;
  startPoint.dampingDistance = 1.0;
  /// waypointList.push_back(startPoint);

  // Iterative algorithm
  float32_t X = 20.0;
  float32_t Y = 0.0;
  for (int i = 0; i < n_points; i++)
  {
    setWaypointV2Defaults(waypointV2);
    if (i % 2)
    {
      X += step;
    }
    else
    {
      Y += step;
    }
    waypointV2.dampingDistance = 0.0;
    /// waypointV2.waypointType =
    /// DJIWaypointV2FlightPathModeGoToPointAlongACurve;
    waypointV2.waypointType = DJIWaypointV2FlightPathModeCoordinateTurn;
    if (i == (n_points - 1))
    {
      waypointV2.waypointType =
        DJIWaypointV2FlightPathModeGoToPointAlongACurveAndStop;
      waypointV2.dampingDistance = 0.0;
    }
    waypointV2.latitude = X / EARTH_RADIUS + startPoint.latitude;
    waypointV2.longitude =
      Y / (EARTH_RADIUS * cos(startPoint.latitude)) + startPoint.longitude;
    waypointV2.relativeHeight = startPoint.relativeHeight;
    waypointList.push_back(waypointV2);
  }
  /// waypointList.push_back(startPoint);
  return waypointList;
}

double
WaypointV2MissionSample::rad2deg(const double& rad)
{
  return rad / M_PI * 180.0;
}

std::string
WaypointV2MissionSample::wpTypeToString(const DJIWaypointV2FlightPathMode& mode)
{
  switch (mode)
  {
    case DJIWaypointV2FlightPathModeGoToPointAlongACurve:
      return "GoToPointAlongACurve";
    case DJIWaypointV2FlightPathModeGoToPointAlongACurveAndStop:
      return "GoToPointAlongACurveAndStop";
    case DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop:
      return "GoToPointInAStraightLineAndStop";
    case DJIWaypointV2FlightPathModeCoordinateTurn:
      return "CoordinateTurn";
    case DJIWaypointV2FlightPathModeGoToFirstPointAlongAStraightLine:
      return "GoToFirstPointAlongAStraightLine";
    case DJIWaypointV2FlightPathModeStraightOut:
      return "StraightOut";
    case DJIWaypointV2FlightPathModeUnknown:
      return "Unknown";
  }
  return "Unknown";
}

void
WaypointV2MissionSample::printWpInfo(const WaypointV2&  wp,
                                     const std::string& prefix)
{
  printf("%s: (%f,%f) relalt: %.02f damp: %d type: %s\n",
         prefix.c_str(),
         rad2deg(wp.latitude),
         rad2deg(wp.longitude),
         wp.relativeHeight,
         wp.dampingDistance,
         wpTypeToString(wp.waypointType).c_str());
}

// Returns (dx, dy) for a vector of length "step" rotated by "angle_rad"
std::pair<double, double>
WaypointV2MissionSample::rotateVector(double step, double angle_rad)
{
  double dx = step * std::cos(angle_rad);
  double dy = step * std::sin(angle_rad);
  return { dx, dy };
}

// input: 0.0 - 1.0 - will be clamped
// 0.0 means overshooting a waypoint
// 1.0 means breaking early to "skip" the waypoint on a curve
// this applies only to coordinated turn waypoint type
uint16_t
WaypointV2MissionSample::getDampingFactor(float32_t dampingFactor,
                                          float32_t wpDistanceM)
{
  double f = std::max(0.0, std::min(1.0, (double)dampingFactor));
  // the damping factor should be less than half the segment distance
  // but making it a bit smaller seems to work more reliably
  // and having a minimum is also more reliable
  // both 0.4 and 0.1 are experimental
  double halfWpDistance = wpDistanceM*0.4*f;
  double minimum = 0.1*wpDistanceM;
  if (halfWpDistance < minimum)
      halfWpDistance = minimum;

  return (uint16_t)(halfWpDistance * 100.0);
}

std::vector<WaypointV2>
WaypointV2MissionSample::generateAngleWaypoints(float32_t step,
                                                float32_t angle_deg,
                                                uint16_t  n_points,
                                                float32_t damping,
                                                float32_t speed,
                                                uint16_t  wp_type,
                                                uint16_t  wp_type_first,
                                                uint16_t  wp_type_last)
{
  // Let's create a vector to store our waypoints in.

  double                  angle_rad = angle_deg * M_PI / 180.0;
  std::vector<WaypointV2> waypointList;
  WaypointV2              wpFirst;
  WaypointV2              wp;

  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition =
    vehiclePtr->subscribe->getValue<TOPIC_GPS_FUSED>();

  DJIWaypointV2FlightPathMode wpTypeFirst;
  DJIWaypointV2FlightPathMode wpType;
  DJIWaypointV2FlightPathMode wpTypeLast;

  //  -----------------------------------------
  //  -----------------------------------------
  //  -----------------------------------------

  if (wp_type_first == 0)
  {
    wpTypeFirst = DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop;
  }
  else if (wp_type_first == 1)
  {
    wpTypeFirst = DJIWaypointV2FlightPathModeGoToPointAlongACurve;
  }
  else // if (wp_type_first == 2)
  {
    wpTypeFirst = DJIWaypointV2FlightPathModeGoToPointAlongACurveAndStop;
  }

  //  -----------------------------------------
  //  -----------------------------------------
  //  -----------------------------------------
  if (wp_type == 0)
  {
    wpType = DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop;
  }
  else if (wp_type == 1)
  {
    wpType = DJIWaypointV2FlightPathModeGoToPointAlongACurve;
  }
  else if (wp_type == 2)
  {
    wpType = DJIWaypointV2FlightPathModeCoordinateTurn;
  }
  else // if (wp_type == 3)
  {
    wpType = DJIWaypointV2FlightPathModeGoToPointAlongACurveAndStop;
  }

  //  -----------------------------------------
  //  -----------------------------------------
  //  -----------------------------------------
  if (wp_type_last == 0)
  {
    wpTypeLast = DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop;
  }
  else if (wp_type_last == 1)
  {
    wpTypeLast = DJIWaypointV2FlightPathModeGoToPointAlongACurve;
  }
  else if (wp_type_last == 2)
  {
    wpTypeLast = DJIWaypointV2FlightPathModeCoordinateTurn;
  }
  else if (wp_type_last == 3)
  {
    wpTypeLast = DJIWaypointV2FlightPathModeStraightOut;
  }
  else // if (wp_type_last == 4)
  {
    wpTypeLast = DJIWaypointV2FlightPathModeGoToPointAlongACurveAndStop;
  }

  // comments:
  // * damping is in cm - so if given in meters, has to be divided by 100.
  //   we can do it 0..1 - and scale based on segment length/2
  //   this might still fail for very short distances
  // * first waypoint cannot be a coordinated turn - will refuse to fly
  // * heading is always along the segment for the first WP
  // * damping seems not to do anything for straight line and curve
  // * WP types:
  //   -coordinated turn: can turn before a WP if damping is high, fly pass
  //    the waypoint if damping is small
  //   -curve: always crosses the waypoint -
  //      damping does nothing
  //   - DJIWaypointV2FlightPathModeGoToPointAlongACurveAndStop
  //    if it overshoots, it will correct itself by moving closer - looks weird
  // * if second (and other) WP are curve, and the first is a straight line -
  //      the line s ignored - it will curve the first segment also
  // * distance between waypoints:
  //   - for straight lines, curves: 0.1m is ok (in sim)
  //   - coordinated turn: 4m (damp 20)
  // * for coordinated turn, the minimum angle between segments should be > 15 deg

  setWaypointV2Defaults(wpFirst);
  wp.headingMode          = DJIWaypointV2HeadingModeAuto;
  wp.heading              = 45.0;
  wpFirst.latitude        = (subscribeGPosition.latitude);
  wpFirst.longitude       = (subscribeGPosition.longitude);
  wpFirst.relativeHeight  = 15;
  wpFirst.waypointType    = wpTypeFirst;
  wpFirst.dampingDistance = 0;
  waypointList.push_back(wpFirst);

  printWpInfo(wpFirst, "start wp");

  // Iterative algorithm
  double X     = step;
  double Y     = 0.0;
  double a_rad = angle_rad;
  for (int i = 0; i < n_points; i++)
  {
    setWaypointV2Defaults(wp);
    wp.headingMode = DJIWaypointV2HeadingModeAuto;
    wp.heading     = 45.0;
    auto [dx, dy]  = rotateVector(step, a_rad);

    if (i % 2)
    {
      X += dx;
      a_rad += angle_rad;
    }
    else
    {
      X -= dx;
      a_rad -= angle_rad;
    }
    Y += dy;

    xyzToWaypointV2(X, Y, 0, wpFirst, wp);

    uint16_t dampingDistance = 500; // 500 is 5m
    if (!waypointList.empty())
    {
      float32_t dist = calculateDistance3D(wp, waypointList.back());
      dampingDistance        = getDampingFactor(damping, dist);
    }
    wp.dampingDistance = dampingDistance;
    wp.waypointType    = wpType;

    // last waypoint
    if (i == (n_points - 1))
    {

      wp.waypointType = wpTypeLast;
    }
    //    else {
    //      waypointList.push_back(wp);
    //    }

    waypointList.push_back(wp);
  }
  /// waypointList.push_back(wpFirst);
  return waypointList;
}

std::vector<DJIWaypointV2Action>
WaypointV2MissionSample::generateWaypointActions(uint16_t actionNum)
{
  std::vector<DJIWaypointV2Action> actionVector;

  for (uint16_t i = 0; i < actionNum; i++)
  {
    DJIWaypointV2SampleReachPointTriggerParam sampleReachPointTriggerParam;
    sampleReachPointTriggerParam.waypointIndex = i;
    sampleReachPointTriggerParam.terminateNum  = 0;

    auto trigger =
      DJIWaypointV2Trigger(DJIWaypointV2ActionTriggerTypeSampleReachPoint,
                           &sampleReachPointTriggerParam);
    auto cameraActuatorParam = DJIWaypointV2CameraActuatorParam(
      DJIWaypointV2ActionActuatorCameraOperationTypeTakePhoto, nullptr);
    auto actuator = DJIWaypointV2Actuator(
      DJIWaypointV2ActionActuatorTypeCamera, 0, &cameraActuatorParam);
    auto action = DJIWaypointV2Action(i, trigger, actuator);
    actionVector.push_back(action);
  }
  return actionVector;
}

void
WaypointV2MissionSample::setWaypointV2Defaults(WaypointV2& waypointV2)
{

  waypointV2.waypointType =
    DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop;
  waypointV2.headingMode = DJIWaypointV2HeadingModeAuto;
  //  waypointV2.headingMode              = DJIWaypointV2HeadingFixed;
  waypointV2.config.useLocalCruiseVel = 0;
  waypointV2.config.useLocalMaxVel    = 0;

  waypointV2.dampingDistance = 0;
  waypointV2.heading         = 0.0;
  waypointV2.turnMode        = DJIWaypointV2TurnModeClockwise;

  waypointV2.pointOfInterest.positionX = 0;
  waypointV2.pointOfInterest.positionY = 0;
  waypointV2.pointOfInterest.positionZ = 0;
  waypointV2.maxFlightSpeed            = 12.0;
  waypointV2.autoFlightSpeed           = 2.0;
}

ErrorCode::ErrorCodeType
WaypointV2MissionSample::getActionRemainMemory(GetRemainRamAck& actionMemory,
                                               int              timeout)
{
  ErrorCode::ErrorCodeType ret =
    vehiclePtr->waypointV2Mission->getActionRemainMemory(actionMemory, timeout);
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("get waypoint v2 action remain memory failed:0x%lX", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return ret;
  }
  else
  {
    DSTATUS("get waypoint v2 action remain memory successfully!");
  }
  return ret;
}
