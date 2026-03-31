
#ifndef LRS_WAYPOINT_TYPES
#define LRS_WAYPOINT_TYPES

#include <dji_vehicle.hpp>

struct GenParams
{
  float32_t damp;
  int timeout;
  uint16_t n_points;
  uint16_t wp_type;
  uint16_t wp_type_first;
  uint16_t wp_type_last;
  float speed;
  float step;
  float angle_deg;

  std::vector<float> steps;
  std::vector<float> lengths;
  std::vector<float> angles_deg;
  std::vector<float> speeds;
  std::vector<float> damps;
  std::vector<float> alts;
  std::vector<uint16_t> types;

};


#endif // LRS_WAYPOINT_TYPES
