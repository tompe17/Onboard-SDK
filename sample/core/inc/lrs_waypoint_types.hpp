
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
};


#endif // LRS_WAYPOINT_TYPES
