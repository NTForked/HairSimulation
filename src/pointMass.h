#ifndef POINTMASS_H
#define POINTMASS_H

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "CGL/vector3D.h"

using namespace CGL;

// Forward declarations
class Halfedge;
class Edge;

struct PointMass {
  PointMass(Vector3D position, bool pinned)
      : pinned(pinned), start_position(position), position(position),
        last_position(position) {}

  Vector3D normal();
  Vector3D velocity(double delta_t) {
    return (position - last_position) / delta_t;
  }

  // static values
  bool pinned;
  Vector3D start_position;
  Vector3D rest_bend_smoothing_amt;
  Vector3D rest_bend_smoothed_position;
  Vector3D rest_core_smoothing_amt;
  Vector3D rest_core_smoothed_position;
  Vector3D ref_vector;

// dynamic values
  Vector3D position;
  Vector3D last_position;
  Vector3D forces;
  Vector3D smoothing_amt;
  Vector3D smoothed_position;
  Vector3D smoothed_velocity;


  // debug values
  Vector3D bend_target_pos;
  Vector3D frame_1;
  Vector3D frame_2;

  // mesh reference
  Halfedge *halfedge;
};

#endif /* POINTMASS_H */
