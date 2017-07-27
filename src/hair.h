#ifndef CLOTHSIM_HAIR_H
#define CLOTHSIM_HAIR_H

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "spring.h"

using namespace CGL;
using namespace std;

struct Hair {
  Hair() {}
  Hair(int particles_count, double length, float thickness)
          : particles_count(particles_count), length(length), thickness(thickness) {}
  ~Hair();

  void buildGrid();

  void simulate(double frames_per_sec, double simulation_steps, vector<Vector3D> external_accelerations);
  void stretchSpring(double frames_per_sec, double simulation_steps);
  void smoothingFunction();
  void update_bending_positions();

  int particles_count;
  double length;
  float thickness;
  vector<PointMass> point_masses;
  vector<Spring> springs;

  // Mass-spring parameters
  double density;
  double damping;
  double ks;
  double ab; // bend smoothing amount
  };

#endif //CLOTHSIM_HAIR_H
