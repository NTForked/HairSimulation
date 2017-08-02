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
  void bendSpring(double frames_per_sec, double simulation_steps);
  void coreSpring(double frames_per_sec, double simulation_steps);
  void restBendSmoothingFunction();
  void restCoreSmoothingFunction();
  void positionSmoothingFunction(int bend);   // parameter bend = 1 for bending springs, and 0 for core springs
  void velocitySmoothingFunction(double frames_per_sec, double simulation_steps);

  int particles_count;
  double length;
  float thickness;
  vector<PointMass> point_masses;
  vector<Spring> springs;
  double avg_spring_length;
  double density;

  // stretch springs
  double cs;
  double ks;

  // bend springs
  double ab; // bend smoothing amount
  double cb;
  double kb;

  // core springs
  double ac; // core smoothing amount
  double cc;
  double kc;

  bool enable_stretch_constraints = true;
  bool enable_bending_constraints = true;
  bool enable_core_constraints = true;
  };

#endif //CLOTHSIM_HAIR_H
