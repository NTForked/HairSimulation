#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "hair.h"

using namespace std;

Hair::~Hair() {
  point_masses.clear();
  springs.clear();
}

void Hair::buildGrid() {
  double space = this->length / (double) (particles_count - 1);

  bool pinned = true;
  for (int y = 0; y < particles_count; y++){
    double x_pos = ((double) rand() * 0.6) / (RAND_MAX);
    double y_pos = -1.0 * (y * space);
    Vector3D pos = Vector3D(x_pos, y_pos, 1.0);
    PointMass *m = new PointMass(pos, pinned);

    point_masses.push_back(*m);
    pinned = false;
  }

  for (int i = 0; i < particles_count - 1; i++){
    PointMass* pm1 = &(point_masses[i]);
    PointMass* pm2 = &(point_masses[i+1]);
    Spring* s = new Spring(pm1, pm2, space);
    springs.push_back(*s);
  }
}

void Hair::simulate(double frames_per_sec, double simulation_steps, vector<Vector3D> external_accelerations) {
  double mass = length * density / (double) particles_count;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // Compute total force acting on each point mass.
  Vector3D totalExtAccel = Vector3D();
  for (int accel = 0; accel < external_accelerations.size(); accel++) {
    totalExtAccel += external_accelerations[accel];
  }
  totalExtAccel *= mass;

  for (PointMass &pm : point_masses) {
    pm.forces = Vector3D();
    pm.forces += totalExtAccel;
  }

  for (Spring &s : springs) {
    double current_length = (s.pm_a->position - s.pm_b->position).norm();
    Vector3D delta_v = s.pm_b->velocity(delta_t) - s.pm_a->velocity(delta_t);
    Vector3D unit_dir = (s.pm_b->position - s.pm_a->position).unit();
    double forceApplied = ks * (current_length - s.rest_length) + ((damping) * dot(delta_v, unit_dir));
    s.pm_a->forces += unit_dir * forceApplied;
    s.pm_b->forces += (s.pm_a->position - s.pm_b->position).unit() * forceApplied;
  }

  for (PointMass& pm : point_masses) {
    if (!pm.pinned) {
      Vector3D temp = pm.position;
      pm.position += (pm.position - pm.last_position) + ((pm.forces/mass) * (delta_t * delta_t));
      pm.last_position = temp;
    }
  }

  for (Spring &s : springs) {
    if (s.pm_a->pinned && !s.pm_b->pinned) {   // a pinned, b loose
      double springLength = (s.pm_a->position - s.pm_b->position).norm();
      if (springLength > s.rest_length * 1.1) {
        double diff = springLength - (s.rest_length * 1.1);
        s.pm_b->position += (s.pm_a->position - s.pm_b->position).unit() * diff;
      }
    } else {
      double springLength = (s.pm_a->position - s.pm_b->position).norm();
      if (springLength > s.rest_length * 1.1) {
        double diff = springLength - (s.rest_length * 1.1);
        s.pm_a->position += (s.pm_b->position - s.pm_a->position).unit() * (diff/2.0);
        s.pm_b->position += (s.pm_a->position - s.pm_b->position).unit() * (diff/2.0);
      }
    }
  }

}