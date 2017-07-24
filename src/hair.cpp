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

  for (int x = 0; x < particles_count; x++){
    Vector3D pos = Vector3D(x * space, 1.0,  1.0);
    PointMass *m = new PointMass(pos, false);

    point_masses.push_back(*m);
  }

  for (int i = 0; i < particles_count - 1; i++){
    PointMass* pm1 = &(point_masses[i]);
    PointMass* pm2 = &(point_masses[i+1]);
    Spring* s = new Spring(pm1, pm2, space);
    springs.push_back(*s);
  }
}

void Hair::simulate(double frames_per_sec, double simulation_steps, int *particles_count,
                    double *length, float *thickness) {

}