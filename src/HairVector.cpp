#include <iostream>
#include <math.h>
#include <random>
#include <vector>
#include "./CGL/matrix3x3.h"

#include "HairVector.h"

using namespace std;

HairVector::~HairVector() {
  hair_vector->clear();
}

void HairVector::buildGrid(Vector3D start_pos) {
  Hair* hair = new Hair(particles_count, length);
  bool pinned = true;
  double space = -1.0;
  hair->avg_spring_length = length / (particles_count - 1);
  Vector3D new_pos = start_pos;
  for (int i = 0; i < particles_count; i++){
//    double x_pos = -5.0 + ((i % 2) * 10.0);
    double x_pos = (i % 2) * (((double) rand()/RAND_MAX) * 5.0 + 2.5);
    double y_pos = space;

//    double r = (double) rand()/RAND_MAX;
//    double z_pos = (r * 2.0 - 1.0)/ 1000.0;
    double z_pos = 0;

    PointMass *m = new PointMass(new_pos, pinned);
    Vector3D pos = new_pos + (Vector3D(x_pos, y_pos, z_pos).unit() * hair->avg_spring_length);
    new_pos = pos;

    hair->point_masses.push_back(*m);
    pinned = false;
  }

  // create stretch springs
  for (int i = 0; i < particles_count - 1; i++){
    PointMass* pm1 = &(hair->point_masses[i]);
    PointMass* pm2 = &(hair->point_masses[i+1]);
    double spring_length = (pm2->position - pm1->position).norm();
    Spring* s = new Spring(pm1, pm2, spring_length);
    hair->springs.push_back(*s);
  }

  //  create support springs 
  for (int j = 0; j < hair->point_masses.size() - 2; j++) { 
    PointMass* pm1 = &(hair->point_masses[j]);
    PointMass* pm2 = &(hair->point_masses[j+2]); 
    double spring_length = (pm2->start_position - pm1->start_position).norm(); 
    Spring* s = new Spring(pm1, pm2, spring_length); 
    hair->support_springs.push_back(*s); 
  }

//  restBendSmoothingFunction();
//  restCoreSmoothingFunction();
//
//  for (int i = 0; i < point_masses.size() - 1; i++) {
//    Vector3D rest_edge = point_masses[i+1].start_position - point_masses[i].start_position;
//
//    if (i == 0) {
//      (&point_masses[i])->ref_vector = Vector3D();
//    } else {
//      Vector3D frame_1 = (point_masses[i].rest_bend_smoothed_position - point_masses[i-1].rest_bend_smoothed_position).unit();
//      Vector3D frame_2 = cross(frame_1, Vector3D(0, 0, 1)).unit();
//
//      double frame_coord[] = {
//              frame_1.x, frame_2.x, 0.0,
//              frame_1.y, frame_2.y, 0.0,
//              frame_1.z, frame_2.z, 1.0
//      };
//
//      (&point_masses[i])->ref_vector = Matrix3x3(frame_coord).T() * rest_edge;
//    }
//  }
  hair_vector->push_back(hair);
}


void HairVector::simulate(double frames_per_sec, double simulation_steps, vector<Vector3D> external_accelerations) {
  for (Hair* hair : *hair_vector) {
    hair->externalForces(frames_per_sec, simulation_steps, external_accelerations, density);
    hair->restCoreSmoothingFunction(ac);
    if (enable_stretch_constraints) { hair->stretchSpring(frames_per_sec, simulation_steps, ks, cs, ab); }
    if (enable_support_constraints) { hair->supportSpring(frames_per_sec, simulation_steps, kb, cb, ab); }
//    if (enable_bending_constraints) { hair->bendSpring(frames_per_sec, simulation_steps, kb, cb, ab); }
    if (enable_core_constraints) { hair->coreSpring(frames_per_sec, simulation_steps, kc, cc, ac); }
    hair->updatePositions(frames_per_sec, simulation_steps, density, damping);
  }
}

