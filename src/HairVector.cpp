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

void HairVector::simulate(double frames_per_sec, double simulation_steps, vector<Vector3D> external_accelerations) {
  for (Hair* hair : *hair_vector) {
    hair->externalForces(frames_per_sec, simulation_steps, external_accelerations, density);
    hair->restCoreSmoothingFunction(ac);
    if (enable_stretch_constraints) { hair->stretchSpring(frames_per_sec, simulation_steps, ks, cs); }
    if (enable_support_constraints) { hair->supportSpring(frames_per_sec, simulation_steps, kb, cb, ab); }
    if (enable_bending_constraints) { hair->bendSpring(frames_per_sec, simulation_steps, kb, cb, ab); }
    if (enable_core_constraints) { hair->coreSpring(frames_per_sec, simulation_steps, kc, cc, ac); }
    hair->updatePositions(frames_per_sec, simulation_steps, density);
  }
}

