#ifndef CLOTHSIM_HAIRVECTOR_H
#define CLOTHSIM_HAIRVECTOR_H

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "spring.h"
#include "hair.h"

using namespace CGL;
using namespace std;

struct HairVector {
HairVector() {
  hair_vector = new vector<Hair *>();
}
HairVector(vector<Hair*> *hair_vector)
        : hair_vector(hair_vector) {}
~HairVector();

void simulate(double frames_per_sec, double simulation_steps, vector<Vector3D> external_accelerations);

vector<Hair*> * hair_vector;

bool enable_stretch_constraints = true;
bool enable_support_constraints = true;
bool enable_bending_constraints = false;
bool enable_core_constraints = true;

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
};

#endif //CLOTHSIM_HAIRVECTOR_H
