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
Hair(int particles_count, double length)
        : particles_count(particles_count), length(length) {}
~Hair();

void externalForces(double frames_per_sec, double simulation_steps, vector<Vector3D> external_accelerations, double density);
void stretchSpring(double frames_per_sec, double simulation_steps, double ks, double cs);
void supportSpring(double frames_per_sec, double simulation_steps, double kb, double cb, double bend_constant);
void bendSpring(double frames_per_sec, double simulation_steps, double kb, double cb, double bend_constant);
void coreSpring(double frames_per_sec, double simulation_steps, double kc, double cc, double bend_constant);
void restBendSmoothingFunction(double ab);
void restCoreSmoothingFunction(double ac);
void positionSmoothingFunction(double bend_constant);
void velocitySmoothingFunction(double frames_per_sec, double simulation_steps, double ac);
void updatePositions(double frames_per_sec, double simulation_steps, double density, double damping);

int particles_count;
double length;
double avg_spring_length;
vector<Spring> springs;
vector<Spring> support_springs;
vector<PointMass> point_masses;
};

#endif //CLOTHSIM_HAIR_H