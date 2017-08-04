#include <iostream>
#include <math.h>
#include <random>
#include <vector>
#include "./CGL/matrix3x3.h"

#include "hair.h"

using namespace std;

Hair::~Hair() {
  point_masses.clear();
  springs.clear();
}

void Hair::buildGrid() {
  bool pinned = true;
  double space = -5.0;
  avg_spring_length = length / (particles_count - 1);
  Vector3D last_pos = offset;
  for (int i = 0; i < particles_count; i++){
    double x_pos = -5.0 + ((i % 2) * 10.0);
//    double x_pos = (y%2) * (rand() / RAND_MAX);
    double y_pos = space;
    double z_pos = (i%2) * 1.0;

    Vector3D pos = last_pos + (Vector3D(x_pos, y_pos, z_pos).unit() * avg_spring_length);
    last_pos = pos;
    PointMass *m = new PointMass(pos, pinned);

    point_masses.push_back(*m);
    pinned = false;
  }

  // create stretch springs
  for (int i = 0; i < particles_count - 1; i++){
    PointMass* pm1 = &(point_masses[i]);
    PointMass* pm2 = &(point_masses[i+1]);
    double spring_length = (pm2->position - pm1->position).norm();
    Spring* s = new Spring(pm1, pm2, spring_length);
    springs.push_back(*s);
  }

  //  create support springs 
  for (int j = 0; j < point_masses.size() - 2; j++) { 
    PointMass* pm1 = &(point_masses[j]);
    PointMass* pm2 = &(point_masses[j+2]); 
    double spring_length = (pm2->position - pm1->position).norm(); 
    Spring* s = new Spring(pm1, pm2, spring_length); 
    support_springs.push_back(*s); 
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
}

void Hair::externalForces(double frames_per_sec, double simulation_steps, vector<Vector3D> external_accelerations, double density) {
  double mass = length * density / (double) particles_count;

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
}

void Hair::stretchSpring(double frames_per_sec, double simulation_steps, double ks, double cs) {
  double delta_t = 1.0f / frames_per_sec / simulation_steps;
  for (Spring &s : springs) {
    double current_length = (s.pm_a->position - s.pm_b->position).norm();
    Vector3D delta_v = s.pm_b->velocity(delta_t) - s.pm_a->velocity(delta_t);
    Vector3D unit_dir = (s.pm_b->position - s.pm_a->position).unit();
//    Vector3D force = (ks * (current_length - s.rest_length) * unit_dir);
//    Vector3D dampingApplied = (unit_dir * cs * dot(delta_v, unit_dir));
    Vector3D forceApplied = (ks * (current_length - s.rest_length) * unit_dir) + (unit_dir * cs * dot(delta_v, unit_dir));
    s.pm_a->forces += forceApplied;
    s.pm_b->forces += -forceApplied;
  }
}

void Hair::supportSpring(double frames_per_sec, double simulation_steps, double kb, double cb, double ab) { 
  positionSmoothingFunction(ab);   // to show smoothed curve

  double delta_t = 1.0f / frames_per_sec / simulation_steps; 
  for (Spring &s : support_springs) { 
    double current_length = (s.pm_a->position - s.pm_b->position).norm();
    Vector3D delta_v = s.pm_b->velocity(delta_t) - s.pm_a->velocity(delta_t); 
    Vector3D unit_dir = (s.pm_b->position - s.pm_a->position).unit(); 
    Vector3D forceApplied = (kb * (current_length - s.rest_length) * unit_dir) + (unit_dir * cb * dot(delta_v, unit_dir)); 
 //    Vector3D forceApplied = (kb * (current_length - s.rest_length) * unit_dir);
     s.pm_a->forces += forceApplied;
     s.pm_b->forces += -forceApplied;
   } 
}

void Hair::bendSpring(double frames_per_sec, double simulation_steps, double kb, double cb, double ab) {
  positionSmoothingFunction(ab);
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  for (int i = 1; i < point_masses.size() - 1; i++) {
    PointMass* pm = &point_masses[i];
    PointMass* pm_before = &point_masses[i-1];
    PointMass* pm_after = &point_masses[i+1];

    Vector3D frame_1 =  (pm->smoothed_position - pm_before->smoothed_position).unit();
    Vector3D frame_2 = cross(frame_1, Vector3D(0, 0, 1)).unit();

    double frame_coord[] = {
            frame_1.x, frame_2.x, 0.0,
            frame_1.y, frame_2.y, 0.0,
            frame_1.z, frame_2.z, 1.0
    };

    Matrix3x3 ref_frame = Matrix3x3(frame_coord);

    Vector3D target_vector = (ref_frame * pm->ref_vector);
    Vector3D target_pos = pm->position + target_vector;
    Vector3D delta_v = pm_after->velocity(delta_t) - pm->velocity(delta_t);

    Vector3D edge = pm_after->start_position - pm->start_position;
    Vector3D forceApplied = (kb * (edge - target_vector)) +
                            cb * (delta_v - (dot(delta_v, edge.unit())) * edge.unit());

    pm->forces += -forceApplied;
    pm_after->forces += forceApplied;
    pm_after->bend_target_pos = target_pos;
  }
}

void Hair::coreSpring(double frames_per_sec, double simulation_steps, double kc, double cc, double ac) {
  positionSmoothingFunction(ac);
  velocitySmoothingFunction(frames_per_sec, simulation_steps, ac);

  for (Spring &s: springs) {
    PointMass *pm = s.pm_a;
    PointMass *pm_after = s.pm_b;
    Vector3D smooth_rest_edge = pm_after->rest_core_smoothed_position - pm->rest_core_smoothed_position;    // rest bi
    Vector3D smooth_curr_edge = pm_after->smoothed_position - pm->smoothed_position;    // bi

    Vector3D forceApplied = (kc * (smooth_curr_edge.norm() - smooth_rest_edge.norm()) * smooth_curr_edge.unit()) +
                            (cc * dot(pm->smoothed_velocity, smooth_curr_edge.unit()) * smooth_curr_edge.unit());

    pm->forces += -forceApplied;
    pm_after->forces += forceApplied;
    pm->bend_target_pos = pm->position - forceApplied.unit();
  }
}

void Hair::restBendSmoothingFunction(double ab) {
  for (int i = 0; i < point_masses.size()-1; i++) {
    if (i == 0) {
      PointMass* pm = &point_masses[0];
      PointMass* pm1 = &point_masses[1];
      pm->rest_bend_smoothing_amt = pm1->start_position - pm->start_position;
      pm->rest_bend_smoothed_position = pm->start_position;
    } else {
      double beta = min(1.0, 1.0-exp(-avg_spring_length/ab));
      double minus_beta = 1.0 - beta;

      PointMass* pm = &point_masses[i];
      PointMass* pm_before = &point_masses[i-1];

      Vector3D a = 2.0 * minus_beta * pm_before->rest_bend_smoothing_amt;

      Vector3D b;
      if (i == 1) {
        b = pow(minus_beta, 2.0) * point_masses[0].rest_bend_smoothing_amt;
      } else {
        b = pow(minus_beta, 2.0) * point_masses[i-2].rest_bend_smoothing_amt;
      }

      Vector3D pm_diff = point_masses[i+1].start_position - pm->start_position;
      Vector3D c = pow(beta, 2.0) * pm_diff;
      pm->rest_bend_smoothing_amt = a - b + c;
      pm->rest_bend_smoothed_position = pm_before->rest_bend_smoothed_position + pm_before->rest_bend_smoothing_amt;
    }

    if (i == point_masses.size()-2) { // last loop
      PointMass* pm = &point_masses[i+1];
      PointMass* pm_before = &point_masses[i];
      pm->rest_bend_smoothed_position = pm_before->rest_bend_smoothed_position + pm_before->rest_bend_smoothing_amt;
    }
  }
}

void Hair::restCoreSmoothingFunction(double ac) {
  for (int i = 0; i < point_masses.size()-1; i++) {
    if (i == 0) {
      PointMass* pm = &point_masses[0];
      PointMass* pm1 = &point_masses[1];
      pm->rest_core_smoothing_amt = pm1->start_position - pm->start_position;
      pm->rest_core_smoothed_position = pm->start_position;
    } else {
      double beta = min(1.0, 1.0-exp(-avg_spring_length/ac));
      double minus_beta = 1.0 - beta;

      PointMass* pm = &point_masses[i];
      PointMass* pm_before = &point_masses[i-1];
      Vector3D pm_diff = point_masses[i+1].start_position - pm->start_position;

      Vector3D a = (2.0 * minus_beta * point_masses[i-1].rest_core_smoothing_amt);
      Vector3D b;

      if (i == 1) {
        b = (pow(minus_beta, 2.0) * point_masses[0].rest_core_smoothing_amt);
      } else {
        b = (pow(minus_beta, 2.0) * point_masses[i-2].rest_core_smoothing_amt);
      }

      Vector3D c = (pow(beta, 2.0) * pm_diff);
      pm->rest_core_smoothing_amt = a - b + c;
      pm->rest_core_smoothed_position = pm_before->rest_core_smoothed_position + pm_before->rest_core_smoothing_amt;
    }

    if (i == point_masses.size()-2) { // last loop
      PointMass* pm = &point_masses[i+1];
      PointMass* pm_before = &point_masses[i];
      pm->rest_core_smoothed_position = pm_before->rest_core_smoothed_position + pm_before->rest_core_smoothing_amt;
    }
  }
}

void Hair::positionSmoothingFunction(double bend_constant) {
  double smoothing_constant;

  for (int i = 0; i < point_masses.size()-1; i++) {
    if (i == 0) {
      PointMass* pm = &point_masses[0];
      PointMass* pm1 = &point_masses[1];
      pm->smoothing_amt = pm1->position - pm->position;
      pm->smoothed_position = pm->position;
    } else {
      double beta = min(1.0, 1.0-exp(-avg_spring_length/bend_constant));
      double minus_beta = 1.0 - beta;

      PointMass* pm = &point_masses[i];
      PointMass* pm_before = &point_masses[i-1];
      Vector3D pm_diff = point_masses[i+1].position - pm->position;

      Vector3D a = (2.0 * minus_beta * point_masses[i-1].smoothing_amt);
      Vector3D b;

      if (i == 1) {
        b = (pow(minus_beta, 2.0) * point_masses[0].smoothing_amt);
      } else {
        b = (pow(minus_beta, 2.0) * point_masses[i-2].smoothing_amt);
      }

      Vector3D c = (pow(beta, 2.0) * pm_diff);
      pm->smoothing_amt = a - b + c;
      pm->smoothed_position = pm_before->smoothed_position + pm_before->smoothing_amt;
    }

    if (i == point_masses.size()-2) { // last loop
      PointMass* pm = &point_masses[i+1];
      PointMass* pm_before = &point_masses[i];
      pm->smoothed_position = pm_before->smoothed_position + pm_before->smoothing_amt;
    }
  }
}

void Hair::velocitySmoothingFunction(double frames_per_sec, double simulation_steps, double ac) {
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  for (int i = 0; i < point_masses.size()-1; i++) {
    if (i == 0) {
      PointMass* pm = &point_masses[0];
      PointMass* pm1 = &point_masses[1];
      pm->smoothing_amt = pm1->velocity(delta_t) - pm->velocity(delta_t);
      pm->smoothed_velocity = pm->velocity(delta_t);
    } else {
      double beta = min(1.0, 1.0-exp(-avg_spring_length/ac));
      double minus_beta = 1.0 - beta;

      PointMass* pm = &point_masses[i];
      PointMass* pm_before = &point_masses[i-1];
      Vector3D pm_diff = point_masses[i+1].velocity(delta_t) - pm->velocity(delta_t);

      Vector3D a = (2.0 * minus_beta * point_masses[i-1].smoothing_amt);
      Vector3D b;

      if (i == 1) {
        b = (pow(minus_beta, 2.0) * point_masses[0].smoothing_amt);
      } else {
        b = (pow(minus_beta, 2.0) * point_masses[i-2].smoothing_amt);
      }

      Vector3D c = (pow(beta, 2.0) * pm_diff);
      pm->smoothing_amt = a - b + c;
      pm->smoothed_velocity = pm_before->smoothed_velocity + pm_before->smoothing_amt;
    }

    if (i == point_masses.size()-2) { // last loop
      PointMass* pm = &point_masses[i+1];
      PointMass* pm_before = &point_masses[i];
      pm->smoothed_velocity = pm_before->smoothed_velocity + pm_before->smoothing_amt;
    }
  }
}

void Hair::updatePositions(double frames_per_sec, double simulation_steps, double density) {
  double mass = length * density / (double) particles_count;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  for (PointMass &pm : point_masses) {
    if (!pm.pinned) {
      Vector3D temp = pm.position;
      pm.position += (pm.position - pm.last_position) + ((pm.forces / mass) * pow(delta_t, 2.0));   // Verlet Integration
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
        s.pm_a->position += (s.pm_b->position - s.pm_a->position).unit() * (diff / 2.0);
        s.pm_b->position += (s.pm_a->position - s.pm_b->position).unit() * (diff / 2.0);
      }
    }
  }

  for (Spring &s : support_springs) {
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
        s.pm_a->position += (s.pm_b->position - s.pm_a->position).unit() * (diff / 2.0);
        s.pm_b->position += (s.pm_a->position - s.pm_b->position).unit() * (diff / 2.0);
      }
    }
  }
}

