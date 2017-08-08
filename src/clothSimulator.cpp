#include <cmath>
#include <glad/glad.h>

#include <CGL/vector3D.h>
#include <nanogui/nanogui.h>

#include "clothSimulator.h"

#include "camera.h"
#include "misc/camera_info.h"

using namespace nanogui;
using namespace std;

ClothSimulator::ClothSimulator(Screen *screen) {
  this->screen = screen;

  // Initialize OpenGL buffers and shaders

  wireframeShader.initFromFiles("Wireframe", "../shaders/camera.vert",
                                "../shaders/wireframe.frag");
  normalShader.initFromFiles("Normal", "../shaders/camera.vert",
                             "../shaders/normal.frag");
  phongShader.initFromFiles("Phong", "../shaders/camera.vert",
                            "../shaders/phong.frag");

  shaders.push_back(wireframeShader);
  shaders.push_back(normalShader);
  shaders.push_back(phongShader);

  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);
}

ClothSimulator::~ClothSimulator() {
  for (auto shader : shaders) {
    shader.free();
  }

  if (hairs) delete hairs;
}


void ClothSimulator::loadHair(HairVector *hairs) { this->hairs = hairs; }

/**
 * Initializes the cloth simulation and spawns a new thread to separate
 * rendering from simulation.
 */
void ClothSimulator::init() {
  // Initialize GUI
  initGUI(screen);
  screen->setSize(default_window_size);

  // Initialize camera

  CGL::Collada::CameraInfo camera_info;
  camera_info.hFov = 50;
  camera_info.vFov = 35;
  camera_info.nClip = 0.01;
  camera_info.fClip = 10000;

  // Try to intelligently figure out the camera target

  Vector3D avg_pm_position(0, 0, 0);

  for (Hair *hair : *(hairs->hair_vector)) {
    for (auto &pm : hair->point_masses) {
      avg_pm_position += pm.position / hair->point_masses.size();
    }
  }

  avg_pm_position /= (double) hairs->hair_vector->size();
  CGL::Vector3D target(avg_pm_position.x, avg_pm_position.y / 2, avg_pm_position.z);
  CGL::Vector3D c_dir(0., 0., 0.);

  Hair* hair = (*(hairs->hair_vector))[0];
  canonical_view_distance = hair->length * 0.9;
  scroll_rate = canonical_view_distance / 10;

  view_distance = canonical_view_distance * 2;
  min_view_distance = canonical_view_distance / 10.0;
  max_view_distance = canonical_view_distance * 20.0;

  // canonicalCamera is a copy used for view resets

  camera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z), view_distance,
               min_view_distance, max_view_distance);
  canonicalCamera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z),
                        view_distance, min_view_distance, max_view_distance);

  screen_w = default_window_size(0);
  screen_h = default_window_size(1);

  camera.configure(camera_info, screen_w, screen_h);
  canonicalCamera.configure(camera_info, screen_w, screen_h);
}

bool ClothSimulator::isAlive() { return is_alive; }

void ClothSimulator::drawContents() {
  glEnable(GL_DEPTH_TEST);

  if (!is_paused) {
    vector<Vector3D> external_accelerations = {gravity};

    double change_pos = 1000.0;
    if (left_pressed || down_pressed) {
      change_pos *= -1.0;
    }

    Vector3D accel;
    if (left_pressed || right_pressed) {
      accel = Vector3D(change_pos, 0, 0);
    } else if (up_pressed || down_pressed) {
      accel = Vector3D(0, change_pos, 0);
    }
    external_accelerations.push_back(accel);

    for (int i = 0; i < simulation_steps; i++) {
      hairs->simulate(frames_per_sec, simulation_steps, external_accelerations);
    }

    external_accelerations = {gravity};
    left_pressed = false;
    right_pressed = false;
    up_pressed = false;
    down_pressed = false;
  }

  // Bind the active shader

  GLShader shader = shaders[activeShader];
  shader.bind();

  // Prepare the camera projection matrix

  Matrix4f model;
  model.setIdentity();

  Matrix4f view = getViewMatrix();
  Matrix4f projection = getProjectionMatrix();

  Matrix4f viewProjection = projection * view;

  shader.setUniform("model", model);
  shader.setUniform("viewProjection", viewProjection);

  switch (activeShader) {
  case WIREFRAME:
    drawHead(shader);
//    drawRestPose(shader);
//    drawStretchSprings(shader);
//    drawSupportSprings(shader);
    drawSmoothCurve(shader);
//    drawLocalFrame(shader);
//    drawTargetVector(shader);
    break;
  }
}

void ClothSimulator::drawHead(GLShader &shader) {
  Vector3D center = Vector3D();

  for (Hair* hair : *(hairs->hair_vector)) {
    center += hair->point_masses[0].start_position;
  }
  center /= hairs->num_hairs;

  int num_tris = 25;
  double theta = 2.0 * PI / num_tris;
  MatrixXf head(3, num_tris * 3);

  int col_count = 0;
  Vector3D p = center;
  double s = 10.0;

  double next_x, next_y;
  for (int j = 0; j < num_tris; j++) {
    head.col(col_count) << p.x, p.y, -1.0;
    next_x = p.x + (s * cos(j * theta));
    next_y = p.y + (s * sin(j * theta));
    col_count++;
    head.col(col_count) << next_x, next_y, -1.0;
    next_x = p.x + (s * cos((j + 1) * theta));
    next_y = p.y + (s * sin((j + 1) * theta));
    col_count++;
    head.col(col_count) << next_x, next_y, -1.0;
    col_count++;
  }

  shader.setUniform("in_color", nanogui::Color(239.0f/255, 209.0f/255, 199.0f/255, 1.0f));
  shader.uploadAttrib("in_position", head);
  shader.drawArray(GL_TRIANGLES, 0, num_tris * 3);

}

void ClothSimulator::drawRestPose(GLShader &shader) {
  int total_springs = 0;
  int total_particles = 0;
  for (Hair* hair : *(hairs->hair_vector)) {
    total_springs += hair->particles_count - 1;
    total_particles += hair->particles_count;
  }

  MatrixXf particle_positions(3, 3);
  MatrixXf rest_springs(3, total_springs * 2);

  int si = 0;
  // Draw springs as lines
  for (Hair* hair : *(hairs->hair_vector)) {
    for (int i = 0; i < hair->springs.size(); i++) {
      Spring s = hair->springs[i];

      Vector3D pa = s.pm_a->start_position;
      Vector3D pb = s.pm_b->start_position;

      particle_positions.col(0) << pa.x+0.1, pa.y, pa.z;
      particle_positions.col(1) << pa.x, pa.y+0.1, pa.z;
      particle_positions.col(2) << pa.x-0.1, pa.y, pa.z;

      rest_springs.col(si) << pa.x, pa.y, pa.z;
      rest_springs.col(si + 1) << pb.x, pb.y, pb.z;

      shader.setUniform("in_color", nanogui::Color(0.0f, 0.0f, 1.0f, 1.0f));
      shader.uploadAttrib("in_position", particle_positions);
      shader.drawArray(GL_TRIANGLES, 0, 3);

      if (i == hair->springs.size() - 1) {
        particle_positions.col(0) << pb.x+0.1, pb.y, pb.z;
        particle_positions.col(1) << pb.x, pb.y+0.1, pb.z;
        particle_positions.col(2) << pb.x-0.1, pb.y, pb.z;

        shader.setUniform("in_color", nanogui::Color(0.0f, 0.0f, 1.0f, 1.0f));
        shader.uploadAttrib("in_position", particle_positions);
        shader.drawArray(GL_TRIANGLES, 0, 3);
      }

      si += 2;
    }
  }

  shader.setUniform("in_color", nanogui::Color(0.0f, 0.0f, 1.0f, 1.0f));
  shader.uploadAttrib("in_position", rest_springs);
  shader.drawArray(GL_LINES, 0, total_springs * 2);
}

void ClothSimulator::drawStretchSprings(GLShader &shader) {
  for (Hair* hair : *(hairs->hair_vector)) {
    int num_springs = hair->particles_count - 1;
    int num_particles = hair->particles_count;

    MatrixXf particle_positions(3, 3);
    MatrixXf stretch_springs(3, num_springs * 2);

    int si = 0;
    // Draw springs as lines
    for (int i = 0; i < num_springs; i++) {
      Spring s = hair->springs[i];

      Vector3D pa = s.pm_a->position;
      Vector3D pb = s.pm_b->position;

      particle_positions.col(0) << pa.x + 0.5, pa.y, pa.z;
      particle_positions.col(1) << pa.x, pa.y + 0.5, pa.z;
      particle_positions.col(2) << pa.x - 0.5, pa.y, pa.z;

      stretch_springs.col(si) << pa.x, pa.y, pa.z;
      stretch_springs.col(si + 1) << pb.x, pb.y, pb.z;

      shader.setUniform("in_color", nanogui::Color(1.0f, 1.0f, 1.0f, 1.0f));
      shader.uploadAttrib("in_position", particle_positions);
      shader.drawArray(GL_TRIANGLES, 0, 3);

      if (i == num_springs - 1) {
        particle_positions.col(0) << pb.x + 0.5, pb.y, pb.z;
        particle_positions.col(1) << pb.x, pb.y + 0.5, pb.z;
        particle_positions.col(2) << pb.x - 0.5, pb.y, pb.z;

        shader.setUniform("in_color", nanogui::Color(1.0f, 1.0f, 1.0f, 1.0f));
        shader.uploadAttrib("in_position", particle_positions);
        shader.drawArray(GL_TRIANGLES, 0, 3);
      }

      si += 2;
    }

    shader.setUniform("in_color", nanogui::Color(1.0f, 1.0f, 1.0f, 1.0f));
    shader.uploadAttrib("in_position", stretch_springs);
    shader.drawArray(GL_LINES, 0, num_springs * 2);
  }
}

void ClothSimulator::drawSupportSprings(GLShader &shader) {
  for (Hair* hair : *(hairs->hair_vector)) {
    int num_springs = hair->support_springs.size();

    MatrixXf particle_positions(3, 3);
    MatrixXf support_springs(3, num_springs * 2);

    int si = 0;
    // Draw springs as lines
    for (int i = 0; i < num_springs; i++) {
      Spring s = hair->support_springs[i];

      Vector3D pa = s.pm_a->position;
      Vector3D pb = s.pm_b->position;

      particle_positions.col(0) << pa.x + 0.1, pa.y, pa.z;
      particle_positions.col(1) << pa.x, pa.y + 0.1, pa.z;
      particle_positions.col(2) << pa.x - 0.1, pa.y, pa.z;

      support_springs.col(si) << pa.x, pa.y, pa.z;
      support_springs.col(si + 1) << pb.x, pb.y, pb.z;

      shader.setUniform("in_color", nanogui::Color(0.0f, 0.0f, 0.0f, 1.0f));
      shader.uploadAttrib("in_position", particle_positions);
      shader.drawArray(GL_TRIANGLES, 0, 3);

      if (i == num_springs - 1) {
        particle_positions.col(0) << pb.x + 0.1, pb.y, pb.z;
        particle_positions.col(1) << pb.x, pb.y + 0.1, pb.z;
        particle_positions.col(2) << pb.x - 0.1, pb.y, pb.z;

        shader.setUniform("in_color", nanogui::Color(0.0f, 0.0f, 0.0f, 1.0f));
        shader.uploadAttrib("in_position", particle_positions);
        shader.drawArray(GL_TRIANGLES, 0, 3);
      }

      si += 2;
    }

    shader.setUniform("in_color", nanogui::Color(0.0f, 0.0f, 0.0f, 1.0f));
    shader.uploadAttrib("in_position", support_springs);
    shader.drawArray(GL_LINES, 0, num_springs * 2);
  }
}

void ClothSimulator::drawSmoothCurve(GLShader &shader) {
  int total_springs = 0;
  for (Hair* hair : *(hairs->hair_vector)) {
    total_springs += hair->particles_count - 1;
  }

//  MatrixXf smoothed_positions(3, 3);
//  MatrixXf smooth_curve(3, total_springs * 2);
//
//  int si = 0;
//  // Draw springs as lines
//  for (Hair* hair : *(hairs->hair_vector)) {
//    for (int i = 0; i < hair->springs.size(); i++) {
//      Spring s = hair->springs[i];
//
//      Vector3D smoothed_pa = s.pm_a->smoothed_position;
//      Vector3D smoothed_pb = s.pm_b->smoothed_position;
//      Vector3D center = Vector3D(smoothed_pa.x - (smoothed_pa.x - smoothed_pb.x)/2.0,
//                                 smoothed_pa.y - (smoothed_pa.y - smoothed_pb.y)/2.0, 0.0);
//
//
//    smoothed_positions.col(0) << center.x+0.5, center.y, center.z + 0.05;
//    smoothed_positions.col(1) << center.x, center.y+0.5, center.z + 0.05;
//    smoothed_positions.col(2) << center.x-0.5, center.y, center.z + 0.05;
//
//      smooth_curve.col(si) << smoothed_pa.x, smoothed_pa.y, smoothed_pa.z;
//      smooth_curve.col(si + 1) << smoothed_pb.x, smoothed_pb.y, smoothed_pb.z;
//
//    shader.setUniform("in_color", nanogui::Color(0.0f, 0.0f, 0.0f, 1.0f));
//    shader.uploadAttrib("in_position", smoothed_positions);
//    shader.drawArray(GL_TRIANGLES, 0, 3);
//
//    if (i == hair->springs.size() - 1) {
//      smoothed_positions.col(0) << smoothed_pb.x + 0.5, smoothed_pb.y, smoothed_pb.z + 0.05;
//      smoothed_positions.col(1) << smoothed_pb.x, smoothed_pb.y + 0.5, smoothed_pb.z + 0.05;
//      smoothed_positions.col(2) << smoothed_pb.x - 0.5, smoothed_pb.y, smoothed_pb.z + 0.05;
//
//      shader.setUniform("in_color", nanogui::Color(0.0f, 0.0f, 0.0f, 1.0f));
//      shader.uploadAttrib("in_position", smoothed_positions);
//      shader.drawArray(GL_TRIANGLES, 0, 3);
//    }
//
//      si += 2;
//    }
//  }
//
//  shader.setUniform("in_color", nanogui::Color(0.698f, 0.133f, 0.133f, 1.0f));
//  shader.uploadAttrib("in_position", smooth_curve);
//  shader.drawArray(GL_LINES, 0, total_springs * 2);

  //draw curves
  int lineAmount = 20;
  float z = 0.0f;
  for (Hair* hair : *(hairs->hair_vector)) {
    int total_segments = (int) hair->springs.size() * lineAmount * 2;
    MatrixXf curve(3, total_segments);
    int si2 = 0;
    for (int i = 0; i < hair->springs.size(); i++) {
      Spring s = hair->springs[i];
      Vector3D pos_a = s.pm_a->smoothed_position;
      Vector3D pos_b = s.pm_b->smoothed_position;
      Vector3D center = Vector3D(pos_a.x - (pos_a.x - pos_b.x)/2.0,
                                 pos_a.y - (pos_a.y - pos_b.y)/2.0, 0.0);
      float theta = (float) (PI / float(lineAmount));
      float radius = float((pos_a - pos_b).norm() / 2.0);
      Vector3D v1 = Vector3D(radius, 0, 0).unit();
      Vector3D v2 = (pos_a - center).unit();
      float start_angle = atan2f(v2.y, v2.x) - atan2f(v1.y, v1.x);

      float x = radius * cosf(start_angle);
      float y = radius * sinf(start_angle);

      for(float j = 0.0f; j <= PI; j+=theta) {
        curve.col(si2) << center.x + x, center.y + y, z;

        x = radius * cosf(start_angle + j + theta);
        y = radius * sinf(start_angle + j + theta);

        curve.col(si2 + 1) << center.x + x, center.y + y, z;

        si2 += 2;
      }
    }
  shader.setUniform("in_color", nanogui::Color(0.698f, 0.133f, 0.133f, 1.0f));
//    shader.setUniform("in_color", nanogui::Color(0.0f, 1.0f, 0.0f, 1.0f));
    shader.uploadAttrib("in_position", curve);
    shader.drawArray(GL_LINE_STRIP, 0, total_segments);
  }


}

void ClothSimulator::drawLocalFrame(GLShader &shader) {
  for (Hair* hair : *(hairs->hair_vector)) {
    int num_springs = hair->particles_count - 1;
    int num_particles = hair->particles_count;

    MatrixXf frame_1_positions(3, 3);
    MatrixXf frame_2_positions(3, 3);

    MatrixXf frame_1_vector(3, num_particles * 2);
    MatrixXf frame_2_vector(3, num_particles * 2);

    int si = 0;
    // Draw springs as lines
    for (int i = 0; i < num_particles; i++) {
      PointMass pm = hair->point_masses[i];

      Vector3D smoothed_pm = pm.smoothed_position;

      Vector3D frame_1 = pm.frame_1;
      Vector3D frame_2 = pm.frame_2;

      frame_1_positions.col(0) << frame_1.x + 0.1, frame_1.y, frame_1.z;
      frame_1_positions.col(1) << frame_1.x, frame_1.y + 0.1, frame_1.z;
      frame_1_positions.col(2) << frame_1.x - 0.1, frame_1.y, frame_1.z;

      frame_2_positions.col(0) << frame_2.x + 0.1, frame_2.y, frame_2.z;
      frame_2_positions.col(1) << frame_2.x, frame_2.y + 0.1, frame_2.z;
      frame_2_positions.col(2) << frame_2.x - 0.1, frame_2.y, frame_2.z;

      frame_1_vector.col(si) << smoothed_pm.x, smoothed_pm.y, smoothed_pm.z;
      frame_1_vector.col(si + 1) << frame_1.x, frame_1.y, frame_1.z;

      frame_2_vector.col(si) << smoothed_pm.x, smoothed_pm.y, smoothed_pm.z;
      frame_2_vector.col(si + 1) << frame_2.x, frame_2.y, frame_2.z;

      shader.setUniform("in_color", nanogui::Color(0.0f, 1.0f, 0.0f, 1.0f));
      shader.uploadAttrib("in_position", frame_1_positions);
      shader.drawArray(GL_TRIANGLES, 0, 3);

      shader.setUniform("in_color", nanogui::Color(1.0f, 0.0f, 0.0f, 1.0f));
      shader.uploadAttrib("in_position", frame_2_positions);
      shader.drawArray(GL_TRIANGLES, 0, 3);

      si += 2;
    }

    shader.setUniform("in_color", nanogui::Color(0.0f, 1.0f, 0.0f, 1.0f));
    shader.uploadAttrib("in_position", frame_1_vector);
    shader.drawArray(GL_LINES, 0, num_springs * 2);

    shader.setUniform("in_color", nanogui::Color(1.0f, 0.0f, 0.0f, 1.0f));
    shader.uploadAttrib("in_position", frame_2_vector);
    shader.drawArray(GL_LINES, 0, num_springs * 2);
  }
}

void ClothSimulator::drawTargetVector(GLShader &shader) {
  for (Hair* hair : *(hairs->hair_vector)) {
    int num_springs = hair->particles_count - 1;
    int num_particles = hair->point_masses.size();

    MatrixXf target_positions(3, 3);
    MatrixXf target_vector(3, num_particles * 2);

    int si = 0;
    // Draw springs as lines
    for (int i = 0; i < num_particles; i++) {
      PointMass pm = hair->point_masses[i];

      Vector3D pm_pos = pm.position;
      Vector3D target = pm.bend_target_pos;

      target_positions.col(0) << target.x + 0.1, target.y, target.z;
      target_positions.col(1) << target.x, target.y + 0.1, target.z;
      target_positions.col(2) << target.x - 0.1, target.y, target.z;

      target_vector.col(si) << pm_pos.x, pm_pos.y, pm_pos.z;
      target_vector.col(si + 1) << target.x, target.y, target.z;

      shader.setUniform("in_color", nanogui::Color(1.0f, 0.0f, 1.0f, 1.0f));
      shader.uploadAttrib("in_position", target_positions);
      shader.drawArray(GL_TRIANGLES, 0, 3);

      si += 2;
    }

    shader.setUniform("in_color", nanogui::Color(1.0f, 0.0f, 1.0f, 1.0f));
    shader.uploadAttrib("in_position", target_vector);
    shader.drawArray(GL_LINES, 0, num_springs * 2);
  }
}

// ----------------------------------------------------------------------------
// CAMERA CALCULATIONS
//
// OpenGL 3.1 deprecated the fixed pipeline, so we lose a lot of useful OpenGL
// functions that have to be recreated here.
// ----------------------------------------------------------------------------

void ClothSimulator::resetCamera() { camera.copy_placement(canonicalCamera); }

Matrix4f ClothSimulator::getProjectionMatrix() {
  Matrix4f perspective;
  perspective.setZero();

  double near = camera.near_clip();
  double far = camera.far_clip();

  double theta = camera.v_fov() * M_PI / 360;
  double range = far - near;
  double invtan = 1. / tanf(theta);

  perspective(0, 0) = invtan / camera.aspect_ratio();
  perspective(1, 1) = invtan;
  perspective(2, 2) = -(near + far) / range;
  perspective(3, 2) = -1;
  perspective(2, 3) = -2 * near * far / range;
  perspective(3, 3) = 0;

  return perspective;
}

Matrix4f ClothSimulator::getViewMatrix() {
  Matrix4f lookAt;
  Matrix3f R;

  lookAt.setZero();

  // Convert CGL vectors to Eigen vectors
  // TODO: Find a better way to do this!

  CGL::Vector3D c_pos = camera.position();
  CGL::Vector3D c_udir = camera.up_dir();
  CGL::Vector3D c_target = camera.view_point();

  Vector3f eye(c_pos.x, c_pos.y, c_pos.z);
  Vector3f up(c_udir.x, c_udir.y, c_udir.z);
  Vector3f target(c_target.x, c_target.y, c_target.z);

  R.col(2) = (eye - target).normalized();
  R.col(0) = up.cross(R.col(2)).normalized();
  R.col(1) = R.col(2).cross(R.col(0));

  lookAt.topLeftCorner<3, 3>() = R.transpose();
  lookAt.topRightCorner<3, 1>() = -R.transpose() * eye;
  lookAt(3, 3) = 1.0f;

  return lookAt;
}

// ----------------------------------------------------------------------------
// EVENT HANDLING
// ----------------------------------------------------------------------------

bool ClothSimulator::cursorPosCallbackEvent(double x, double y) {
  if (left_down && !middle_down && !right_down) {
    if (ctrl_down) {
      mouseRightDragged(x, y);
    } else {
      mouseLeftDragged(x, y);
    }
  } else if (!left_down && !middle_down && right_down) {
    mouseRightDragged(x, y);
  } else if (!left_down && !middle_down && !right_down) {
    mouseMoved(x, y);
  }

  mouse_x_prev = mouse_x;
  mouse_y_prev = mouse_y;
  mouse_x = x;
  mouse_y = y;

  return true;
}

bool ClothSimulator::mouseButtonCallbackEvent(int button, int action,
                                              int modifiers) {
  switch (action) {
  case GLFW_PRESS:
    switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT:
      left_down = true;
      break;
    case GLFW_MOUSE_BUTTON_MIDDLE:
      middle_down = true;
      break;
    case GLFW_MOUSE_BUTTON_RIGHT:
      right_down = true;
      break;
    }
    return true;

  case GLFW_RELEASE:
    switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT:
      left_down = false;
      break;
    case GLFW_MOUSE_BUTTON_MIDDLE:
      middle_down = false;
      break;
    case GLFW_MOUSE_BUTTON_RIGHT:
      right_down = false;
      break;
    }
    return true;
  }

  return false;
}

void ClothSimulator::mouseMoved(double x, double y) { y = screen_h - y; }

void ClothSimulator::mouseLeftDragged(double x, double y) {
  double dx = x - mouse_x_prev;
  double dy = y - mouse_y_prev;

  camera.rotate_by(-dy * (PI / screen_h), -dx * (PI / screen_w));
}

void ClothSimulator::mouseRightDragged(double x, double y) {
  camera.move_by(mouse_x - x, y - mouse_y, canonical_view_distance);
}

bool ClothSimulator::keyCallbackEvent(int key, int scancode, int action,
                                      int mods) {
  ctrl_down = (bool)(mods & GLFW_MOD_CONTROL);

  if (action == GLFW_PRESS) {
    switch (key) {
    case GLFW_KEY_ESCAPE:
      is_alive = false;
      break;
//    case 'r':
//    case 'R':
//      cloth->reset();
//      break;
    case ' ':
      resetCamera();
      break;
    case 'p':
    case 'P':
      is_paused = !is_paused;
      break;
    case 'n':
    case 'N':
      if (is_paused) {
        is_paused = false;
        drawContents();
        is_paused = true;
      }
      break;
      case GLFW_KEY_LEFT:
        left_pressed = true;
        break;
      case GLFW_KEY_RIGHT:
        right_pressed = true;
        break;
      case GLFW_KEY_UP:
        up_pressed = true;
        break;
      case GLFW_KEY_DOWN:
        down_pressed = true;
        break;
    }
  }

  return true;
}

bool ClothSimulator::dropCallbackEvent(int count, const char **filenames) {
  return true;
}

bool ClothSimulator::scrollCallbackEvent(double x, double y) {
  camera.move_forward(y * scroll_rate);
  return true;
}

bool ClothSimulator::resizeCallbackEvent(int width, int height) {
  screen_w = width;
  screen_h = height;

  camera.set_screen_size(screen_w, screen_h);
  return true;
}

void ClothSimulator::initGUI(Screen *screen) {
  Window *window = new Window(screen, "Settings");
  window->setPosition(Vector2i(15, 15));
  window->setLayout(new GroupLayout(15, 6, 14, 5));

  // Spring types
  new Label(window, "Spring types", "sans-bold");

  {
    Button *b = new Button(window, "stretch");
    b->setFlags(Button::ToggleButton);
    b->setPushed(hairs->enable_stretch_constraints);
    b->setFontSize(14);
    b->setChangeCallback(
        [this](bool state) { hairs->enable_stretch_constraints = state; });

    b = new Button(window, "support");
    b->setFlags(Button::ToggleButton);
    b->setPushed(hairs->enable_support_constraints);
    b->setFontSize(14);
    b->setChangeCallback(
            [this](bool state) { hairs->enable_support_constraints = state; });

//    b = new Button(window, "bending");
//    b->setFlags(Button::ToggleButton);
//    b->setPushed(hairs->enable_bending_constraints);
//    b->setFontSize(14);
//    b->setChangeCallback(
//        [this](bool state) { hairs->enable_bending_constraints = state; });

    b = new Button(window, "core");
    b->setFlags(Button::ToggleButton);
    b->setPushed(hairs->enable_core_constraints);
    b->setFontSize(14);
    b->setChangeCallback(
        [this](bool state) { hairs->enable_core_constraints = state; });
  }

  // Mass-spring, smoothing constants parameters

  new Label(window, "Parameters", "sans-bold");

  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);

    new Label(panel, "density :", "sans-bold");
    FloatBox<double> *fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(hairs->density / 10);
    fb->setUnits("g/cm^2");
    fb->setSpinnable(true);
    fb->setCallback([this](float value) { hairs->density = (double)(value * 10); });

    new Label(panel, "ab :", "sans-bold");
    fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(hairs->ab);
//    fb->setUnits("N/m");
    fb->setSpinnable(true);
    fb->setMinValue(0);
    fb->setCallback([this](float value) { hairs->ab = value; });

    new Label(panel, "ac :", "sans-bold");
    fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(hairs->ac);
//    fb->setUnits("N/m");
    fb->setSpinnable(true);
    fb->setMinValue(0);
    fb->setCallback([this](float value) { hairs->ac = value; });
  }

  // Simulation constants

  new Label(window, "Simulation", "sans-bold");

  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);

    new Label(panel, "frames/s :", "sans-bold");

    IntBox<int> *fsec = new IntBox<int>(panel);
    fsec->setEditable(true);
    fsec->setFixedSize(Vector2i(100, 20));
    fsec->setFontSize(14);
    fsec->setValue(frames_per_sec);
    fsec->setSpinnable(true);
    fsec->setCallback([this](int value) { frames_per_sec = value; });

    new Label(panel, "steps/frame :", "sans-bold");

    IntBox<int> *num_steps = new IntBox<int>(panel);
    num_steps->setEditable(true);
    num_steps->setFixedSize(Vector2i(100, 20));
    num_steps->setFontSize(14);
    num_steps->setValue(simulation_steps);
    num_steps->setSpinnable(true);
    num_steps->setMinValue(0);
    num_steps->setCallback([this](int value) { simulation_steps = value; });
  }

  // Damping & spring constants slider and textbox
  new Label(window, "ks", "sans-bold");
  {
    Widget *panel = new Widget(window);
    panel->setLayout(
        new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));

    Slider *slider = new Slider(panel);
    slider->setValue(hairs->ks);
    slider->setFixedWidth(105);

    TextBox *percentage = new TextBox(panel);
    percentage->setFixedWidth(75);
    percentage->setValue(to_string((hairs->ks/5000000.0) - 1.0));
//    percentage->setUnits("%");
    percentage->setFontSize(14);

    slider->setCallback([percentage](float value) { percentage->setValue(std::to_string(value)); });
    slider->setFinalCallback([&](float value) {
      hairs->ks = ((value + 1.0) * 5000000.0);
//       cout << "Final slider value: " << hair->ks << endl;
    });
  }

  new Label(window, "kb", "sans-bold");
  {
    Widget *panel = new Widget(window);
    panel->setLayout(
            new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));

    Slider *slider = new Slider(panel);
    slider->setValue(hairs->kb);
    slider->setFixedWidth(105);

    TextBox *percentage = new TextBox(panel);
    percentage->setFixedWidth(75);
    percentage->setValue(to_string((hairs->kb - 100.0) / 71900.0));
    percentage->setUnits("%");
    percentage->setFontSize(14);

    slider->setCallback([percentage](float value) { percentage->setValue(std::to_string(value)); });
    slider->setFinalCallback([&](float value) {
        hairs->kb = (value * 71900.0) + 100.0;
//        cout << "Final slider value: " << hair->kb << endl;
    });
  }

  new Label(window, "kc", "sans-bold");
  {
    Widget *panel = new Widget(window);
    panel->setLayout(
            new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));

    Slider *slider = new Slider(panel);
    slider->setValue(hairs->kc);
    slider->setFixedWidth(105);

    TextBox *percentage = new TextBox(panel);
    percentage->setFixedWidth(75);
    percentage->setValue(to_string((hairs->kc - 15000.0) / 585000.0));
    percentage->setUnits("%");
    percentage->setFontSize(14);

    slider->setCallback([percentage](float value) { percentage->setValue(std::to_string(value)); });
    slider->setFinalCallback([&](float value) {
        hairs->kc = (value * 585000.0) + 15000.0;
//        cout << "Final slider value: " << hair->kc << endl;
    });
  }

  new Label(window, "damping", "sans-bold");
  {
    Widget *panel = new Widget(window);
    panel->setLayout(
            new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));

    Slider *slider = new Slider(panel);
    slider->setValue(hairs->damping);
    slider->setFixedWidth(105);

    TextBox *percentage = new TextBox(panel);
    percentage->setFixedWidth(75);
    percentage->setValue(to_string(hairs->damping));
    percentage->setUnits("%");
    percentage->setFontSize(14);

    slider->setCallback([percentage](float value) { percentage->setValue(std::to_string(value)); });
    slider->setFinalCallback([&](float value) {
        hairs->damping = value;
//        cout << "Final slider value: " << hairs->damping << endl;
    });
  }


  }

  // Gravity
//  new Label(window, "Gravity", "sans-bold");
//  {
//    Widget *panel = new Widget(window);
//    GridLayout *layout =
//        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
//    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
//    layout->setSpacing(0, 10);
//    panel->setLayout(layout);
//
//    new Label(panel, "x :", "sans-bold");
//    FloatBox<double> *fb = new FloatBox<double>(panel);
//    fb->setEditable(true);
//    fb->setFixedSize(Vector2i(100, 20));
//    fb->setFontSize(14);
//    fb->setValue(gravity.x);
//    fb->setUnits("m/s^2");
//    fb->setSpinnable(true);
//    fb->setCallback([this](float value) { gravity.x = value; });
//
//    new Label(panel, "y :", "sans-bold");
//    fb = new FloatBox<double>(panel);
//    fb->setEditable(true);
//    fb->setFixedSize(Vector2i(100, 20));
//    fb->setFontSize(14);
//    fb->setValue(gravity.y);
//    fb->setUnits("m/s^2");
//    fb->setSpinnable(true);
//    fb->setCallback([this](float value) { gravity.y = value; });
//
//    new Label(panel, "z :", "sans-bold");
//    fb = new FloatBox<double>(panel);
//    fb->setEditable(true);
//    fb->setFixedSize(Vector2i(100, 20));
//    fb->setFontSize(14);
//    fb->setValue(gravity.z);
//    fb->setUnits("m/s^2");
//    fb->setSpinnable(true);
//    fb->setCallback([this](float value) { gravity.z = value; });
//  }

  // Appearance

//  new Label(window, "Appearance", "sans-bold");
//
//  {
//    ComboBox *cb = new ComboBox(window, {"Wireframe", "Normals", "Shaded"});
//    cb->setFontSize(14);
//    cb->setCallback(
//        [this, screen](int idx) { activeShader = static_cast<e_shader>(idx); });
//  }
//
//  new Label(window, "Color", "sans-bold");
//
//  {
//    ColorWheel *cw = new ColorWheel(window, color);
//    cw->setCallback(
//        [this](const nanogui::Color &color) { this->color = color; });
//  }

