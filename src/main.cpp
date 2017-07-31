#include <getopt.h>
#include <iostream>
#include <fstream>
#include <nanogui/nanogui.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <unordered_set>

#include "CGL/CGL.h"
#include "clothSimulator.h"
#include "json.hpp"
#include "hair.h"

typedef uint32_t gid_t;

using namespace std;
using namespace nanogui;

using json = nlohmann::json;

#define msg(s) cerr << "[ClothSim] " << s << endl;

const string HAIR = "hair";

const unordered_set<string> VALID_KEYS = {HAIR};

ClothSimulator *app = nullptr;
GLFWwindow *window = nullptr;
Screen *screen = nullptr;

void error_callback(int error, const char* description) {
  puts(description);
}

void createGLContexts() {
  if (!glfwInit()) {
    return;
  }

  glfwSetTime(0);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  glfwWindowHint(GLFW_SAMPLES, 0);
  glfwWindowHint(GLFW_RED_BITS, 8);
  glfwWindowHint(GLFW_GREEN_BITS, 8);
  glfwWindowHint(GLFW_BLUE_BITS, 8);
  glfwWindowHint(GLFW_ALPHA_BITS, 8);
  glfwWindowHint(GLFW_STENCIL_BITS, 8);
  glfwWindowHint(GLFW_DEPTH_BITS, 24);
  glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

  // Create a GLFWwindow object
  window = glfwCreateWindow(800, 800, "Cloth Simulator", nullptr, nullptr);
  if (window == nullptr) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    throw std::runtime_error("Could not initialize GLAD!");
  }
  glGetError(); // pull and ignore unhandled errors like GL_INVALID_ENUM

  glClearColor(0.2f, 0.25f, 0.3f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  // Create a nanogui screen and pass the glfw pointer to initialize
  screen = new Screen();
  screen->initialize(window, true);

  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  glViewport(0, 0, width, height);
  glfwSwapInterval(1);
  glfwSwapBuffers(window);
}

void setGLFWCallbacks() {
  glfwSetCursorPosCallback(window, [](GLFWwindow *, double x, double y) {
      if (!screen->cursorPosCallbackEvent(x, y)) {
        app->cursorPosCallbackEvent(x / screen->pixelRatio(),
                                    y / screen->pixelRatio());
      }
  });

  glfwSetMouseButtonCallback(
          window, [](GLFWwindow *, int button, int action, int modifiers) {
              if (!screen->mouseButtonCallbackEvent(button, action, modifiers) ||
                  action == GLFW_RELEASE) {
                app->mouseButtonCallbackEvent(button, action, modifiers);
              }
          });

  glfwSetKeyCallback(
          window, [](GLFWwindow *, int key, int scancode, int action, int mods) {
              if (!screen->keyCallbackEvent(key, scancode, action, mods)) {
                app->keyCallbackEvent(key, scancode, action, mods);
              }
          });

  glfwSetCharCallback(window, [](GLFWwindow *, unsigned int codepoint) {
      screen->charCallbackEvent(codepoint);
  });

  glfwSetDropCallback(window,
                      [](GLFWwindow *, int count, const char **filenames) {
                          screen->dropCallbackEvent(count, filenames);
                          app->dropCallbackEvent(count, filenames);
                      });

  glfwSetScrollCallback(window, [](GLFWwindow *, double x, double y) {
      if (!screen->scrollCallbackEvent(x, y)) {
        app->scrollCallbackEvent(x, y);
      }
  });

  glfwSetFramebufferSizeCallback(window,
                                 [](GLFWwindow *, int width, int height) {
                                     screen->resizeCallbackEvent(width, height);
                                     app->resizeCallbackEvent(width, height);
                                 });
}

void usageError(const char *binaryName) {
  printf("Usage: %s [options]\n", binaryName);
  printf("Required program options:\n");
  printf("  -f     <STRING>    Filename of scene");
  printf("\n");
  exit(-1);
}

void incompleteObjectError(const char *object, const char *attribute) {
  cout << "Incomplete " << object << " definition, missing " << attribute << endl;
  exit(-1);
}

void loadObjectsFromFile(string filename, Hair *hair) {
  // Read JSON from file
  ifstream i(filename);
  json j;
  i >> j;

  // Loop over objects in scene
  for (json::iterator it = j.begin(); it != j.end(); ++it) {
    string key = it.key();

    // Check that object is valid
    unordered_set<string>::const_iterator query = VALID_KEYS.find(key);
    if (query == VALID_KEYS.end()) {
      cout << "Invalid scene object found: " << key << endl;
      exit(-1);
    }

    // Retrieve object
    json object = it.value();

    if (key == HAIR) {
      int particles_count;
      float thickness;
      double length, density, cs, ks, ab, cb, kb;
      vector<vector<int>> positions;

      auto it_particles_count = object.find("particles count");
      if (it_particles_count != object.end()) {
        particles_count = *it_particles_count;
      } else {
        incompleteObjectError("hair", "particles_count");
      }

      auto it_length = object.find("length");
      if (it_length != object.end()) {
        length = *it_length;
      } else {
        incompleteObjectError("hair", "length");
      }

      auto it_thickness = object.find("thickness");
      if (it_thickness != object.end()) {
        thickness = *it_thickness;
      } else {
        incompleteObjectError("hair", "thickness");
      }

      auto it_density = object.find("density");
      if (it_density != object.end()) {
        density = *it_density;
      } else {
        incompleteObjectError("hair", "density");
      }

      auto it_cs = object.find("cs");
      if (it_cs != object.end()) {
        cs = *it_cs;
      } else {
        incompleteObjectError("hair", "cs");
      }

      auto it_ks = object.find("ks");
      if (it_ks != object.end()) {
        ks = *it_ks;
      } else {
        incompleteObjectError("hair", "ks");
      }

      auto it_cb = object.find("cb");
      if (it_cb != object.end()) {
        cb = *it_cb;
      } else {
        incompleteObjectError("hair", "cb");
      }

      auto it_kb = object.find("kb");
      if (it_kb != object.end()) {
        kb = *it_kb;
      } else {
        incompleteObjectError("hair", "kb");
      }

      auto it_ab = object.find("bend smoothing amount");
      if (it_ab != object.end()) {
        ab = *it_ab;
      } else {
        incompleteObjectError("hair", "bend smoothing amount");
      }

      auto it_positions = object.find("positions");
      if (it_positions != object.end()) {
        vector<json> points = *it_positions;
        for (auto pt : points) {
          vector<int> point = pt;
          positions.push_back(point);
        }
      }

      hair->particles_count = particles_count;
      hair->length = length;
      hair->thickness = thickness;
      hair->density = density;
      hair->cs = cs;
      hair->ks = ks;
      hair->cb = cb;
      hair->kb = kb;
      hair->ab = ab;

      i.close();
    }
  }
}

int main(int argc, char **argv) {
  Hair hair;

  if (argc == 1) { // No arguments, default initialization
    string default_file_name = "../scene/pinned2.json";
    loadObjectsFromFile(default_file_name, &hair);
  } else {
    int c;

    while ((c = getopt (argc, argv, "f:")) != -1) {
      switch (c) {
        case 'f':
          loadObjectsFromFile(optarg, &hair);
          break;
        default:
          usageError(argv[0]);
      }
    }
  }

  glfwSetErrorCallback(error_callback);

  createGLContexts();

  // Initialize the Hair object
  hair.buildGrid();

  // Initialize the ClothSimulator object
  app = new ClothSimulator(screen);

  app->loadHair(&hair);
  app->init();

  // Call this after all the widgets have been defined

  screen->setVisible(true);
  screen->performLayout();

  // Attach callbacks to the GLFW window

  setGLFWCallbacks();

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    glClearColor(0.25f, 0.25f, 0.25f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    app->drawContents();

    // Draw nanogui
    screen->drawContents();
    screen->drawWidgets();

    glfwSwapBuffers(window);

    if (!app->isAlive()) {
      glfwSetWindowShouldClose(window, 1);
    }
  }

  return 0;
}