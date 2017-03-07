#ifndef DISP_POSISION_OF_ROBOT
#define DISP_POSISION_OF_ROBOT

#include <GL/freeglut.h>
#include "detect_cercle/MBinput.h"

static void init_glut(char ** argv);
static void SubCallback(const detect_cercle::MBinput& msg);
static void display(void);
static void write_bg(GLuint texture);
static void write_triangle(const int field_width, const int field_height, const float x, const float y, const float theta, const float len, const float alpha, GLuint texture);
static void resize(int width, int height);
static void write_position(const int field_width, const int field_height, const int field_string_x, const int field_string_y, const int disp_width, const int disp_height, const float x_val, const float y_val, const float t_val);

#ifndef M_PI
#define M_PI 3.1415
#endif

#endif
