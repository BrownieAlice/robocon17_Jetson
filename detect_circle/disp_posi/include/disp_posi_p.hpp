#ifndef INCLUDED_DISP_POSI_P
#define INCLUDED_DISP_POSI_P

#include <GL/freeglut.h>
#include "detect_circle/MBinput.h"

static void init_glut(int *glut_argc_p, char ** argv, const int disp_width, const int disp_height, const char rel_img_address[], GLuint texture[], const int texture_num, pngInfo *png_info_p);
static void SubCallback(const detect_circle::MBinput& msg);
static void display(void);
static void write_bg(GLuint texture);
static void write_triangle(const int field_width, const int field_height, const float x, const float y, const float theta, const float len, const float alpha, GLuint texture);
static void resize(int width, int height);
static void write_position(const int field_width, const int field_height, const int field_string_x, const int field_string_y, const int disp_width, const int disp_height, const float x_val, const float y_val, const float t_val);

#ifndef M_PI
#define M_PI 3.1415
#endif

#endif
