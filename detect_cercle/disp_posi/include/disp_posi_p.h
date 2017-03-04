#ifndef DISP_POSISION_OF_ROBOT
#define DISP_POSISION_OF_ROBOT

static void SubCallback(const detect_cercle::MBinput& msg);
static void display(void);
static void write_bg(GLuint texture);
static void write_triangle(const int field_width, const int field_height, const float x, const float y, const float theta, const float len, const float alpha, GLuint texture);
static void resize(int width, int height);

#ifndef M_PI
#define M_PI 3.1415
#endif

#endif
