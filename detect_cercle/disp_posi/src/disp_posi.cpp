/*
======================================================================
Project Name    : display robot's position
File Name       : disp_posi
Encoding        : UTF-8
Creation Date   : 2017/03/04

Copyright © 2017 Alice.
======================================================================
*/

#include <GL/freeglut.h>
#include <GL/glpng.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include "ros/ros.h"
#include "detect_cercle/MBinput.h"
#include "../include/disp_posi_p.h"

namespace
{
  int glut_argc = 1;
  bool now_data = false;
  char field_color;
  float robot_x, robot_y, robot_theta;

  constexpr float alpha_coe = 0.05;
  constexpr float alpha_min = 0.3;
  int alpha_count = (int)((1 - alpha_min) / alpha_coe);

  constexpr int field_width = 15050, field_height = 14150;
  constexpr float field_tri_len = 1;
  constexpr int redu_ratio = 20;
  constexpr int disp_width = (int)ceil((double)field_width / redu_ratio), disp_height = (int)ceil((double)field_height / redu_ratio);
  constexpr float disp_tri_len = field_tri_len / redu_ratio;
  constexpr int main_loop_hz = 60;
  constexpr int texture_num = 2;

  const std::string rel_img_address = "/catkin_ws/fig/field.png";

  pngInfo info;
  GLuint texture[texture_num];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "disp_posi");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("MBdata", 1, SubCallback);
  ros::Rate loop_rate(main_loop_hz);

  glutInit(&glut_argc, argv);

  /* ウィンドウの位置とサイズを指定 */
  glutInitWindowPosition(100, 100);
  glutInitWindowSize(disp_width, disp_height);

  /* ウィンドウを生成 */
  glutCreateWindow("display robot's position");

  /* 背景色を指定: 白 */
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA | GLUT_DOUBLE);
  glClearColor(1.0, 1.0, 1.0, 1.0);

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glGenTextures(texture_num, texture);

  std::string real_img_address;
  const char *home_address =  getenv("HOME");
  if (NULL != home_address)
  {
    real_img_address = home_address;
    real_img_address += rel_img_address;
    texture[0] = pngBind (real_img_address.c_str(), PNG_NOMIPMAP, PNG_ALPHA, &info, GL_CLAMP, GL_NEAREST, GL_NEAREST);
  }
  else
  {
    std::cout << "fail to get home address." << std::endl;
    std::cout << "can't open background image." << std::endl;
  }

  /* 画面を再描写するときに実行される関数を指定
       (初期化、ウィンドウサイズ変更時など) */
  glutDisplayFunc(display);

  glutReshapeFunc(resize);

  while (ros::ok())
  {
    if (1 - alpha_count * alpha_coe > alpha_min){
      alpha_count++;
      glutPostRedisplay();
    }
    ros::spinOnce();

    glutMainLoopEvent();

    loop_rate.sleep();
  }
  return 0;
}

static void SubCallback(const detect_cercle::MBinput& msg)
{
  field_color = msg.color;
  robot_x = msg.x;
  robot_y = msg.y;
  robot_theta = msg.theta;
  now_data = true;
  alpha_count -= 2;
  if (alpha_count < 0)
  {
    alpha_count = 0;
  }

  glutPostRedisplay();
}

static void display(void) {
    /* 画面全体を指定した色で塗りつぶす */
    glClear(GL_COLOR_BUFFER_BIT);
    /* まだ実行されていない命令をすべて実行 */
    glLoadIdentity();

    glEnable(GL_TEXTURE_2D);
    glEnable(GL_ALPHA_TEST);
    glEnable(GL_BLEND);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    write_bg(texture[0]);

    float tri_alpha = 1 - alpha_count * alpha_coe;
    if (tri_alpha > 1)
    {
      tri_alpha = 1;
    }
    else if (tri_alpha < alpha_min)
    {
      tri_alpha = alpha_min;
    }

    if(true == now_data){
      glEnable(GL_POLYGON_SMOOTH);

      glHint(GL_POLYGON_SMOOTH_HINT,GL_NICEST);

      write_triangle(field_width, field_height, robot_x * 1000, robot_y * 1000, robot_theta, disp_tri_len, tri_alpha, texture[1]);

      glDisable(GL_POLYGON_SMOOTH);
    }

    glDisable(GL_BLEND);
    glDisable(GL_ALPHA_TEST);
    glDisable(GL_TEXTURE_2D);
    glutSwapBuffers();
}

static void write_bg(GLuint texture)
{
  glPushMatrix();
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  glBegin(GL_POLYGON);
    glTexCoord2f(0, 1);
    glVertex2d(-1, 1);
    glTexCoord2f(0, 0);
    glVertex2d(-1, -1);
    glTexCoord2f(1, 0);
    glVertex2d(1, -1);
    glTexCoord2f(1, 1);
    glVertex2d(1, 1);
  glEnd();
  glPopMatrix();
}

static void write_triangle(const int field_width, const int field_height, const float x, const float y, const float theta, const float len, const float alpha, GLuint texture)
{
  glPushMatrix();
  glBindTexture(GL_TEXTURE_2D, texture);
  glTranslated(-1, -1, 0);
  glTranslated(x / field_width * 2, y / field_height * 2, 0);
  glRotated(theta / M_PI * 180, 0.0, 0.0, 1.0);
  glBegin(GL_TRIANGLES);
    glColor4f(0, 0.7, 0, alpha);
    glVertex2f( len / sqrt(3)       , 0  );
    glVertex2f(-len / (2 * sqrt(3)) , len / 2);
    glVertex2f(-len / (2 * sqrt(3)) , -len / 2);
  glEnd();
  glPopMatrix();
}

static void resize(int width, int height)
{
  glutReshapeWindow(disp_width, disp_height);
}
