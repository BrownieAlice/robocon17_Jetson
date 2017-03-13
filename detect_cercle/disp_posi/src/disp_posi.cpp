/*
======================================================================
Project Name    : display robot's position
File Name       : disp_posi
Encoding        : UTF-8
Creation Date   : 2017/03/07

Copyright © 2017 Alice.
======================================================================
*/

#include <GL/freeglut.h>
#include <GL/glpng.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>
#include "ros/ros.h"
#include "detect_cercle/MBinput.h"
#include "../include/disp_posi_p.h"

namespace
{
  namespace param
  {
    int glut_argc = 1;
    // glutに渡す引数の個数.

    constexpr float alpha_coe = 0.025;
    // alpha_countを透明度に変換するときの比例定数.
    constexpr float alpha_min = 0.3;
    // 透明度の最低値.
    constexpr int alpha_count_margin_max = 10;
    // 何回MBから情報を受け取らなかったか,この回数だけ受け取れなかったら透明度を下げ始める.

    constexpr int field_width = 15050, field_height = 14150;
    // フィールドそのものの横/縦の大きさ[mm].
    constexpr float field_tri_len = 1;
    // フィールド図で示す自機の三角形の1辺の大きさ[m].
    constexpr int redu_ratio = 20;
    // フィールドの大きさをウィンドウのピクセルに変換する際の縮小比.
    constexpr int disp_width = (int)ceil((double)field_width / redu_ratio), disp_height = (int)ceil((double)field_height / redu_ratio);
    // ウィンドウの横/縦のピクセル数.
    constexpr float disp_tri_len = field_tri_len / redu_ratio;
    // ウィンドウの自機の三角形のピクセル数.

    constexpr int field_string_x = 10000, field_string_y = 10000;

    constexpr int main_loop_hz = 60;
    // メイン関数を実行する周波数.実質的には描画のフレームレート.
    constexpr int texture_num = 2;
    // テクスチャの数.

    constexpr char rel_img_address[] = "/catkin_ws/fig/field.png";
    // ホームディレクトリから見たフィールド図の位置.
  }

  namespace var
  {
    bool now_data = false;
    // データを受け取ったか否か.
    char field_color;
    // フィールドの色.
    float robot_x, robot_y, robot_theta;
    // MBから渡されたロボットの自己位置,姿勢角.

    int alpha_count = (int)((1 - param::alpha_min) / param::alpha_coe);
    // 透明度をMBからデータが来なかった秒数によって決める.そのカウント用変数.
    int alpha_count_margin = 0;
    // メインループが60Hz程度だがMBからは10msごとにしか情報が来ない.そのため数回情報が来なかったとしても無視する.そのための変数.

    GLuint texture[param::texture_num];
    // テクスチャを格納する配列.
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "disp_posi");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("MBdata", 1, SubCallback);
  ros::Rate loop_rate(param::main_loop_hz);
  // ROS設定

  pngInfo png_info;
  // pngデータの情報.

  init_glut(&param::glut_argc, argv, param::disp_width, param::disp_height, param::rel_img_address, var::texture, param::texture_num, &png_info);
  // glut設定

  while (ros::ok())
  {
    if (1 - var::alpha_count * param::alpha_coe > param::alpha_min){
      // alpha_countがある程度小さい時(無限にカウントし続けるとオーバーフローするのでその対策)
      if(param::alpha_count_margin_max <= var::alpha_count_margin)
      {
        // 規定回数以上連続でMBの情報を受け取れなかった時(数回MBの情報が受け取れなくてもこのループの周波数が高いだけの可能性もあるので).
        var::alpha_count++;
        glutPostRedisplay();
        // 再描画の必要性を主張.
      }
      else
      {
        // 規定回数未満だけ連続でMBの情報を受け取れなかった時.
        var::alpha_count_margin++;
      }
    }
    ros::spinOnce();

    glutMainLoopEvent();
    // 再描画が必要なら行う.

    loop_rate.sleep();
  }
  return 0;
}

static void init_glut(int *glut_argc_p, char ** argv, const int disp_width, const int disp_height, const char rel_img_address[], GLuint texture[], const int texture_num, pngInfo *png_info_p)
{
  glutInit(glut_argc_p, argv);
  // glut初期化

  glutInitWindowPosition(100, 100);
  // ウィンドウ位置設定.
  glutInitWindowSize(disp_width, disp_height);
  // ウィンドウサイズ設定.
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA | GLUT_DOUBLE);
  // ウィンドウ初期化.ダブルバッファモード.

  glutCreateWindow("display robot's position");
  // ウィンドウを生成.

  glClearColor(1.0, 1.0, 1.0, 1.0);
  // 初期化時の背景色を白に設定.

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // 混合処理(アルファチャンネル有効時の処理)の設定.

  glGenTextures(texture_num, texture);
  // テクスチャオブジェクトの生成.

  texture[0] = 0;
  texture[1] = 0;
  // テクスチャオブジェクトの初期化.

  std::string real_img_address;
  // pngファイルのパスを格納する
  const char *home_address =  getenv("HOME");
  // HOMEアドレスのパスを格納する.

  if (NULL != home_address)
  {
    real_img_address = home_address;
    real_img_address += rel_img_address;
    texture[0] = pngBind (real_img_address.c_str(), PNG_NOMIPMAP, PNG_ALPHA, png_info_p, GL_CLAMP, GL_LINEAR, GL_LINEAR);
    // png画像の取得.

    if (texture[0] == 0)
    {
      // pngデータを取得できなかった際.
      std::cout << "fail to get PNG image." << std::endl;
    }
  }
  else
  {
    // HOMEディレクトリがわからなかった時.
    std::cout << "fail to get home address." << std::endl;
    std::cout << "can't open background image." << std::endl;
  }


  glutDisplayFunc(display);
  // ディスプレイコールバック関数を設定.
  glutReshapeFunc(resize);
  // リサイズ時のコールバック関数を設定.
}

static void SubCallback(const detect_cercle::MBinput& msg)
{
  // MBからのデータのコールバック関数.

  var::alpha_count_margin = 0;
  // 連続でMBからの情報が受け取れないという状況はなくなった.

  if (true == var::now_data && var::field_color == msg.color && var::robot_x == msg.x && var::robot_y == msg.y && var::robot_theta == msg.theta && 0 == var::alpha_count )
  {
    // 変更する必要がないときは何もしない.とくに再描画させない.
    return;
  }

  var::field_color = msg.color;
  var::robot_x = msg.x;
  var::robot_y = msg.y;
  var::robot_theta = msg.theta;
  // MBからの情報.

  var::now_data = true;
  // データが書き込まれたことを示す.

  var::alpha_count--;
  if (var::alpha_count < 0)
  {
    var::alpha_count = 0;
  }
  // 透明度を下げる方向に変更.ただ透明度はなめらかに変えるためにいきなり0にはしない.アンダーフローしないように監視.
  glutPostRedisplay();
  // 再描画の必要性を主張.
}

static void display(void) {
    // ディスプレイコールバック関数

    glClear(GL_COLOR_BUFFER_BIT);
    // 色をクリア.指定した色で初期化する.

    glLoadIdentity();
    // 投影の変換行列を単位行列に初期化.

    glEnable(GL_TEXTURE_2D);
    glEnable(GL_ALPHA_TEST);
    glEnable(GL_BLEND);
    // テクスチャ/アルファチャンネル/混合処理を有効化.

    write_bg(var::texture[0]);
    // 背景の描画

    float tri_alpha = 1 - var::alpha_count * param::alpha_coe;
    // 三角形(自機の位置表示)の透明度

    if (tri_alpha > 1)
    {
      tri_alpha = 1;
    }
    else if (tri_alpha < param::alpha_min)
    {
      tri_alpha = param::alpha_min;
    }
    // 透明度が1より大きくなったり,指定された最小値より小さくなったりしないようにしてる.

    if(true == var::now_data){
      // 自己位置情報を受け取ったことがあるとき.

      glEnable(GL_POLYGON_SMOOTH);
      // ポリゴンのアンチエイリアスを有効化

      glHint(GL_POLYGON_SMOOTH_HINT,GL_NICEST);
      // 画質優先.

      write_triangle(param::field_width, param::field_height, var::robot_x * 1000, var::robot_y * 1000, var::robot_theta, param::disp_tri_len, tri_alpha, var::texture[1]);
      // 三角形の描画

      glDisable(GL_POLYGON_SMOOTH);
      // ポリゴンのアンチエイリアス無効化.

      write_position(param::field_width, param::field_height, param::field_string_x, param::field_string_y, param::disp_width, param::disp_height, var::robot_x, var::robot_y, var::robot_theta);
      // 位置情報の書き込み.
    }

    glDisable(GL_BLEND);
    glDisable(GL_ALPHA_TEST);
    glDisable(GL_TEXTURE_2D);
    // テクスチャ/アルファチャンネル/混合処理を無効化.

    glutSwapBuffers();
    // 新しい画面に入れ替え.
}

static void write_bg(GLuint texture)
{
  // 背景の描画.

  glPushMatrix();
  // 変換行列の退避.

  glBindTexture(GL_TEXTURE_2D, texture);
  // テクスチャの割当.
  glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  // テクスチャを下地の色と合成.

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
  // ポリゴン生成.

  glPopMatrix();
  // 変換行列を元に戻す.
}

static void write_triangle(const int field_width, const int field_height, const float x, const float y, const float theta, const float len, const float alpha, GLuint texture)
{
  // 三角形の描画.
  /*
    field_width/field_height:フィールドの縦/横の長さ.
    x/y:三角形を描画したい場所.実フィールドにおける位置.
    theta:姿勢角[rad].
    len:三角形の１辺の長さ(ピクセル数).
    alpha:三角形の透明度.
    texture:テクスチャ.
  */

  glPushMatrix();
  // 変換行列の退避.

  glBindTexture(GL_TEXTURE_2D, texture);
  // テクスチャの割当.

  glTranslated(-1, -1, 0);
  // 左下に移動.
  glTranslated(x / field_width * 2, y / field_height * 2, 0);
  // 指定された位置に移動.

  glRotated(theta / M_PI * 180, 0.0, 0.0, 1.0);
  // 指定されただけ回転.

  glBegin(GL_TRIANGLES);
    glColor4f(0, 0.7, 0, alpha);
    glVertex2f( len / sqrt(3)       , 0  );
    glVertex2f(-len / (2 * sqrt(3)) , len / 2);
    glVertex2f(-len / (2 * sqrt(3)) , -len / 2);
  glEnd();
  //三角形の描画.

  glPopMatrix();
  // 変換行列を元に戻す.
}

static void resize(int width, int height)
{
  // リサイズコールバック関数.

  glutReshapeWindow(param::disp_width, param::disp_height);
  // 一定値に保つ.
}

static void write_position(const int field_width, const int field_height, const int field_string_x, const int field_string_y, const int disp_width, const int disp_height, const float x_val, const float y_val, const float t_val)
{
  // 自己位置を表示.

  constexpr int string_point = 15;
  // 文字のポイント.

  glPushMatrix();
  // 変換行列の退避.

  glColor3f(0, 0, 0);
  // 文字の色を黒に.

  const float disp_string_x = (float)field_string_x / field_width * 2 - 1, disp_string_y = (float)field_string_y / field_height * 2 - 1;
  // ディスプレイ上での文字の位置.

  const float string_height_width = (float)string_point / disp_height * 3;
  // 各文字の縦方向の猶予.

  constexpr void* font = GLUT_BITMAP_9_BY_15;
  // フォントの割当.

  std::stringstream x_string;
  x_string << "x:" << std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(6) << std::right << x_val << "[m]";
  glRasterPos3f(disp_string_x, disp_string_y + string_height_width, 0);
  glutBitmapString(font,(unsigned char *)(x_string.str().c_str()));
  // xの描画.

  std::stringstream y_string;
  y_string << "y:" << std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(6) << std::right << y_val << "[m]";
  glRasterPos3f(disp_string_x, disp_string_y, 0);
  glutBitmapString(font,(unsigned char *)(y_string.str().c_str()));
  // yの描画.

  std::stringstream t_string;
  t_string << "t:" << std::showpos << std::fixed << std::setprecision(1) << std::setfill(' ') << std::setw(6) << std::right << t_val / M_PI *180 << "[deg]";
  glRasterPos3f(disp_string_x, disp_string_y - string_height_width, 0);
  glutBitmapString(font,(unsigned char *)(t_string.str().c_str()));
  // tの描画.

  glPopMatrix();
  // 変換行列を元に戻す.
}
