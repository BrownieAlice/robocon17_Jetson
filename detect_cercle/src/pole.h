#ifndef _POLE_CLASS_
#define _POLE_CLASS_

class pole
{
private:
 float define_posi[2];
 float esti_posi[2];
 bool esti_start;
 int num;
public:
 pole();
 pole(float x,float y,int n);
 void init_pole(float x,float y,int n);
 void set_esti_posi(float x,float y);
 void get_esti_posi(float *x,float *y);
};
#endif
