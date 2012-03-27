/*
 * support.hh
 *
 *  Created on: 29 Apr 2010
 *      Author: wliu
 */

#ifndef SUPPORT_HH_
#define SUPPORT_HH_

#include <string>
#include <stdio.h>


#define RESET           0
#define BRIGHT          1
#define DIM             2
#define UNDERLINE       3
#define BLINK           4
#define REVERSE         7
#define HIDDEN          8

#define SCR_BLACK               0
#define SCR_RED         1
#define SCR_GREEN               2
#define SCR_YELLOW              3
#define SCR_BLUE                4
#define SCR_MAGENTA             5
#define SCR_CYAN                6
#define SCR_WHITE               7
void textcolor(int attr, int fg, int bg);

using namespace std;

class vect2
{
    public:
        vect2():x(0),y(0){};
        vect2(float _x, float _y):x(_x),y(_y){};
        ~vect2(){};
    float x;
    float y;
};

#define sign(x) (( x > 0 ) - ( x < 0 ))

double simple_normal_deviate( double mean, double stddev );
string datetime_to_string(const tm& time, const char* format);
string convBase(unsigned char v, long base);
int next_comb(int *comb, int k, int n);
bool  init_pos_matrix(vect2 center, vect2 size, vect2 * pos_data, int num);

#define CPrintf(color, m) {textcolor(BRIGHT, color, SCR_BLACK); printf(""m"\n"); textcolor(RESET, SCR_WHITE, SCR_BLACK);}
#define CPrintf1(color, m, a) {textcolor(BRIGHT, color, SCR_BLACK); printf(""m"\n", a); textcolor(RESET, SCR_WHITE, SCR_BLACK);}
#define CPrintf2(color, m, a, b) {textcolor(BRIGHT, color, SCR_BLACK); printf(""m"\n", a, b); textcolor(RESET, SCR_WHITE, SCR_BLACK);}
#define CPrintf3(color, m, a, b, c) {textcolor(BRIGHT, color, SCR_BLACK); printf(""m"\n", a, b, c); textcolor(RESET, SCR_WHITE, SCR_BLACK);}
#define CPrintf4(color, m, a, b, c, d) {textcolor(BRIGHT, color, SCR_BLACK); printf(""m"\n", a, b, c, d); textcolor(RESET, SCR_WHITE, SCR_BLACK);}
#define CPrintf5(color, m, a, b, c, d, e) {textcolor(BRIGHT, color, SCR_BLACK); printf(""m"\n",a, b, c, d, e); textcolor(RESET, SCR_WHITE, SCR_BLACK);}

#endif /* SUPPORT_HH_ */
