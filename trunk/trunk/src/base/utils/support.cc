/*
 * support.cc
 *
 *  Created on: 29 Apr 2010
 *      Author: wliu
 */

#include <sstream>
#include "support.hh"
#include <locale>
#include <ctime>
#include <stdexcept>
#include <math.h>
#include <string.h>
#include <cstdlib>

void textcolor(int attr, int fg, int bg)
{
        char command[13];

        /* Command is the control command to the terminal */
        sprintf(command, "%c[%d;%d;%dm", 0x1B, attr, fg + 30, bg + 40);
        printf("%s", command);
}

double simple_normal_deviate( double mean, double stddev )
{
    double x = 0.0;

    for ( int i = 0; i < 12; i++ )
        x += rand() / (RAND_MAX + 1.0);

    return ( stddev * (x - 6.0) + mean );
}

string convBase(unsigned char v, long base)
{
    string digits = "0123456789abcdef";
    string result;
    if ((base < 2) || (base > 16))
    {
        result = "Error: base out of range.";
    }
    else
    {
        do
        {
            result = digits[v % base] + result;
            v /= base;
        }
        while (v);
    }
    return result;
}

string datetime_to_string(const tm& time, const char* format)
{
    stringstream datetime;

    // retrieve the time_put facet installed in the stream
    const time_put<char>& writer =
        use_facet< time_put<char> >(datetime.getloc());

    int len = strlen(format);

    //formats the contents of the tm time into the output stream datetime
    if (writer.put(datetime, datetime, ' ',
                   &time, format, format + len).failed( ))
    {
        throw runtime_error("formatting date time failed!");
    }

    return datetime.str();
}

/*
 next_comb(int comb[], int k, int n)
 Generates the next combination of n elements as k after comb

 comb => the previous combination ( use (0, 1, 2, ..., k) for first)
 k => the size of the subsets to generate
 n => the size of the original set

 Returns: 1 if a valid combination was found
 0, otherwise
 */
int next_comb(int *comb, int k, int n)
{
    int i = k - 1;
    ++comb[i];
    while ((i >= 0) && (comb[i] >= n - k + 1 + i))
    {
        --i;
        ++comb[i];
    }

    if (comb[0] > n - k) /* Combination (n-k, n-k+1, ..., n) reached */
        return 0; /* No more combinations can be generated */

    /* comb now looks like (..., x, n, n, n, ..., n).
     Turn it into (..., x, x + 1, x + 2, ...) */
    for (i = i + 1; i < k; ++i)
        comb[i] = comb[i - 1] + 1;

    return 1;
}


bool  init_pos_matrix(vect2 center, vect2 size, vect2 * pos_data, int num)
{
    if(!pos_data || num < 0)
        return false;
    int col = ceil(sqrt(num));
    vect2 offset;
    if(col >1)
    {
        offset.x = size.x / (col - 1);
        offset.y = size.y / (col - 1);
    }
    else
    {
        offset.x = 0;
        offset.y = 0;
    }

    vect2 start;
    if(col%2)
    {
        start.x = center.x - ((col - 1)/2) *offset.x;
        start.y = center.y - ((col - 1)/2) *offset.y;
    }
    else
    {
        start.x = center.x - ((col - 1)/2 + 0.5) *offset.x;
        start.y = center.y - ((col - 1)/2 + 0.5) *offset.y;
    }

    bool flag = false;
    for(int i=0;i<col;i++)
    {
    for(int j=0;j<col;j++)
    {
        pos_data[i * col + j].x = start.x + j * offset.x; 
        pos_data[i * col + j].y = start.y + i * offset.y; 
        if(i*col + j>=num)
        {
            flag = true;
            break;
        }
    }
    if(flag)
        break;
    }
    return true;
}
