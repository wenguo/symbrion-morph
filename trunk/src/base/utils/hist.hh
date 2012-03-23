#ifndef HIST_HH
#define HIST_HH
#include <stdint.h>
#include <string.h>
#include <stdio.h>

class Hist
{
    public:
        Hist(uint8_t s = 8);
        ~Hist();

        void Reset();
        void Resize(uint8_t s);

        void Push(int32_t value);
        //used for robot_in_range_detected
        void Push2(int32_t value);

        inline int32_t Avg() {return sum / size;}

        inline int32_t Avg(int i){return (sum2>>(4*i) & 0xF)/size;}

        inline int64_t Sum(int i){return sum2>>(4*i) & 0xF;     }

        inline int64_t Sum(){return sum;}

        void Print();

        void Print2();
    private:
        int64_t sum;
        uint64_t sum2;
        int32_t *valueHist;
        uint8_t valPointer;
        uint8_t size;

} ;

#endif
