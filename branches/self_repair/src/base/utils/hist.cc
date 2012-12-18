#include "hist.hh"

Hist::Hist(uint8_t s )
{
        sum = 0;
        sum2=0;
        size = s;
        valPointer = 0;
        valueHist = new int32_t[size];
        memset(valueHist, 0, sizeof(int32_t)*size);
}

Hist::~Hist()
{
        if(valueHist)
                delete []valueHist;
}

void Hist::Reset()
{
        sum = 0;
        sum2=0;
        valPointer = 0;
        memset(valueHist, 0, sizeof(int32_t)*size);
}

void Hist::Resize(uint8_t s)
{
        if(valueHist)
                delete []valueHist;

        sum = 0;
        sum2=0;
        size = s;
        valPointer = 0;
        valueHist = new int32_t[size];
        memset(valueHist, 0, sizeof(int32_t)*size);

}

void Hist::Push(int32_t value)
{
        sum = sum - valueHist[valPointer] + value;
        valueHist[valPointer] = value;
        valPointer++;
        if (valPointer >= size)
                valPointer = 0;
}

//used for robot_in_range_detected
void Hist::Push2(uint8_t value)
{
        uint64_t temp_sum=0;
        for(int i=0;i<8;i++)
        {
                temp_sum = (sum2 >> (i*8)) & 0xFF;
                temp_sum = temp_sum - (((valueHist[valPointer] & (1<<i)) ==0 ) ? 0 : 1) + (((value & (1<<i))==0) ? 0 : 1);
                sum2 &= ~(uint64_t(0xFF) << (i*8));
                sum2 |= (temp_sum) << (i*8);//MAX 255 
        }
        valueHist[valPointer] = value;
        valPointer++;
        if (valPointer >= size)
                valPointer = 0;
}

void Hist::Print()
{
        if(valueHist==NULL)
                printf("empty hist buffer!\n");
        else
        {
                printf("Hist: ");
                for(int i=0;i<size;i++)
                {
                        printf("%d\t", valueHist[i]);
                }
                printf("\n");
        }
}

void Hist::Print2()
{
        if(valueHist==NULL)
                printf("empty hist buffer!\n");
        else
        {
                printf("Hist: ");
                for(int i=0;i<size;i++)
                {
                        printf("%#x\t", valueHist[i]);
                }
                printf("\n");
                printf("Sum2: ");
                for(int i=0;i<8;i++)
                {
                        printf("%#x\t", (unsigned int)Sum(i));
                }
                printf("\n");
                printf("Avg: ");
                for(int i=0;i<8;i++)
                {
                        printf("%#x\t", Avg(i));
                }
                printf("\n");
        }
}
