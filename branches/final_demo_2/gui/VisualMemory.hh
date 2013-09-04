#ifndef VISUALMEMORY_H
#define VISUALMEMORY_H

#include <stdint.h>
#include <list>
#include <iostream>
#include <fstream>

#include "../src/base/vision_def.h"

using namespace std;


//threahold for a valid object
struct object_threshold
{
    vect_2 min;
    vect_2 max;
};

class Blob_info
{
    public:
        Blob_info();
        Blob_info(int x, int y, int w, int h, uint32_t ts, int32_t sys_ts, uint8_t ch);
        Blob_info(const Blob_info&b);
        ~Blob_info();
        void Reset()
        {
            offset.x = 0;
            offset.y = 0;
            size.x = 0;
            size.y = 0;
            channel = 0;
        }
        float Distance(const Blob_info& a, const Blob_info & b);
        float Distance(const Blob_info & b);
        void Print();

        Blob_info& operator=(const Blob_info & b);
        friend std::ostream& operator<<(std::ostream& out, const Blob_info&b);
        bool Related(const Blob_info & b); //check if it belongs to the the same object, b -- body

        vect_2 offset;
        vect_2 size;
        uint8_t channel;
        uint32_t timestamp;
        int32_t sys_timestamp;
};

class Object
{
    public:
        Object(uint32_t ts);
        Object();
        ~Object();

        void Log(const char * prefix="object");

        void Print();

        void CleanMemory();

        bool LoadRecordsFromFile(const char * prefix="object", int id=0);

        Blob_info GetAveragedBlobInfo(uint32_t ts, uint8_t channel=0); //forward searching, slow version
        Blob_info GetLatestAveragedBlobInfo(uint32_t ts, uint8_t channel=0); // fast searching, backward
        std::list<Blob_info*> GetBlobs(uint32_t ts, uint8_t channel=0);

        bool operator<(const Object& b) const;
        static bool Compare(const Object* a, const Object* b);

        static int index;
        int last_update_timestamp;
        int id;
        vect_2 size; //size of body footprint
        std::list<Blob_info*> footprint[MAX_COLORS_TRACKED];  //two channels, one for body, one for led signals
};


class VisualMemory
{
    public:
        VisualMemory(int w, int h);
        ~VisualMemory();

        bool LoadFromFile(const char * filename="blob_info_raw.log");
        bool AddTrackingData(int x1, int y1, int x2, int y2, uint32_t timestamp, uint32_t sys_timestamp, uint8_t channel = 0);
        bool AddTrackingData(const Blob_info& blob);
        void CleanMemory();

        Blob_info GetLatestAveragedBlobInfo(uint32_t ts, uint8_t channel=0);
        Blob_info GetLatestAveragedBlobInfo(uint32_t ts, int id, uint8_t channel=0);

        inline const std::list<Object*>&	Objects(){return objects;}
        inline const std::list<Object*>&	OldObjects(){return old_objects;}
        void SetObjectsThreshold(object_threshold *th, uint8_t size=2);

        void Print();
        void Log();
    private:
        std::list<Object*> objects;
        std::list<Object*> old_objects;
        std::list<Blob_info*> blobs;

        uint8_t *x_class;
        uint8_t *y_class; //lookup table for object blob threshold
        

        int img_width;
        int img_height;
};

#endif
