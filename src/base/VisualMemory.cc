#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <algorithm>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <string.h>
#include "VisualMemory.hh"

enum blob_threshold_id {BODY_GREETING=0, BODY_OBSERVING, LEDS_GREETING, LEDS_OBSERVING, MAX_NUM_BLOB_THRESHOLDS};
object_threshold g_objects_threshold[MAX_NUM_BLOB_THRESHOLDS] = 
    {
        {{50,20},{180,75}},
        {{20,10},{60,20}},
        {{0, 0},{320, 240}},
        {{0, 0},{20, 20}}
    }; 

Blob_info::Blob_info()
{
    //	printf("Construct Blob_info\n");
    offset.x = 0;
    offset.y = 0;
    size.x = 0;
    size.y = 0;
    timestamp = 0;
    sys_timestamp = -1;
    channel = -1;
}

Blob_info::Blob_info(int x, int y, int w, int h, uint32_t ts, int32_t sys_ts, uint8_t ch)
{
    //	printf("Construct Blob_info\n");
    offset.x = x;
    offset.y = y;
    size.x = w;
    size.y = h;
    timestamp = ts;
    sys_timestamp = sys_ts;
    channel = ch;
}

Blob_info::Blob_info(const Blob_info&b)
{
    //	printf("Construct Blob_info\n");
    offset.x = b.offset.x;
    offset.y = b.offset.y;
    size.x = b.size.x;
    size.y = b.size.y;
    timestamp = b.timestamp;
    sys_timestamp = b.sys_timestamp;
    channel = b.channel;
}

Blob_info& Blob_info::operator=(const Blob_info & rhs)
{
    if(this!=&rhs)
    {
        offset.x = rhs.offset.x;
        offset.y = rhs.offset.y;
        size.x = rhs.size.x;
        size.y = rhs.size.y;
        timestamp = rhs.timestamp;
        sys_timestamp = rhs.sys_timestamp;
        channel = rhs.channel;
    }

    return *this;
}

bool Blob_info::Related(const Blob_info& b)
{
    return abs(b.offset.x - offset.x) <= b.size.x / 2 && abs(b.offset.y - offset.y) <=b.size.y /2;
}

std::ostream& operator<<(std::ostream& out, const Blob_info& b)
{
    out << b.offset.x <<" "
        << b.offset.y <<" "
        << b.size.x	  <<" "
        << b.size.y	  <<" "
        << b.timestamp <<" "
        << b.sys_timestamp<< " "
        << (int)b.channel;
    return out;
}

Blob_info::~Blob_info()
{
    //	printf("Destroy Blob_info\n");
}

float Blob_info::Distance(const Blob_info&a, const Blob_info&b)
{
    return	(a.offset.x - b.offset.x)*(a.offset.x-b.offset.x) + (a.offset.y - b.offset.y)*(a.offset.y-b.offset.y);
}

float Blob_info::Distance(const Blob_info&b)
{
    return Distance(*this, b);
}

void Blob_info::Print()
{
    if(sys_timestamp!=-1)
        printf("offset(%d\t%d)\tsize(%d\t%d)\ttimestamp(%d\t%d)\n", offset.x, offset.y, size.x,size.y, timestamp, sys_timestamp);
}

int Object::index = 0;

Object::Object()
{
   id = index;
    size.x = 0;
    size.y = 0;
    //	printf("Construct Object %d\n", id);
    index++;

    for(int i=0;i<MAX_COLORS_TRACKED;i++)
    {
        footprint[i].clear();
    }
}
Object::Object(uint32_t ts)
{
    id = index;
    size.x = 0;
    size.y = 0;
    //	printf("Construct Object %d\n", id);
    index++;

    for(int i=0;i<MAX_COLORS_TRACKED;i++)
    {
        footprint[i].clear();
    }

    last_update_timestamp = ts;
}

Object::~Object()
{
    //	printf("Destroy Object %d\n",id);
    CleanMemory();
}

void Object::CleanMemory()
{
    for(int i=0; i< MAX_COLORS_TRACKED;i++)
    {
        footprint[i].clear();
        //memory of blobs will be free in blob list
        /*
        while(!footprint[i].empty())
        {
            Blob_info * tmp= footprint[i].front();
            delete tmp;
            footprint[i].pop_front();
        }*/
    }
}
void Object::Log(const char* prefix)
{
    char filename[64];
    for(int i=0;i<MAX_COLORS_TRACKED;i++)
    {
        sprintf(filename, "%s%d_ch%d.log", prefix, id, i);
        FILE *fp = fopen(filename,"w"); 
        if(fp)
        {
            std::list<Blob_info*>::iterator it;
            for(it = footprint[i].begin();it!=footprint[i].end();it++)
            {
                Blob_info * b = *it;
                if(b)
                    fprintf(fp, "%d %d %d %d %d %d %d\n", b->offset.x, b->offset.y, b->size.x, b->size.y, b->timestamp, b->sys_timestamp, b->channel);
            }
            fclose(fp);
        }
        else
        {
            printf("failed to create file %s", filename);
        }
    }
}

bool Object::operator<(const Object& b) const
{
    printf("compare size (%d %d)",size.x, b.size.x);
    return size.x < b.size.x;
}

bool Object::Compare(const Object* a, const Object* b)
{
    return a->size.x < b->size.x;
}

void Object::Print()
{
    printf("Object %d's information:\n", id);
    for(int i=0;i<MAX_COLORS_TRACKED;i++)
    {
        printf("-----channel %d --------\n", i);
        std::list<Blob_info*>::iterator it_blob;
        if(!footprint[i].empty())
        {
            for(it_blob = footprint[i].begin(); it_blob != footprint[i].end(); it_blob++)
            {
                if((*it_blob)!=NULL)
                    (*it_blob)->Print();	
            }
        }
    }

}
bool Object::LoadRecordsFromFile(const char * prefix, int id)
{
    //remove all saved blob info
    CleanMemory();


    bool ret = false;
    char filename[64];
    int n,x,y,w,h,ts,sys_ts, ch;
    for(int i=0;i<MAX_COLORS_TRACKED;i++)
    {
        FILE *logfile;
        char buffer[256];
        sprintf(filename, "%s%d_ch%d.log", prefix, id, i);
        logfile=fopen(filename, "r");
        n = 0;
        if(logfile)
        {
            while(fgets(buffer, 256, logfile))
            {
                n = sscanf(buffer, "%d %d %d %d %d %d %d",&x,&y,&w,&h,&ts,&sys_ts, &ch);
                if(n==7)
                {
                    footprint[i].push_back(new Blob_info(x,y,w,h,ts,sys_ts,i));
                    ret = true;
                }

            }
            fclose(logfile);
        }

    }

    return ret;
}

Blob_info Object::GetAveragedBlobInfo(uint32_t ts, uint8_t channel)
{
    std::list<Blob_info*>::iterator it;
    int sum_x=0;
    int sum_y=0;
    int sum_w=0;
    int sum_h=0;
    int sys_ts=0;
    int count = 0;

    bool matched=false;
    for(it = footprint[channel].begin();it!=footprint[channel].end();it++)
    {
        if((*it) && (*it)->timestamp == ts)
        {
            matched = true;
            count++;
            sum_x += (*it)->offset.x;
            sum_y += (*it)->offset.y;
            sum_w += (*it)->size.x;
            sum_h += (*it)->size.y;
            sys_ts = (*it)->sys_timestamp;
        }
        else if(matched)
            break;
    }
    if(count!=0)
        return Blob_info( sum_x/count, sum_y/count, sum_w/count, sum_h/count, ts, sys_ts, channel);
    else
        return Blob_info();
}

std::list<Blob_info*> Object::GetBlobs(uint32_t ts, uint8_t channel)
{
    std::list<Blob_info*> ret;
    std::list<Blob_info*>::iterator it;
    bool matched=false;
    for(it = footprint[channel].begin();it!=footprint[channel].end();it++)
    {
        if((*it) && (*it)->timestamp == ts)
        {
            matched = true;
            ret.push_back(*it);
        }
        else if(matched)
            break;
    }

    return ret;

}

Blob_info Object::GetLatestAveragedBlobInfo(uint32_t ts, uint8_t channel)
{
    std::list<Blob_info*>::reverse_iterator rit;
    int sum_x=0;
    int sum_y=0;
    int sum_w=0;
    int sum_h=0;
    int sys_ts=0;
    int count = 0;

    bool matched=false;
    for(rit = footprint[channel].rbegin();rit!=footprint[channel].rend();rit++)
    {
        if((*rit) && (*rit)->timestamp == ts)
        {
            matched = true;
            count++;
            sum_x += (*rit)->offset.x;
            sum_y += (*rit)->offset.y;
            sum_w += (*rit)->size.x;
            sum_h += (*rit)->size.y;
            sys_ts = (*rit)->sys_timestamp;
        }
        else if(matched)
            break;
    }
    if(count!=0)
        return Blob_info( sum_x/count, sum_y/count, sum_w/count, sum_h/count, ts, sys_ts, channel);
    else
        return Blob_info();
}



VisualMemory::VisualMemory(int w, int h)
{
    //	printf("Construct VisualMemory\n");

    img_width = w;
    img_height = h;

    x_class = new uint8_t[w];
    y_class = new uint8_t[h];

    SetObjectsThreshold(g_objects_threshold,MAX_NUM_BLOB_THRESHOLDS);


}

VisualMemory::~VisualMemory()
{
  //  printf("Destroy VisualMemory\n");
    CleanMemory();

    if(x_class!=NULL)
        delete []x_class;
    if(y_class!=NULL)
        delete []y_class;
}

void VisualMemory::CleanMemory()
{
    while(!blobs.empty())
    {
        Blob_info *blob = blobs.front();
        delete blob;
        blobs.pop_front();
    }
    while(!objects.empty())
    {
        Object * tmp = objects.front();
        delete tmp;
        objects.pop_front();
    }
    while(!old_objects.empty())
    {
        Object * tmp = old_objects.front();
        delete tmp;
        old_objects.pop_front();
    }



}

void VisualMemory::SetObjectsThreshold(object_threshold *th, uint8_t size)
{
    if(size>MAX_NUM_BLOB_THRESHOLDS)
    {
        printf("max threshold size is %d", MAX_NUM_BLOB_THRESHOLDS);
        return;
    }

    memset(x_class, 0, img_width);
    memset(y_class, 0, img_height);

    for(int i=0;i<size;i++)
    {
        uint8_t k = 1<<i;
        
        int xl = max((int)th[i].min.x,0);
        int xh = min((int)th[i].max.x,img_width);
        int yl = max((int)th[i].min.y,0);
        int yh = min((int)th[i].max.y,img_height);

        for(int j=xl; j<=xh; j++) 
            x_class[j] |= k;
        for(int j=yl; j<=yh; j++) 
            y_class[j] |= k;
    }

/*
    for(int i=0;i<img_width;i++)
    {
        printf("%d: %d ", i,x_class[i]);
        if(i<img_height)
            printf("%d\n", y_class[i]);
        else
            printf("\n");
    }
    */
}

Blob_info VisualMemory::GetLatestAveragedBlobInfo(uint32_t ts, uint8_t channel)
{
    if(channel >= MAX_COLORS_TRACKED)
        return Blob_info();

    std::list<Blob_info*>::reverse_iterator rit;
    int sum_x=0;
    int sum_y=0;
    int sum_w=0;
    int sum_h=0;
    int sys_ts = 0;
    int count = 0;

    for(rit = blobs.rbegin();rit!=blobs.rend();rit++)
    {
        if((*rit) && (*rit)->timestamp == ts)
        {
            if((*rit)->channel == channel)
            {
                count++;
                sum_x += (*rit)->offset.x;
                sum_y += (*rit)->offset.y;
                sum_w += (*rit)->size.x;
                sum_h += (*rit)->size.y;
                sys_ts = (*rit)->sys_timestamp;
            }
        }
        else
            break;
    }

    if(count!=0)
        return Blob_info( sum_x/count, sum_y/count, sum_w/count, sum_h/count, ts, sys_ts, channel);
    else
        return Blob_info();
}

Blob_info VisualMemory::GetLatestAveragedBlobInfo(uint32_t ts, int id, uint8_t channel)
{
    if(channel >= MAX_COLORS_TRACKED)
        return Blob_info();

    std::list<Blob_info*>::reverse_iterator rit;
    int sum_x=0;
    int sum_y=0;
    int sum_w=0;
    int sum_h=0;
    int sys_ts=0;
    int count = 0;
    std::list<Object*>::iterator it;
    for(it=objects.begin();it!=objects.end();it++)
    {
        Object * o = *it;
        if(o && o->id ==id)
        {
            for(rit = o->footprint[channel].rbegin();rit!=o->footprint[channel].rend();rit++)
            {
                if((*rit) && (*rit)->timestamp == ts)
                {
                    count++;
                    sum_x += (*rit)->offset.x;
                    sum_y += (*rit)->offset.y;
                    sum_w += (*rit)->size.x;
                    sum_h += (*rit)->size.y;
                    sys_ts = (*rit)->sys_timestamp;
                }
                else
                    break;
            }
            break;
        }
    }

    if(count!=0)
        return Blob_info( sum_x/count, sum_y/count, sum_w/count, sum_h/count, ts, sys_ts, channel);
    else
        return Blob_info();
}
bool VisualMemory::AddTrackingData(const Blob_info& b)
{
    Blob_info  *blob = new Blob_info(b);
    uint8_t channel = blob->channel;

    //save all blobs for post analysis
    blobs.push_back(blob);

    //remove objects if it's last footprint is too old
    std::list<Object*> tmp_objects;
    int objects_count = objects.size();
    for(int i=0;i<objects_count;i++)
    {
        Object* obj=objects.front();
        objects.pop_front();
        if(b.timestamp - obj->last_update_timestamp > 10)
            old_objects.push_back(obj);
        else
            objects.push_back(obj);
    }

    //	printf("-- current VM\n");
    //	Print();
    //	printf("==== new data entry fron channel %d :", channel);
    //blob->Print();

    //seperate the data to different object
    //footprint?
    if(channel >= MAX_COLORS_TRACKED)
    {
        printf("error, can only track %d colors, but now the value is %d\n", MAX_COLORS_TRACKED, channel);
        return false;
    }


    std::list<Object*>::iterator it;
    Object * target_object = NULL;
    float dist=9999;

    if(channel == BODY)
    {
        if(x_class[blob->size.x] & y_class[blob->size.y] & (1<<BODY_GREETING))
        {
            for(it = objects.begin();it!=objects.end();it++)
            {
                if(!(*it)->footprint[BODY].empty())
                {
                    Blob_info * tmp_blob = (*it)->footprint[BODY].back();
                    if((tmp_blob->sys_timestamp != blob->sys_timestamp) 
                            && blob->Distance(*tmp_blob) < dist)
                    {
                        target_object = *it;
                        dist = blob->Distance(*tmp_blob);
                    }
                }
            }

            //not found? creat a new one
        //    blob->Print();
            if(!target_object)
            {
                Object *tmp = new Object(blob->timestamp);
                tmp->footprint[channel].push_back(blob);
                objects.push_back(tmp);
                tmp->size = blob->size;
                tmp->last_update_timestamp = blob->timestamp;
        //        printf("---> new object %d\n", tmp->id);
            }
            else
            {
                target_object->footprint[channel].push_back(blob);
                target_object->size = blob->size;
                target_object->last_update_timestamp = blob->timestamp;
         //       printf("---> exisiting object %d\n", target_object->id);
            }
        }
        else
        {
         //   printf("blob is not in range, ignore it\n");
        //    blob->Print();
            return false;
        }

    }
    else if(channel == LEDS)
    {
        for(it = objects.begin();it!=objects.end();it++)
        {
            if(!(*it)->footprint[BODY].empty())
            {
                Blob_info * tmp_blob = (*it)->footprint[BODY].back();
                if(tmp_blob->sys_timestamp == blob->sys_timestamp
                    && blob->Related(*tmp_blob))
                {
                    target_object = *it;
                }
                
            }
            
        }

        if(!target_object)
            ;//printf("Associated Body is not found\n");
        else
            target_object->footprint[channel].push_back(blob);

    }
    else
    {	
        printf("not implement yet!");
    }

    objects.sort(Object::Compare);

    return true;

}
bool VisualMemory::AddTrackingData(int x1, int y1, int x2, int y2, uint32_t timestamp, uint32_t sys_timestamp, uint8_t channel)
{
    Blob_info  blob((img_width - (x1 + x2)) / 2,
            (img_height - (y1 + y2 )) / 2,
            x2 - x1,
            y2 - y1,
            timestamp,
            sys_timestamp,
            channel);

    return AddTrackingData(blob);

}

void VisualMemory::Print()
{
    std::list<Object *>::iterator it;
    for(it = objects.begin();it!=objects.end();it++)
    {
        if(*it)
            (*it)->Print();
    }
}

void VisualMemory::Log()
{
    //write blobs info to file
    char hostname[64];
    char blob_logfilename[256];
    char logfolder[256];
    time_t time_now = time(NULL);
    tm * timeinfo= localtime (&time_now);
    gethostname(hostname,64);

    sprintf(logfolder, "log-%02d%02d-%02d%02d%02d-%s", timeinfo->tm_mon+1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, hostname);

    mkdir(logfolder, S_IRWXU|S_IRGRP|S_IXGRP|S_IROTH|S_IXOTH);
    sprintf(blob_logfilename, "%s/blob_info_raw.log",logfolder);
    if(!blobs.empty())
    {
        FILE *fp = fopen(blob_logfilename,"w");
        if(fp)
        {
            std::list<Blob_info*>::iterator it;
            for(it = blobs.begin();it!=blobs.end();it++)
            {
                Blob_info * b = *it;
                if(b)
                    fprintf(fp, "%d %d %d %d %d %d %d\n", b->offset.x, b->offset.y, b->size.x, b->size.y, b->timestamp, b->sys_timestamp, b->channel);
            }

            fclose(fp);
        }
        else
        {
            printf("failed to create file %s", blob_logfilename);
        }
    }

    //write objects info to file
    std::list<Object*>::iterator it;
    char prefix[256];
    sprintf(prefix, "%s/object", logfolder);
    for(it = objects.begin();it!=objects.end();it++)
    {
        if(*it)
            (*it)->Log(prefix);
    }
    for(it = old_objects.begin();it!=old_objects.end();it++)
    {
        if(*it)
            (*it)->Log(prefix);
    }

}

bool VisualMemory::LoadFromFile(const char * filename)
{
    CleanMemory();
    Object::index=0;

    if(!filename)
        return false;

    FILE * fp = fopen(filename, "r");
    int n, x,y,w,h,ts,sys_ts,ch;

    if(fp)
    {
        char line[256];
        while(fgets(line, 256, fp))
        {
            n=sscanf(line, "%d %d %d %d %d %d %d", &x,&y,&w,&h,&ts,&sys_ts,&ch);
            if(n==7)
            {
                AddTrackingData(Blob_info(x,y,w,h,ts,sys_ts,ch));
            }
        }
        fclose(fp);
        return true;
    }
    else
    {
        printf("failed to open file\n");
        return false;
    }

}
