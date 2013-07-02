#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>    // std::sort

#include "global.hh"



typedef struct {
    std::string dir_name;
    std::vector<std::string> files;
} record_file_t;

typedef struct {
    int timestamp;
    int current_state;
    int last_state;
    int speed[3];
    int beacon[2];
    int recruitment_stage[4];

}record_t;

bool comp(record_file_t a, record_file_t b)
{
    return a.dir_name.compare(b.dir_name) > 0 ? false : true;
}
bool comp_string(std::string a, std::string b)
{
    return a.compare(b) > 0 ? false : true;
}

int GetRecord(std::vector<record_file_t> &record_files, const char * parent_dir);
int ProcessRecord(std::string filename, std::string path);

int main(int argc, char ** agrv)
{
    std::vector<record_file_t> record_files;
    GetRecord(record_files, "..");
    std::sort(record_files.begin(),record_files.end(), comp);
    for (unsigned int i = 0;i < record_files.size();i++) 
    {
        int min = 9999;
        int max = -9999;
        int sum = 0;
        int count = 0;
       // std::cout << record_files[i].dir_name << std::endl;
        for (unsigned int j = 0;j < record_files[i].files.size();j++) 
        {
            //std::cout << record_files[i].files[j] << std::endl;
            int ret = ProcessRecord(record_files[i].files[j], record_files[i].dir_name);
            if(ret > max)
                max = ret;
            if(ret < min)
                min = ret;
            if(ret >0)
                count++;
            sum += ret;
        }
        printf("%s\t%f\t%d\t%d\n", record_files[i].dir_name.c_str(), sum * 1.0/count, min, max);
    }


    return 1;
}

int GetRecord(std::vector<record_file_t> &record_files, const char * parent_dir)
{
    DIR *dp, *sub_dp;
    struct dirent *dirp;
    struct dirent *sub_dirp;
    if((dp  = opendir(parent_dir)) == NULL) 
    {
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) 
    {
        std::string str(dirp->d_name);
        if(str.compare(0,1,"p") ==0 && str.length() ==3)
        {
            if((sub_dp  = opendir(std::string(std::string(parent_dir) + std::string("/") + std::string(dirp->d_name)).c_str())) != NULL) 
            {
                record_file_t rec;
                rec.dir_name = std::string(parent_dir) + std::string("/")+std::string(dirp->d_name)+std::string("/");
                while((sub_dirp = readdir(sub_dp)) != NULL)
                {
                    std::string str(sub_dirp->d_name);
                    if(str.compare(0,5,"Robot") ==0)
                        rec.files.push_back(std::string(sub_dirp->d_name));
                }
                std::sort(rec.files.begin(), rec.files.end(), comp_string);
                record_files.push_back(rec);
                closedir(sub_dp);
            }
        }
    }
    closedir(dp);
    return 0;
}

int ProcessRecord(std::string filename, std::string path)
{
    std::ifstream logfile;
    char c;
    logfile.open((path + std::string("/") +filename).c_str(), std::ios::in);
    int trials = 0;
    if(logfile.is_open())
    {
        record_t tmp;
        while(!logfile.eof())
        {
            char line[256];
            logfile.getline(line, 256);
            int tmp_state;
            tmp_state = tmp.current_state;

            sscanf(line, "%d\t%d\t%d\t(%d\t%d\t%d)\t%d\t%d\t[%d\t%d\t%d\t%d]",
                    &tmp.timestamp,
                    &tmp.current_state,
                    &tmp.last_state,
                    &tmp.speed[0],
                    &tmp.speed[1],
                    &tmp.speed[2],
                    &tmp.beacon[0],
                    &tmp.beacon[1],
                    &tmp.recruitment_stage[0],
                    &tmp.recruitment_stage[1],
                    &tmp.recruitment_stage[2],
                    &tmp.recruitment_stage[3]);
            
            if(tmp_state != tmp.current_state)
            {
                if(tmp.current_state == DOCKING)
                    trials++;
                //std::cout<<tmp.timestamp<<":\t"<<state_names[tmp.current_state]<<"\t-\t"<<state_names[tmp.last_state]<<std::endl;
            }

        }
        //std::cout<<filename<<"\t"<<trials<<std::endl;
        logfile.close();
    }

    return trials;
}
