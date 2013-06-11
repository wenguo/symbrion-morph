// 
//
// Author: Wenguo Liu
// Date: 11/2011
//
//
#ifndef CANVAS_HH
#define CANVAS_HH
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Window.H>
#include <FL/fl_draw.H>
#include <FL/gl.h>
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

#include "organism.hh"

typedef void *(*fptr)(void *);

class Camera
{
    public:
        Camera();
        ~Camera();

        float _pitch;
        float _yaw;
        float _x;
        float _y;
        float _z;
};



class Canvas : public Fl_Gl_Window
{
    public:
        Canvas(int x, int y, int width, int height, const char *L);
        ~Canvas();
        static void TimerCallback( Canvas* canvas );

        bool dirty(){return _dirty;}
        void dirty(bool flag){ _dirty = flag;}
        bool step;
        std::vector<Organism*> ogList;
        pthread_mutex_t mutex;

    private:
        void InitGL();
        virtual void draw();
        virtual void resize(int x,int y,int width,int height);
        virtual int handle( int event );

    private:
        Camera camera;
        float startx;
        float starty;

        bool _dirty;

};

#endif
