// 
//
// Author: Wenguo Liu
// Date: 11/2011
//
//
#include <stdio.h>
#include <string.h>
#include "canvas_gl.hh"

extern std::vector<fptr> testFunctionList;
bool draw_id=false;

Canvas::Canvas (int x, int y, int width, int height, const char *L):
    Fl_Gl_Window(x, y, width, height, L)
{
    Fl::scheme("plastic");
    end();
    show();
    _dirty = false;
    step = false;
    pthread_mutex_init(&mutex, NULL);
    Fl::add_timeout(0.1,(Fl_Timeout_Handler)Canvas::TimerCallback,this);
}

Canvas::~Canvas()
{
}

void Canvas::draw()
{
    if (!valid())
    {
        valid(1);
        InitGL();
    }
    // Clear screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //draw organsim
    pthread_mutex_lock(&mutex);
    if(!ogList.empty())
    {
        for(int i=0;i<ogList.size();i++)
        {
            if(ogList[i])
                ogList[i]->Draw(camera._x + 10 * unit * (i % 8), -camera._y + 14 * unit * (i/8), -camera._z, 0, draw_id);
        }
    }
    pthread_mutex_unlock(&mutex);
}

void Canvas::InitGL()
{
    OrganismNode::Geom::BuildList();

    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    // glOrtho( -w(), w(), -h(), h(), -1, 1000 ); 
    gluPerspective(45.0f,(GLfloat)w()/(GLfloat)h(),0.1f,100 * unit);    
    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity();

    // set gl state that won't change every redraw
    glEnable(GL_TEXTURE_2D);                        // Enable Texture Mapping ( NEW )
    glShadeModel(GL_SMOOTH);    
    glClearColor ( 0.9, 0.9, 0.9, 1.0);
    glDisable( GL_LIGHTING );
    glClearDepth(1.0f);                         // Depth Buffer Setup
    glEnable(GL_DEPTH_TEST);                        // Enables Depth Testing
    glDepthFunc(GL_LEQUAL);    
    glCullFace( GL_BACK );
    glEnable (GL_CULL_FACE);
    glEnable( GL_BLEND );
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glEnable( GL_LINE_SMOOTH );
    glHint( GL_LINE_SMOOTH_HINT, GL_FASTEST );
    glDepthMask( GL_TRUE );
    glEnable( GL_TEXTURE_2D );
    glEnableClientState( GL_VERTEX_ARRAY );
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

    // install a font
    gl_font( FL_HELVETICA, 12 );  
}

void Canvas::resize(int x, int y, int width, int height)
{
    Fl_Gl_Window::resize(x,y,width,height);
    glLoadIdentity();
    glViewport(0,0,width,height);
    glOrtho(-width,width,-height,height,-1,1);
    redraw();
}

int Canvas::handle(int event)
{
    static int count = 0;
    int id = 9999;
    switch (event)
    {
        case FL_MOUSEWHEEL:
            camera._z += 10 * Fl::event_dy();
            redraw();
            break;
        case FL_MOVE:
            startx = Fl::event_x();
            starty = Fl::event_y();
            return 1;
        case FL_PUSH:
            return  1;
        case FL_DRAG:
            {
                int dx = Fl::event_x() - startx;
                int dy = Fl::event_y() - starty;
                camera._x += dx;
                camera._y += dy;
                startx = Fl::event_x();
                starty = Fl::event_y();
            }
            redraw();
            return 1;
        case FL_RELEASE:
            return 1;
        case FL_FOCUS:
        case FL_UNFOCUS:
            return 1;            
        case FL_KEYBOARD:
            switch( Fl::event_key() )
            {
                case FL_Left:
                    camera._x +=10;
                    break;
                case FL_Right:
                    camera._x -=10;
                    break;
                case FL_Down:
                    camera._y +=50;
                    break;
                case FL_Up:
                    camera._y -=50;
                    break;
                case FL_Page_Up:
                    camera._z -=10;
                    break;
                case FL_Page_Down:
                    camera._z +=10;
                    break;
                case FL_Shift_L:
                    draw_id = !draw_id;
                    break;
                case FL_Tab:
                    {
                        if(testFunctionList.size() >0)
                            id = count++ % testFunctionList.size();
                    }
                    break;
            }

            //check if a-g is pressed
            if(strcmp(Fl::event_text(), "a") ==0)
                id = 0;
            else if(strcmp(Fl::event_text(), "b") ==0)
                id = 1;
            else if(strcmp(Fl::event_text(), "c") ==0)
                id = 2;
            else if(strcmp(Fl::event_text(), "d") ==0)
                id = 3;
            else if(strcmp(Fl::event_text(), "e") ==0)
                id = 4;
            else if(strcmp(Fl::event_text(), "f") ==0)
                id = 5;
            else if(strcmp(Fl::event_text(), "g") ==0)
                id = 6;
            else if(strcmp(Fl::event_text(), "s") ==0)
                step = true;

            if(testFunctionList.size() >id)
            {
                pthread_t thread;
                //clean drawing list
                pthread_mutex_lock(&mutex);
                while(!ogList.empty())
                {
                    Organism *og = ogList.back();
                    delete og;
                    ogList.pop_back();
                }
                pthread_mutex_unlock(&mutex);
                //call function

                pthread_create(&thread, NULL, testFunctionList[id], this);
            }

            return 1;      
        default:
            return Fl_Gl_Window::handle(event);
    }

}

void Canvas::TimerCallback( Canvas* c )
{
    if( c->_dirty )
    {
        c->redraw();
        c->_dirty = false;
    }

    Fl::repeat_timeout( 0.1,(Fl_Timeout_Handler)Canvas::TimerCallback,c);
}

Camera::Camera()
{
    _pitch = 0;
    _yaw = 0;
    _x = -30 * unit;
    _y = 15 * unit;
    _z = 70 * unit;
}

Camera::~Camera()
{

}
