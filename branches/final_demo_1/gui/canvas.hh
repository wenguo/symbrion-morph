#ifndef CANVAS_HH
#define CANVAS_HH

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Overlay_Window.H>
#include <FL/Fl_Check_Button.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/Fl_Input_Choice.H>

#include <list>

#include "imagebox.hh"

struct UserInput
{
    Fl_Value_Slider *sliders[9];
    Fl_Input_Choice *channel_selection;
    Fl_Input_Choice *client_selection;
};

class Canvas : public Fl_Overlay_Window
{
    public:
        Canvas(int w, int h, UserInput * input); 
        void draw_overlay();
        int handle(int event);
        ~Canvas(){};

        iDisplay *GetNextiDisplay();
        bool AttachToiDisplay(ImageBox* imgbox);


        int startx;
        int starty;

        UserInput *user_input;

        static unsigned int timestamp;
        std::list<iDisplay*> displays;
        std::list<ImageBox*> imageboxes;

        static void TimerCallback( Canvas* canvas );
        bool dirty;

        void Snapshot(tm * timeinfo);
        bool Capture(tm *timeinfo);

        bool capturing;
        char logfolder[256];
};


#endif
