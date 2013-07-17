#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string>

#include <FL/Fl_Button.H>


#include "utils.hh"
#include "../src/base/vision_def.h"
#include "canvas.hh"
#include "imagebox.hh"

#include "bytequeue.h"
#include "ethlolmsg.h"

using namespace std;

const int width  = 320;
const int height = 240;

int channel_index = 0;
color_info channel_info;

void Start(Fl_Widget* o, void*);
void Stop(Fl_Widget* o, void*);
void Snapshot(Fl_Widget* o, void*);
void Capture(Fl_Widget* o, void*);
void Stream(Fl_Widget* o, void*);
void SetChannelInfo(Fl_Widget *, void *);
void GetChannelInfo(Fl_Widget *, void *);

void Monitoring(const ELolMessage* msg, void * connection, void *user_ptr);

const char *slider_names[9] = {"Red", "Green", "Blue", "Y_min", "U_min", "V_min", "Y_max", "U_max", "V_max"};

Canvas * canvas;

UserInput* Input;

#define STR_CHANNEL_BODY "Channel Body"
#define STR_CHANNEL_LED "Channel LEDs"


//buffer for communication via port 5000

//--------------------------------------------
int main(int argc, char **argv) {
    

    srand(time(NULL));

    Input = new UserInput;
    canvas =  new Canvas(900, 600, Input);

    Fl_Button but_start( 70, 520, 80, 40,"Start" );
    Fl_Button but_stop( 220, 520, 80, 40,"Stop" );
    Fl_Button but_snapshot( 370, 520, 80, 40,"Snapshot" );
    Fl_Button but_flip( 520, 520, 80, 40,"Capture" );
    Fl_Check_Button but_stream(700, 20, 200, 20, "Streaming Image");
    but_start.callback(Start);
    but_stop.callback(Stop);
    but_snapshot.callback(Snapshot);
    but_stream.callback(Stream);
    but_flip.callback(Capture);


    Input->client_selection = new Fl_Input_Choice(700, 40, 130, 25);

    //for channel selection and setting
    Input->channel_selection = new Fl_Input_Choice(700, 80, 130, 25);
    Input->channel_selection->add(STR_CHANNEL_BODY);
    Input->channel_selection->add(STR_CHANNEL_LED);
    Input->channel_selection->value(0);

    Fl_Button but_get( 700, 520, 50, 40,"Get" );
    Fl_Button but_set( 800, 520, 50, 40,"Set" );
    but_get.callback(GetChannelInfo);
    but_set.callback(SetChannelInfo);

    int pos_x = 700;
    int pos_y = 120;
    int size_x = 25;
    int size_y = 100;
    int offset_x = 50;
    int offset_y = size_y + 30;

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            Input->sliders[i*3+j] = new Fl_Value_Slider(pos_x + j * offset_x, pos_y + i * offset_y, size_x, size_y, slider_names[i*3+j]);
            Input->sliders[i*3+j]->type(FL_VERTICAL);
            Input->sliders[i*3+j]->range(0, 255);
            Input->sliders[i*3+j]->step(1);
        }
    }

    canvas->end();
    canvas->show();
    canvas->redraw_overlay();

    Fl::add_timeout(0.1, (Fl_Timeout_Handler)Canvas::TimerCallback, canvas);

    IPC::IPC monitoringIPC;
    monitoringIPC.SetCallback(Monitoring, NULL);
    monitoringIPC.Name("GUI");
    monitoringIPC.Start("localhost", PORT_SERVER, true);

    return Fl::run();
}

#define PIXEL_FORMAT V4L2_PIX_FMT_YUYV

void callback(void*input)
{
 //   Fl::repeat_timeout(0.1, callback);    
 //   Canvas::timestamp++;
    //canvas->redraw_overlay();
}

void Start(Fl_Widget* o, void* input)
{
 //   Fl::add_timeout(0.1, callback, input);
}

void Stop(Fl_Widget * o, void * input)
{
//    Fl::remove_timeout(callback);
}

void Snapshot(Fl_Widget * o, void *input)
{   
    string time_string;
    ::time_t time_now = time(NULL);
    struct tm * timeinfo;
    timeinfo = localtime (&time_now);

    canvas->Snapshot(timeinfo);


}

void Stream(Fl_Widget * o, void * input)
{
    Fl_Check_Button * btn = (Fl_Check_Button*) o;
    comm_status_t req_type = btn->value() ? REQ_IMAGE_FRAME : REQ_BLOB_INFO;
    std::list<iDisplay*>::iterator it;
    for(it = canvas->displays.begin();it!=canvas->displays.end();it++)
    {
        ImageBox *imagebox = (*it)->imagebox;
        if(imagebox && imagebox->connected)
            imagebox->SetREQType(req_type);
    }

    printf("stream clicked %d\n", btn->value());
}

void Capture(Fl_Widget * o, void * input)
{
    string time_string;
    ::time_t time_now = time(NULL);
    struct tm * timeinfo;
    timeinfo = localtime (&time_now);
    canvas->Capture(timeinfo);
}


void SetChannelInfo(Fl_Widget * o, void *input)
{
    printf("Set channel info clicked\n");
    std::list<iDisplay*>::iterator it;
    for(it = canvas->displays.begin(); it!= canvas->displays.end();it++)
    {
        if((*it)->clicked > 0)
        {
            ImageBox * imgbox = (*it)->imagebox;
            if(imgbox)
            {
                color_info channel_info;
                int index;
                if(strcmp(Input->channel_selection->value(), STR_CHANNEL_BODY)==0)
                    index = 0;
                else if(strcmp(Input->channel_selection->value(), STR_CHANNEL_LED)==0)
                    index = 1;

                channel_info.color.red = Input->sliders[RED]->value();
                channel_info.color.green = Input->sliders[GREEN]->value();
                channel_info.color.blue = Input->sliders[BLUE]->value();
                channel_info.y_low = Input->sliders[YMIN]->value();
                channel_info.y_high = Input->sliders[YMAX]->value();
                channel_info.u_low = Input->sliders[UMIN]->value();
                channel_info.u_high = Input->sliders[UMAX]->value();
                channel_info.v_low = Input->sliders[VMIN]->value();
                channel_info.v_high = Input->sliders[VMAX]->value();

                imgbox->SetChannelInfo(channel_info, index);
            }
            break;
        }
    }

}

void GetChannelInfo(Fl_Widget * o, void *input)
{
    printf("Get channel info clicked\n");
    std::list<iDisplay*>::iterator it;
    for(it = canvas->displays.begin(); it!= canvas->displays.end();it++)
    {
        if((*it)->clicked > 0)
        {
            ImageBox * imgbox = (*it)->imagebox;
            if(imgbox)
            {
                pthread_mutex_lock(&imgbox->mutex);
                if(strcmp(Input->channel_selection->value(), STR_CHANNEL_BODY)==0)
                    imgbox->channel_index = 0;
                else if(strcmp(Input->channel_selection->value(), STR_CHANNEL_LED)==0)
                    imgbox->channel_index = 1;
                pthread_mutex_unlock(&imgbox->mutex);

                imgbox->RequestInfo(REQ_CHANNEL_INFO);
            }
            break;
        }
    }

}

static int client_port = PORT_CLIENT_START;

void Monitoring(const ELolMessage* msg, void * connection, void *user_ptr)
{
    switch (msg->command)
    {
        case  REQ_SUBSCRIPTION:
            printf("received subscription request from: %s\n", msg->data);
            if(canvas->AttachToiDisplay(new ImageBox(client_port)))
            {
                ((IPC::Connection*)connection)->SendData(REQ_SUBSCRIPTION_ACK, (uint8_t*)&client_port, 4);
            }
            client_port++;
            break;
        case REQ_UNSUBSCRIPTION:
            break;
        default:
            break;
    }

}
