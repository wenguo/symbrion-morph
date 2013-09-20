#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Overlay_Window.H>
#include <FL/fl_draw.H>
#include <FL/Fl_Image.H>

#include "imagebox.hh"
#include "canvas.hh"

using namespace std;

unsigned int Canvas::timestamp = 0;
const Fl_Color colors[MAX_COLORS_TRACKED]={FL_RED, FL_GREEN};


Canvas::Canvas(int w, int h, UserInput * input): 
    Fl_Overlay_Window(w,h,"Canvas")
{
    int startx = 10;
    int starty = 10;
    int gapx = 1;
    int gapy = 1;
    displays.push_back(new iDisplay(this,startx, starty));
//  displays.push_back(new iDisplay(this,startx+gapx+640, starty));
//  displays.push_back(new iDisplay(this,startx, starty+gapy+480));
 //   displays.push_back(new iDisplay(this,startx+gapx+640, starty+gapy+480));

    capturing = false;
    dirty = true;

    user_input = input;

};

void Canvas::TimerCallback(Canvas * canvas)
{
   // if(canvas->dirty)
    {
        canvas->redraw_overlay();
        canvas->dirty = false;
    }

    Canvas::timestamp++;

//    printf("step: %d\n", Canvas::timestamp);

    Fl::repeat_timeout(0.1, (Fl_Timeout_Handler)Canvas::TimerCallback, canvas);

}


void Canvas::draw_overlay()
{
    fl_color(FL_BLACK);
    fl_rect(8,8,645,485); // image display region
    fl_rect(660,10,220,570); // threshold setting region
    fl_rect(10,500,640,80); //button region
    char str[64];
   // printf("...");
    fflush(stdout);

    std::list<iDisplay*>::iterator it;
    for(it=displays.begin();it!=displays.end();it++)
    {
        fl_rect((*it)->startx, (*it)->starty, (*it)->width, (*it)->height);
        ImageBox *imagebox = (*it)->imagebox;
        if(imagebox && imagebox->Connected())
        {
            fl_draw_image(imagebox->img_rgb_buffer, (*it)->startx, (*it)->starty, imagebox->img_width, imagebox->img_height);
            fl_color(FL_RED);
            fl_draw(imagebox->name,(*it)->startx + 20, (*it)->starty+20);
            char timestring[64];
            sprintf(timestring, "%d.%d", (int)imagebox->frame.hdr.timestamp.tv_sec, (int)imagebox->frame.hdr.timestamp.tv_usec);
            fl_draw(timestring,(*it)->startx + 250, (*it)->starty+20);
            fl_color(FL_BLACK);

            int orig_x = (*it)->startx + imagebox->img_width/2;
            int orig_y = (*it)->starty + imagebox->img_height/2;


            for(int i=0;i<MAX_COLORS_TRACKED;i++)
            {
                for(int j=0;j<imagebox->blob_info[i].num_blobs;j++)
                {
                    blob_rect * blob = &(imagebox->blob_info[i].blobs[j]);
                    fl_color(colors[i]);
                    if(imagebox->flipped_image)
                    {
                        fl_rect(-blob->offset.x -  blob->size.x / 2 + orig_x,
                                 blob->offset.y - blob->size.y / 2 + orig_y,
                                blob->size.x,
                                blob->size.y);
                        fl_color(FL_WHITE);

                        fl_line(-blob->offset.x -10 + orig_x, blob->offset.y + orig_y,
                                -blob->offset.x + 10 + orig_x, blob->offset.y + orig_y);
                        fl_line(-blob->offset.x + orig_x, blob->offset.y - 10 + orig_y,
                                -blob->offset.x+ orig_x, blob->offset.y + 10 + orig_y);
                        sprintf(str, "%d", blob->id);
                        fl_color(FL_YELLOW);
                        fl_draw(str,-blob->offset.x + orig_x, blob->offset.y + orig_y);
                        fl_color(FL_BLACK);

                    }
                    else
                    {
                        fl_rect(blob->offset.x -  blob->size.x / 2 + orig_x,
                                - blob->offset.y - blob->size.y / 2 + orig_y,
                                blob->size.x,
                                blob->size.y);
                        fl_color(FL_WHITE);

                        fl_line(blob->offset.x -10 + orig_x, -blob->offset.y + orig_y,
                                blob->offset.x + 10 + orig_x, -blob->offset.y + orig_y);
                        fl_line(blob->offset.x + orig_x, -blob->offset.y - 10 + orig_y,
                                blob->offset.x+ orig_x, -blob->offset.y + 10 + orig_y);
                        sprintf(str, "%d", blob->id);
                        fl_color(FL_YELLOW);
                        fl_draw(str,blob->offset.x + orig_x, -blob->offset.y + orig_y);
                        fl_color(FL_BLACK);
                    }
                }
            }
        }

        if((*it)->clicked ==1)
        {
            fl_color(FL_WHITE);
            fl_line_style(FL_DOT);
            fl_rect((*it)->region_x1,(*it)->region_y1,(*it)->region_x2-(*it)->region_x1,(*it)->region_y2-(*it)->region_y1);
            fl_color(FL_BLACK);
            fl_line_style(FL_SOLID);
            /*
               int index;
               int r,g,b,y,u,v;
               index = ((region_y1 - starty )* width + region_x1-startx )* 3 ;
               r = img_rgb_buffer[index];
               g = img_rgb_buffer[index+1];
               b = img_rgb_buffer[index+2];
               RGB2YUV(r,g,b,y,u,v);
               sprintf(str, "(%d %d)- (%d %d %d) (%d %d %d)", region_x1-startx, region_y1-starty, r,g,b,y,u,v);
               fl_draw(str,100, 20);
               index = ((region_y2 - starty )* width + region_x2-startx )* 3 ;
               r = img_rgb_buffer[index];
               g = img_rgb_buffer[index+1];
               b = img_rgb_buffer[index+2];
               RGB2YUV(r,g,b,y,u,v);
               sprintf(str, "(%d %d)- (%d %d %d) (%d %d %d)", region_x2-startx, region_y2-starty, r,g,b,y,u,v);
               fl_draw(str,100, 40);*/
        }

        if((*it)->clicked > 0)
        {
            fl_color(FL_RED);
            fl_line_style(FL_SOLID, 5);
            fl_rect((*it)->startx,(*it)->starty,IMAGE_WIDTH, IMAGE_HEIGHT);
            fl_color(FL_BLACK);
            fl_line_style(FL_SOLID, 1);
        }


    }

}

//event handler
int Canvas::handle(int event)
{
    Fl_Window::handle(event);
    switch(event)
    {
        case FL_MOVE:
            {
                std::list<iDisplay*>::iterator it;
                for(it=displays.begin();it!=displays.end();it++)
                {
                    if((*it)->clicked > 0)
                    {
                        if((*it)->InRegion(Fl::event_x(), Fl::event_y()))
                        {
                            (*it)->region_x2 = Fl::event_x();
                            (*it)->region_y2 = Fl::event_y();
                        }
                    }
                    else
                    {
                        (*it)->region_x2 = 0;//Fl::event_x();
                        (*it)->region_y2 = 0;//Fl::event_y();
                    }


                    if((*it)->clicked==1)
                        redraw();
                }
            }
            break;
        case FL_PUSH:
            break;
        case FL_RELEASE:
            {
                std::list<iDisplay*>::iterator it;
                for(it=displays.begin();it!=displays.end();it++)
                {
                    if((*it)->InRegion(Fl::event_x(), Fl::event_y()))
                        (*it)->clicked++;
                    else
                        (*it)->clicked = 0;

                    if((*it)->clicked > 2)
                        (*it)->clicked = 0;
                    else if((*it)->clicked == 1)
                    {
                        (*it)->region_x1 = Fl::event_x();
                        (*it)->region_y1 = Fl::event_y();
                    }
                    else if((*it)->clicked ==2)
                    {
                        printf("%d %d %d %d\n", (*it)->region_x1 -  (*it)->startx,  (*it)->region_y1 -  (*it)->starty, (*it)-> region_x2 -  (*it)->startx,  (*it)->region_y2 -  (*it)->starty);
                        string time_string;
                        ::time_t time_now = time(NULL);
                        struct tm * timeinfo;
                        timeinfo = localtime (&time_now);

                        char filename[64];
                        sprintf(filename, "log-%02d%02d-%02d:%02d:%02d.csv", timeinfo->tm_mon+1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);


                        int r,g,b,y,u,v;
                        int w =  (*it)->region_x2 - (*it)->region_x1 +1;
                        int h = (*it)->region_y2 - (*it)->region_y1 + 1;
                        if(w > 0 && h>0)
                        {
                            int index=0;
                            unsigned char *temp_buffer = new unsigned char[ w * h * 3];
                            unsigned char * y_buffer = new unsigned char[w*h];
                            unsigned char * u_buffer = new unsigned char[w*h];
                            unsigned char * v_buffer = new unsigned char[w*h];
                            unsigned char * r_buffer = new unsigned char[w*h];
                            unsigned char * g_buffer = new unsigned char[w*h];
                            unsigned char * b_buffer = new unsigned char[w*h];
                            memset(temp_buffer, 0, w * h * 3);
                            printf("w: %d\th: %d\n", w, h);
                            FILE *fp = fopen(filename, "w");
                            for(int i= (*it)->region_y1 -  (*it)->starty;i<= (*it)->region_y2- (*it)->starty;i++)
                            {
                                for(int j= (*it)->region_x1 -  (*it)->startx;j<= (*it)->region_x2- (*it)->startx;j++)
                                {
                                    ImageBox * imgbox = (*it)->imagebox;

                                    if(imgbox)
                                    {
                                        int width = imgbox->img_width;
                                        r = imgbox->img_rgb_buffer[(i* width + j )*3];
                                        g = imgbox->img_rgb_buffer[(i* width + j )*3 + 1];
                                        b = imgbox->img_rgb_buffer[(i* width + j )*3 + 2];
                                    }
                                    RGB2YUV(r,g,b,y,u,v);
                                    //save y,u,v separately
                                    y_buffer[index/3] = y;
                                    u_buffer[index/3] = u;
                                    v_buffer[index/3] = v;
                                    r_buffer[index/3] = r;
                                    g_buffer[index/3] = g;
                                    b_buffer[index/3] = b;
                                    //save rgb for drawing
                                    temp_buffer[index++] = r;
                                    temp_buffer[index++] = g;
                                    temp_buffer[index++] = b;
                                    fprintf(fp, "%d\t%d\t%d\t%d\t%d\t%d\n", r, g, b, y, u, v);
                                }
                            }
                            fclose(fp);

                            unsigned char *y_result = Quartile(y_buffer, w*h);
                            unsigned char *u_result = Quartile(u_buffer, w*h);
                            unsigned char *v_result = Quartile(v_buffer, w*h);
                            unsigned char *r_result = Quartile(r_buffer, w*h);
                            unsigned char *g_result = Quartile(g_buffer, w*h);
                            unsigned char *b_result = Quartile(b_buffer, w*h);

                            //set slider values
                            (*it)->canvas->user_input->sliders[RED]->value(r_result[0]);
                            (*it)->canvas->user_input->sliders[GREEN]->value(g_result[0]);
                            (*it)->canvas->user_input->sliders[BLUE]->value(b_result[0]);
                            (*it)->canvas->user_input->sliders[YMIN]->value(y_result[3] < y_result[5] ? y_result[5] : y_result[3]);
                            (*it)->canvas->user_input->sliders[YMAX]->value(y_result[4] < y_result[6] ? y_result[4] : y_result[6]);
                            (*it)->canvas->user_input->sliders[UMIN]->value(u_result[3] < u_result[5] ? u_result[5] : u_result[3]);
                            (*it)->canvas->user_input->sliders[UMAX]->value(u_result[4] < u_result[6] ? u_result[4] : u_result[6]);
                            (*it)->canvas->user_input->sliders[VMIN]->value(v_result[3] < v_result[5] ? v_result[5] : v_result[3]);
                            (*it)->canvas->user_input->sliders[VMAX]->value(v_result[4] < v_result[6] ? v_result[4] : v_result[6]);

                            sprintf(filename, "log-%02d%02d-%02d:%02d:%02d.ppm", timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
                            ppm16Write(filename,temp_buffer, w, h);
                            delete []y_result;
                            delete []u_result;
                            delete []v_result;
                            delete []r_result;
                            delete []g_result;
                            delete []b_result;
                            delete []temp_buffer;

                        }


                    }

                    printf("display %d clicked : %d\n",(*it)->id,(*it)->clicked);

                }
            }
            /*

               if(clicked>2)
               {
               clicked = 0;
               }

               else if (clicked == 1)
               {
               region_x1 = Fl::event_x();
               region_y1 = Fl::event_y();
               }
            //else if (clicked == 2)
            {
            printf("%d %d %d %d\n", region_x1 - startx, region_y1 - starty, region_x2 - startx, region_y2 - starty);
            string time_string;
            ::time_t time_now = time(NULL);
            struct tm * timeinfo;
            timeinfo = localtime (&time_now);

            char filename[64];
            sprintf(filename, "log-%02d%02d-%02d:%02d:%02d.csv", timeinfo->tm_mon+1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
            */
            /*
               FILE *fp = fopen(filename, "w");

               int r,g,b,y,u,v;
               int w = region_x2 - region_x1 +1;
               int h = region_y2 - region_y1 + 1;
               if(w > 0 && h>0)
               {
               int index=0;
               unsigned char *temp_buffer = new unsigned char[ w * h * 3];
               unsigned char * y_buffer = new unsigned char[w*h];
               unsigned char * u_buffer = new unsigned char[w*h];
               unsigned char * v_buffer = new unsigned char[w*h];
               unsigned char * r_buffer = new unsigned char[w*h];
               unsigned char * g_buffer = new unsigned char[w*h];
               unsigned char * b_buffer = new unsigned char[w*h];
               memset(temp_buffer, 0, w * h * 3);
               for(int i=region_y1 - starty;i<=region_y2-starty;i++)
               {
               for(int j=region_x1 - startx;j<=region_x2-startx;j++)
               {
            //        r = img_rgb_buffer[(i* width + j )*3];
            //        g = img_rgb_buffer[(i* width + j )*3 + 1];
            //        b = img_rgb_buffer[(i* width + j )*3 + 2];
            RGB2YUV(r,g,b,y,u,v);
            //save y,u,v separately
            y_buffer[index/3] = y;
            u_buffer[index/3] = u;
            v_buffer[index/3] = v;
            r_buffer[index/3] = r;
            g_buffer[index/3] = g;
            b_buffer[index/3] = b;
            //save rgb for drawing
            temp_buffer[index++] = r;
            temp_buffer[index++] = g;
            temp_buffer[index++] = b;
            fprintf(fp, "%d\t%d\t%d\t%d\t%d\t%d\n", r, g, b, y, u, v);
            }
            }
            fclose(fp);
            unsigned char *y_result = Quartile(y_buffer, w*h);
            unsigned char *u_result = Quartile(u_buffer, w*h);
            unsigned char *v_result = Quartile(v_buffer, w*h);
            unsigned char *r_result = Quartile(r_buffer, w*h);
            unsigned char *g_result = Quartile(g_buffer, w*h);
            unsigned char *b_result = Quartile(b_buffer, w*h);

            //set slider values
            sliders[RED]->value(r_result[0]);
            sliders[GREEN]->value(g_result[0]);
            sliders[BLUE]->value(b_result[0]);
            sliders[YMIN]->value(y_result[3] < y_result[5] ? y_result[5] : y_result[3]);
            sliders[YMAX]->value(y_result[4] < y_result[6] ? y_result[4] : y_result[6]);
            sliders[UMIN]->value(u_result[3] < u_result[5] ? u_result[5] : u_result[3]);
            sliders[UMAX]->value(u_result[4] < u_result[6] ? u_result[4] : u_result[6]);
            sliders[VMIN]->value(v_result[3] < v_result[5] ? v_result[5] : v_result[3]);
            sliders[VMAX]->value(v_result[4] < v_result[6] ? v_result[4] : v_result[6]);

            sprintf(filename, "log-%02d%02d-%02d:%02d:%02d.ppm", timeinfo->tm_mon, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
            ppm16Write(filename,temp_buffer, w, h);
            delete []y_result;
            delete []u_result;
            delete []v_result;
            delete []r_result;
            delete []g_result;
            delete []b_result;
            delete []temp_buffer;
            }
            */

            //}
            break;
        default:
            break;
    }

    return 1;

}


void Canvas::Snapshot(tm * timeinfo)
{
    std::list<iDisplay*>::iterator it;
    for(it = displays.begin();it!= displays.end();it++)
    {
        if((*it)->imagebox)
            (*it)->imagebox->Snapshot(timestamp);
    }
}

bool Canvas::Capture(tm * timeinfo)
{
    if(capturing)
        return false;

    capturing = true;
    std::list<iDisplay*>::iterator it;

    sprintf(logfolder, "./logs/capturing-%02d%02d-%02d%02d%02d", timeinfo->tm_mon+1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    mkdir(logfolder, S_IRWXU|S_IRGRP|S_IXGRP|S_IROTH|S_IXOTH);

    for(it = displays.begin();it!= displays.end();it++)
    {
        if((*it)->imagebox)
            (*it)->imagebox->InitCapturing(logfolder);
    }

    return true;

}

iDisplay *Canvas::GetNextiDisplay()
{
    std::list<iDisplay*>::iterator it;
    for(it = displays.begin();it!= displays.end();it++)
    {
        if((*it)->imagebox == NULL)
            return *it;
    }

    return NULL;
}

bool Canvas::AttachToiDisplay(ImageBox* imgbox)
{
    if(imgbox)
        imageboxes.push_back(imgbox);

    iDisplay * display = GetNextiDisplay();

    if(!display)
    {
        return false;
    }
    else
        return display->SetImageBox(imgbox);

}
