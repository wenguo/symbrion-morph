// 
//
// Author: Wenguo Liu
// Date: 11/2011
//
//
#include <FL/fl_draw.H>
#include <FL/gl.h>
#include <stdio.h>
#include "organism.hh"

unsigned int OrganismNode::Geom::body[ROBOTTYPE_COUNT];

float bodycolor[ROBOTTYPE_COUNT][4] =
{
    {1.0f,0.0f,0.0f, 0.5f},
    {1.0f,0.0f,0.0f, 0.5f}, //ROBOT_KIT
    {0.0f,1.0f,0.0f, 0.5f}, //ROBOT_SCOUT
    {0.0f,0.0f,1.0f, 0.5f}  //ROBOT_AW
};

const double gap = 0.1*unit;

void OrganismNode::Geom::Draw(bool draw_name)
{
    glPushMatrix();
    glTranslated(px, py, 0);
    glRotated(pa, 0.0f, 0.0f, 1.0f);
    glCallList(OrganismNode::Geom::body[parent->type]);

    if(draw_name)
    {
        //draw name
        char str[64];
        sprintf(str, "%d", parent->id);
        glColor4f(0.7f,0.0f,0.7f, 0.7f);
        glRasterPos3d(0, 0, 1);
        //gl_draw(str);
    }

    //draw connection
    glColor4f(bodycolor[parent->type][0], bodycolor[parent->type][1], bodycolor[parent->type][2], bodycolor[parent->type][3]);
    double w = geom_size[parent->type][0] - gap;
    double h = geom_size[parent->type][1] - gap;
    double x1 = 0 ;
    double x2 = gap;
    double y1 = 0;
    double y2 = w/4;
    for(int i=0;i<SIDE_COUNT;i++)
    {
        if(parent->connection[i]!=NULL)
        {
            float alpha = -90 * i;
            double w1 = fabs(h * sin(DTOR(alpha)) + w * cos(DTOR(alpha)));  
            double h1 = fabs(h * cos(DTOR(alpha)) + w * sin(DTOR(alpha)));
            glPushMatrix();
            glTranslated(0.5 * w1 * cos(DTOR(alpha)), 0.5 * h1 * sin(DTOR(alpha)), 0);
            glRotated(alpha, 0.0f,0.0f,1.0f);
            glBegin( GL_QUADS);
            glVertex3d(x1, y1, 0 );
            glVertex3d(x2, y1, 0 );
            glVertex3d(x2, y2, 0 );
            glVertex3d(x1, y2, 0 );
            glEnd();
            glPopMatrix();
        }
    }

    glPopMatrix();
}


void OrganismNode::Geom::BuildList()
{
    static bool initialised = false;

    //only need to generate once
    if(initialised)
        return;

    initialised = true;

    body[0] = glGenLists(ROBOTTYPE_COUNT);
    body[ROBOT_KIT] = body[0] + 1;
    double w = geom_size[ROBOT_KIT][0] - gap;
    double h = geom_size[ROBOT_KIT][1] - gap;
    double x1 = -w/2;
    double x2 = w/2;
    double y1 = -h/2;
    double y2 = h/2;
    glNewList(body[ROBOT_KIT], GL_COMPILE);
    glColor4f(bodycolor[ROBOT_KIT][0], bodycolor[ROBOT_KIT][1], bodycolor[ROBOT_KIT][2], bodycolor[ROBOT_KIT][3]);
    glBegin( GL_QUADS);
    glVertex3d(x1, y1, 0 );
    glVertex3d(x2, y1, 0 );
    glVertex3d(x2, y2, 0 );
    glVertex3d(x1, y2, 0 );
    glEnd();
    glColor4f(0.f,0.f,0.f, 1.f);
    glBegin( GL_LINE_LOOP);
    glVertex3d(x1, y1, 1 );
    glVertex3d(x2, y1, 1 );
    glVertex3d(x2, y2, 1 );
    glVertex3d(x1, y2, 1 );
    glEnd();

    glColor4f(1.0f,1.0f,1.0f, 0.8f);
    glBegin(GL_TRIANGLES);
    glVertex3d(w/2, 0, 1 );
    glVertex3d(w/4, h/4, 1 );
    glVertex3d(w/4, -h/4, 1 );
    glEnd();

    //hinge
    glColor4f(0.f, 0.f, 0.f, 0.6f);
    glBegin(GL_LINES);
    glVertex3d(0, 0.42 * h, 1);
    glVertex3d(0, -0.42 * h, 1);
    glEnd();
    glBegin(GL_LINES);
    glVertex3d(0.1*w, 0.42 * h, 1);
    glVertex3d(-0.1*w, 0.42 * h, 1);
    glEnd();
    glBegin(GL_LINES);
    glVertex3d(0.1*w, -0.42 * h, 1);
    glVertex3d(-0.1*w, -0.42 * h, 1);
    glEnd();
    glEndList();

    body[ROBOT_SCOUT] = body[0] + 2;
    w =  geom_size[ROBOT_SCOUT][0] - gap;
    h =  geom_size[ROBOT_SCOUT][1] - gap;
    x1 = -w/2;
    x2 = w/2;
    y1 = -h/2;
    y2 = h/2;
    glNewList(body[ROBOT_SCOUT], GL_COMPILE);
    glColor4f(bodycolor[ROBOT_SCOUT][0], bodycolor[ROBOT_SCOUT][1], bodycolor[ROBOT_SCOUT][2], bodycolor[ROBOT_SCOUT][3]);
    glBegin( GL_QUADS);
    glVertex3d(x1, y1, 0 );
    glVertex3d(x2, y1, 0 );
    glVertex3d(x2, y2, 0 );
    glVertex3d(x1, y2, 0 );
    glEnd();
    glColor4f(0.f,0.f,0.f, 1.f);
    glBegin( GL_LINE_LOOP);
    glVertex3d(x1, y1, 1 );
    glVertex3d(x2, y1, 1 );
    glVertex3d(x2, y2, 1 );
    glVertex3d(x1, y2, 1 );
    glEnd();

    glColor4f(1.0f,1.0f,1.0f, 0.8f);
    glBegin(GL_TRIANGLES);
    glVertex3d(w/2, 0, 1 );
    glVertex3d(w/4, h/4, 1 );
    glVertex3d(w/4, -h/4, 1 );
    glEnd();

    //hinge
    glColor4f(0.f, 0.f, 0.f, 0.6f);
    glBegin(GL_LINES);
    glVertex3d(0, 0.1 * h, 1);
    glVertex3d(0, -0.1 * h, 1);
    glEnd();
    glBegin(GL_LINES);
    glVertex3d(0.03*w, 0.1 * h, 1);
    glVertex3d(-0.03*w, 0.1 * h, 1);
    glEnd();
    glBegin(GL_LINES);
    glVertex3d(0.03*w, -0.1 * h, 1);
    glVertex3d(-0.03*w, -0.1 * h, 1);
    glEnd();

    glBegin(GL_LINES);
    glVertex3d(0, 0, 1);
    glVertex3d(0.4*w, 0, 1);
    glEnd();
    glBegin(GL_LINES);
    glVertex3d(0.4*w, -0.5 * h, 1);
    glVertex3d(0.4*w, 0.5 * h, 1);
    glEnd();
    glEndList();

    body[ROBOT_AW] = body[0] + 3;
    w = geom_size[ROBOT_AW][0] - gap;
    h = geom_size[ROBOT_AW][1] - gap;
    x1 = -w/2;
    x2 = w/2;
    y1 = -h/2;
    y2 = h/2;
    glNewList(body[ROBOT_AW], GL_COMPILE);
    glColor4f(bodycolor[ROBOT_AW][0], bodycolor[ROBOT_AW][1], bodycolor[ROBOT_AW][2], bodycolor[ROBOT_AW][3]);
    glBegin( GL_QUADS);
    glVertex3d(x1, y1, 0 );
    glVertex3d(x2, y1, 0 );
    glVertex3d(x2, y2, 0 );
    glVertex3d(x1, y2, 0 );
    glEnd();
    glColor4f(0.f,0.f,0.f, 1.f);
    glBegin( GL_LINE_LOOP);
    glVertex3d(x1, y1, 1 );
    glVertex3d(x2, y1, 1 );
    glVertex3d(x2, y2, 1 );
    glVertex3d(x1, y2, 1 );
    glEnd();

    glColor4f(1.0f,1.0f,1.0f, 0.8f);
    glBegin(GL_TRIANGLES);
    glVertex3d(w/2, 0, 1 );
    glVertex3d(w/4, w/4, 1 );
    glVertex3d(w/4, -w/4, 1 );
    glEnd();

    //hinge
    glColor4f(0.f, 0.f, 0.f, 0.6f);

    glBegin(GL_LINES);
    glVertex3d(-0.42*w, 0, 1);
    glVertex3d(0.42*w, 0, 1);
    glEnd();
    glBegin(GL_LINES);
    glVertex3d(0.42*w, -0.1*w, 1);
    glVertex3d(0.42*w, 0.1*w, 1);
    glEnd();
    glBegin(GL_LINES);
    glVertex3d(-0.42*w, -0.1*w, 1);
    glVertex3d(-0.42*w, 0.1*w, 1);
    glEnd();
    glEndList();


}

void Organism::Draw(double offset_x, double offset_y, double z, double rot, bool draw_name)
{
    if(NodeList().empty())
        return;

    glPushMatrix();
    glTranslated(offset_x, offset_y, z);
    glRotated(rot, 0.0f, 0.0f, 1.0f);

    for(int i=0;i<NodeList().size();i++)
    {
        if(NodeList()[i])
            NodeList()[i]->GetGeom().Draw(draw_name);
    }
    glPopMatrix();
}

