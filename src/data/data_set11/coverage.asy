import graph;
import palette;
size(20cm,0cm);
string []files = {
"./RobotKIT_20130311111614.log",
"./RobotKIT_20130311111727.log",
"./RobotKIT_20130311111825.log",
"./RobotKIT_20130311111919.log",
"./RobotKIT_20130311112022.log",
"./RobotKIT_20130311112110.log",
"./RobotKIT_20130311112214.log",
"./RobotKIT_20130311112303.log",
"./RobotKIT_20130311112347.log",
"./RobotKIT_20130311112438.log",
"./RobotKIT_20130311112527.log",
"./RobotKIT_20130311112612.log",
"./RobotKIT_20130311112720.log",
"./RobotKIT_20130311112814.log",
"./RobotKIT_20130311112859.log",
"./RobotKIT_20130311112951.log",
"./RobotKIT_20130311113049.log",
"./RobotKIT_20130311113140.log",
"./RobotKIT_20130311113242.log",
"./RobotKIT_20130311113328.log",
"./RobotKIT_20130311113416.log",
"./RobotKIT_20130311113509.log",
"./RobotKIT_20130311113745.log",
"./RobotKIT_20130311113841.log",
"./RobotKIT_20130311113927.log",
"./RobotKIT_20130311114016.log",
"./RobotKIT_20130311114101.log",
"./RobotKIT_20130311114207.log",
"./RobotKIT_20130311114257.log",
"./RobotKIT_20130311114343.log",
"./RobotKIT_20130311111423.log", //0cm real data start from here
"./RobotKIT_20130311111614.log",
"./RobotKIT_20130311111727.log",
"./RobotKIT_20130311111825.log",
"./RobotKIT_20130311111919.log",
"./RobotKIT_20130311112022.log",
"./RobotKIT_20130311112110.log",
"./RobotKIT_20130311112214.log",
"./RobotKIT_20130311112303.log",
"./RobotKIT_20130311112347.log",
"./RobotKIT_20130311112438.log",
"./RobotKIT_20130311112527.log",
"./RobotKIT_20130311112612.log",
"./RobotKIT_20130311112720.log",
"./RobotKIT_20130311112814.log",
"./RobotKIT_20130311112859.log",
"./RobotKIT_20130311112951.log",
"./RobotKIT_20130311113049.log",
"./RobotKIT_20130311113140.log",
"./RobotKIT_20130311113242.log",
"./RobotKIT_20130311113328.log",
"./RobotKIT_20130311113416.log",
"./RobotKIT_20130311113509.log",
"./RobotKIT_20130311113745.log",
"./RobotKIT_20130311113841.log",
"./RobotKIT_20130311113927.log",
"./RobotKIT_20130311114016.log",
"./RobotKIT_20130311114101.log",
"./RobotKIT_20130311114207.log",
"./RobotKIT_20130311114257.log",
"./RobotKIT_20130311114343.log"
};


int MAX_REC_PER_FILE = 180;
int MAX_NUM_FILES = 61;
//pen[] Palette=Rainbow();
pen[] Palette=Grayscale();
int side = 1; //0--left, 1--right
real startx = -30;
real x_interval = 1;
real starty = 1.5;
real y_interval = 0.655;
real threshold = 8;

real []value;
pair []pos = new pair[MAX_REC_PER_FILE*MAX_NUM_FILES];
real []x;
real []y;
real []y1;
real []y2;
real []y3;

guide region1;
for(int i = 0; i < MAX_NUM_FILES; ++i)
{
    write(i);
    //mirror the right half
    file fin;
    if(i<MAX_NUM_FILES/2)
        fin=input(files[MAX_NUM_FILES - i -1]);
    else
        fin=input(files[i]);
    real [][]data = fin.dimension(MAX_REC_PER_FILE, 9);
    data=transpose(data);
    x[i]=startx+i*x_interval;
    y[i]=starty;
    y1[i]=starty;
    y2[i]=starty;
    y3[i]=starty;
    for(int j=0;j<MAX_REC_PER_FILE;++j)
    {
        pos[i*MAX_REC_PER_FILE + j] = (x[i] - 3, starty + j * y_interval);

        if(i<MAX_NUM_FILES/2)
            value[i*MAX_REC_PER_FILE + j]= data[2][j];//max(data[1][j], data[2][j]);
        else
            value[i*MAX_REC_PER_FILE + j]= data[1][j];//max(data[1][j], data[2][j]);
        if(data[1][j]>threshold && data[2][j]>threshold)
            y[i] = max(starty+j*y_interval, y[i]);
        int sum = 0;
        for(int k = 1;k < 9; ++k)
        {
            if(data[k][j]>threshold && k!=3)
            {
                y1[i] = max(starty+j*y_interval, y1[i]);
                ++sum;
            }
            if(data[k][j]>2*threshold && k!=3)
                y2[i] = max(starty+j*y_interval, y2[i]);
            if(data[k][j]>4*threshold && k!=3)
                y3[i] = max(starty+j*y_interval, y3[i]);

            //if(sum >=2)
            //    y2[i] = max(starty+j*y_interval, y2[i]);
            //if(sum >=3)
            //    y3[i] = max(starty+j*y_interval, y3[i]);
        }
    }

    if(y2[i] == starty)
        write("small ",x[i]);
}

//mannually flip the first 3 record to the other end
real []trans_value;
pair []trans_pos = new pair[MAX_REC_PER_FILE*MAX_NUM_FILES];

for(int i = 0; i<MAX_NUM_FILES;++i)
{
    for(int j=0;j<MAX_REC_PER_FILE;++j)
    {
        if(i<MAX_NUM_FILES-3)
            trans_value[i*MAX_REC_PER_FILE+j]=value[(i+3)*MAX_REC_PER_FILE+j];
        else
            trans_value[i*MAX_REC_PER_FILE+j]=value[(2-(i+3)%MAX_NUM_FILES)*MAX_REC_PER_FILE+j];
       trans_pos[i*MAX_REC_PER_FILE+j]= (x[i], starty + j * y_interval);

    }
}

write(x,y1);
picture bar;
bounds range=image(trans_pos,trans_value,Palette);
palette(bar,"",range,(0,0),(1cm,20cm),Right,Palette,
        PaletteTicks(scale(2)*"$%+#.1f$"));
add(bar.fit(),(33,5),UnFill);

real []xx;
for(int i = 0; i<MAX_NUM_FILES;++i)
{
    if(y[i]==starty)
        xx[i]=0;
    else
        xx[i]=x[i];
}
draw(graph(xx,y), red+1pt+dashed);
for(int i = 0; i<MAX_NUM_FILES;++i)
{
    if(y1[i]==starty)
        xx[i]=0;
    else
        xx[i]=x[i];
}
draw(graph(xx,y1), white+1pt+dashed);
for(int i = 0; i<MAX_NUM_FILES;++i)
{
    if(y2[i]==starty)
        xx[i]=0;
    else
        xx[i]=x[i];
}
draw(graph(xx,y2), green+1pt+dashed);
for(int i = 0; i<MAX_NUM_FILES;++i)
{
    if(y3[i]==starty)
        xx[i]=0;
    else
        xx[i]=x[i];
}
draw(graph(xx,y3), blue+1pt+dashed);
xlimits(-31, 31);
ylimits(0, 121);
yaxis(scale(1.5)*"y(cm)",LeftRight,RightTicks(Label(fontsize(14)),new real[]{0,10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120}));
xaxis(scale(1.5)*"x(cm)",BottomTop,LeftTicks(Label(fontsize(14)),new real[]{-30, -25, -20, -15,-10, -5, 0, 5, 10, 15, 20, 25, 30}));
