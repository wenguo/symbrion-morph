import graph;
import palette;
size(20cm,0cm);
string []files = {
"./RobotKIT_20130305125213.log",
"./RobotKIT_20130305125427.log",
"./RobotKIT_20130305125542.log",
"./RobotKIT_20130305125657.log",
"./RobotKIT_20130305125749.log",
"./RobotKIT_20130305125845.log",
"./RobotKIT_20130305125953.log",
"./RobotKIT_20130305130045.log",
"./RobotKIT_20130305130134.log",
"./RobotKIT_20130305130251.log",
"./RobotKIT_20130305130337.log",
"./RobotKIT_20130305130422.log",
"./RobotKIT_20130305130516.log",
"./RobotKIT_20130305130606.log",
"./RobotKIT_20130305130655.log",
"./RobotKIT_20130305130743.log",
"./RobotKIT_20130305130827.log",
"./RobotKIT_20130305130920.log",
"./RobotKIT_20130305131002.log",
"./RobotKIT_20130305131056.log",
"./RobotKIT_20130305133034.log",
"./RobotKIT_20130305133121.log",
"./RobotKIT_20130305133215.log",
"./RobotKIT_20130305133302.log",
"./RobotKIT_20130305133354.log",
"./RobotKIT_20130305133447.log",
"./RobotKIT_20130305133908.log",
"./RobotKIT_20130305134003.log",
"./RobotKIT_20130305134058.log",
"./RobotKIT_20130305134145.log",
"./RobotKIT_20130305134230.log",
"./RobotKIT_20130305134315.log",
"./RobotKIT_20130305134400.log",
"./RobotKIT_20130305134528.log",
"./RobotKIT_20130305134616.log",
"./RobotKIT_20130305134700.log",
"./RobotKIT_20130305134749.log",
"./RobotKIT_20130305134835.log",
"./RobotKIT_20130305134918.log",
"./RobotKIT_20130305135003.log",
"./RobotKIT_20130305135054.log",
"./RobotKIT_20130305135139.log",
"./RobotKIT_20130305135226.log",
"./RobotKIT_20130305135318.log",
"./RobotKIT_20130305135415.log",
"./RobotKIT_20130305135511.log",
"./RobotKIT_20130305135602.log",
"./RobotKIT_20130305135650.log",
"./RobotKIT_20130305135737.log",
"./RobotKIT_20130305135823.log",
"./RobotKIT_20130305135947.log"
};

int MAX_REC_PER_FILE = 160;
int MAX_NUM_FILES = 51;
//pen[] Palette=Rainbow();
pen[] Palette=Grayscale();
int side = 1; //0--left, 1--right
real startx = -25;
real x_interval = 1;
real starty = 1.5;
real y_interval = 0.63125;
real threshold = 6;

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

            if(sum >=2)
                y2[i] = max(starty+j*y_interval, y2[i]);
            if(sum >=3)
                y3[i] = max(starty+j*y_interval, y3[i]);
        }
    }
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

write(x,y);
picture bar;
bounds range=image(trans_pos,trans_value,Palette);
palette(bar,"",range,(0,0),(1cm,20cm),Right,Palette,
        PaletteTicks(scale(2)*"$%+#.1f$"));
add(bar.fit(),(30,5),UnFill);

draw(graph(x,y), red+1pt+dashed);
draw(graph(x,y1), white+1pt+dashed);
//draw(graph(x,y2), green+1pt+dashed);
//draw(graph(x,y3), white+1pt+dashed);
xlimits(-26, 26);
ylimits(0, 105);
yaxis(scale(1.5)*"y(cm)",LeftRight,RightTicks(Label(fontsize(14)),new real[]{0,10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110}));
xaxis(scale(1.5)*"x(cm)",BottomTop,LeftTicks(Label(fontsize(14)),new real[]{-25, -20, -15,-10, -5, 0, 5, 10, 15, 20, 25}));
