import graph;
import palette;
size(10cm,0cm);
string []files = {
"./RobotKIT_20070101003158.log",
"./RobotKIT_20070101003243.log",
"./RobotKIT_20070101003332.log",
"./RobotKIT_20070101003414.log",
"./RobotKIT_20070101003516.log",
"./RobotKIT_20070101003605.log",
"./RobotKIT_20070101003655.log",
"./RobotKIT_20070101003749.log",
"./RobotKIT_20070101003832.log",
"./RobotKIT_20070101003919.log",
"./RobotKIT_20070101004010.log",
"./RobotKIT_20070101004058.log",
"./RobotKIT_20070101004145.log",
"./RobotKIT_20070101004236.log",
"./RobotKIT_20070101004322.log"
};

int MAX_REC_PER_FILE = 160;
int MAX_NUM_FILES = 15;
pen[] Palette=Rainbow();
int side = 1; //0--left, 1--right
real startx = -7;
real x_interval = 1;
real starty = 2;
real y_interval = 0.5;
real threshold = 4;

real []value;
pair []pos = new pair[MAX_REC_PER_FILE*MAX_NUM_FILES];
real []x;
real []y;

guide region1;
for(int i = 0; i < MAX_NUM_FILES; ++i)
{
    file fin=input(files[i]);
    real [][]data = fin.dimension(MAX_REC_PER_FILE,2);
    data=transpose(data);
    x[i]=startx+i*x_interval;
    y[i]=starty;
    for(int j=0;j<MAX_REC_PER_FILE;++j)
    {
        pos[i*MAX_REC_PER_FILE + j] = (startx + i * x_interval, starty + j * y_interval);
        value[i*MAX_REC_PER_FILE + j]= max(data[0][j], data[1][j]);
        if(data[0][j]>threshold && data[1][j]>threshold)
            y[i] = max(starty+j*y_interval, y[i]);
    }
}

write(x,y);
picture bar;
bounds range=image(pos,value,Palette);
palette(bar,"",range,(0,0),(0.5cm,8cm),Right,Palette,
        PaletteTicks("$%+#.1f$"));
add(bar.fit(),(17,20),UnFill);

draw(graph(x,y), black+1pt+dashed);
xlimits(-15, 15);
ylimits(0, 90);
yaxis("y",LeftRight,RightTicks(Label(fontsize(12)),new real[]{0,10, 20, 30, 40, 50, 60, 70, 80, 90}));
xaxis("x",BottomTop,LeftTicks(Label(fontsize(12)),new real[]{-15,-10, -5, 0, 5, 10, 15}));
