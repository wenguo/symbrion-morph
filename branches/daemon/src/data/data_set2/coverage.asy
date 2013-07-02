import graph;
import palette;
size(20cm,0cm);
string []files = {
"./RobotKIT_20070101001536.log",
"./RobotKIT_20070101001707.log",
"./RobotKIT_20070101001819.log",
"./RobotKIT_20070101001905.log",
"./RobotKIT_20070101002017.log",
"./RobotKIT_20070101002124.log",
"./RobotKIT_20070101002202.log",
"./RobotKIT_20070101002251.log",
"./RobotKIT_20070101002349.log",
"./RobotKIT_20070101002422.log",
"./RobotKIT_20070101002504.log",
"./RobotKIT_20070101002549.log",
"./RobotKIT_20070101002644.log",
"./RobotKIT_20070101002734.log",
"./RobotKIT_20070101002838.log",
"./RobotKIT_20070101002927.log",
"./RobotKIT_20070101003024.log",
"./RobotKIT_20070101003128.log",
"./RobotKIT_20070101003214.log",
"./RobotKIT_20070101003301.log",
"./RobotKIT_20070101003345.log",
"./RobotKIT_20070101003439.log",
"./RobotKIT_20070101003541.log",
"./RobotKIT_20070101003654.log",
"./RobotKIT_20070101003747.log",
"./RobotKIT_20070101003827.log",
"./RobotKIT_20070101003912.log",
"./RobotKIT_20070101003952.log",
"./RobotKIT_20070101004042.log"
};

int MAX_REC_PER_FILE = 160;
int MAX_NUM_FILES = 29;
pen[] Palette=Rainbow();
int side = 1; //0--left, 1--right
real startx = -14;
real x_interval = 1;
real starty = 7;
real y_interval = 0.5;
real threshold = 3;

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
palette(bar,"",range,(0,0),(1cm,20cm),Right,Palette,
        PaletteTicks(scale(2)*"$%+#.1f$"));
add(bar.fit(),(17,20),UnFill);

draw(graph(x,y), black+1pt+dashed);
xlimits(-15, 15);
ylimits(0, 90);
yaxis(scale(2.5)*"y(cm)",LeftRight,RightTicks(Label(fontsize(24)),new real[]{0,10, 20, 30, 40, 50, 60, 70, 80, 90}));
xaxis(scale(2.5)*"x(cm)",BottomTop,LeftTicks(Label(fontsize(24)),new real[]{-15,-10, -5, 0, 5, 10, 15}));
