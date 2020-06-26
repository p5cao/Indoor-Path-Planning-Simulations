function obj = circleExtrude(xx,yy,r,h)

theta = 0:0.05:2*pi;

x = r*cos(theta)+xx;
y = r*sin(theta)+yy;
y(end) = yy;
z1 = 0;
z2 = h;
col=[245/255 3/255 70/255];


set(gca,'NextPlot','Add');
h=patch(x,y,z2*ones(size(x)),'y');
set(h,'edgecolor','k','FaceColor',col)
patch(x,y,z1*ones(size(x)),'edgecolor','k','FaceColor',col);

hSurface=surf([x;x],[y;y],[z1*ones(size(x));z2*ones(size(x))],'parent',gca,'LineStyle','none');
set(hSurface,'FaceColor',col);
