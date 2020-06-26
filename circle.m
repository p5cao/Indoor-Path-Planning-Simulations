function circle(x,y,r)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)
hold on
for ri=0:r/100:r
ang=0:0.01:2*pi; 
xp=ri*cos(ang);
yp=ri*sin(ang);
plot(x+xp,y+yp,'Color',[253/255 247/255 2/255]);
end
plot(x+xp,y+yp,'k');
end