function []=plotObstacleExtrude(ob,color)
x=ob(1); y=ob(3); z=ob(5); 
lx=ob(2)-ob(1); ly=ob(4)-ob(3); lz=ob(6)-ob(5);


x=[x (lx+x) (lx+x) x x x;(lx+x) (lx+x) x x (lx+x) (lx+x);(lx+x) (lx+x) x x (lx+x) (lx+x);x (lx+x) (lx+x) x x x];
y=[y y (ly+y) (ly+y) y y;y (ly+y) (ly+y) y y y;y (ly+y) (ly+y) y (ly+y) (ly+y);y y (ly+y) (ly+y) (ly+y) (ly+y)];
z=[z z z z z (lz+z);z z z z z (lz+z);(lz+z) (lz+z) (lz+z) (lz+z) z (lz+z);(lz+z) (lz+z) (lz+z) (lz+z) z (lz+z)];
for i=1:6
    if color == 'r'
        col=[245/255 3/255 70/255];
    else
        if color == 'w'
            col = [222/255 222/255 239/255];
        end
    end
    h=patch(x(:,i),y(:,i),z(:,i),'y');
    
    set(h,'edgecolor','k','FaceColor',col)
end

end