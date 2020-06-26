% This is ECE 276B final project, indoor path planning simulation by Pengcheng Cao
% RRT* Planning
% Initialize the the room environment
close all
clear all
clc
xdim=700;
ydim=320;
zdim=270;
fig=figure('name','Received Data','units','normalized','position',[0.15 0.06 0.7 .83]);
ax=gca; backColor=[222/255 222/255 239/255];
ax.Color = backColor;
axis([0 xdim 0 ydim 0 zdim])
grid

fig.Children.Projection = 'perspective';

material('dull') %Not really needed, just makes the plane shinier.
camlight('headlight'); %... and better lit.

%%
global eta d gama
eta=4; %growth distance (between nearset point towards the random point)
d=2; %2D space
gama=2000; %check this????????????????????????

%% drawing walls and obstacles
% x1 --> x2   y1-->y2    ob#=[x1 x2 y1 y2]
global obd
obd=10;
extrude=200;
% xdim=700;
% ydim=320;
% zdim=270;
% bed dim 130x180x60
bed = [0 130 0 180 0 60]; plotObstacleExtrude(bed,'r');
% ac dim 30x30x80
air_cond = [0 30 190 220 0 80  ]; plotObstacleExtrude(air_cond,'r');
% tv dim 130x85x150
tv = [0 130 225 320 0 150];  plotObstacleExtrude(tv,'r');
% table dim is 85x85x95
table = [130 215 235 320 0 95]; plotObstacleExtrude(table,'r');
% bench dim is 38x38x60
bench1 = [140 178 202 240 0 60]; plotObstacleExtrude(bench1,'r');
bench2 = [182 220 180 218 0 60]; plotObstacleExtrude(bench2,'r');
% desks dim is 205x60x75
desks = [215 420 260 320 0 75]; plotObstacleExtrude(desks,'r');
% rotary chair dim is r = 25, h = 95
chair1 = [ 235 285 200 250 0 95];
chair2 = [ 335 385 230 280 0 95];
circleExtrude(260, 225, 25, 95);
circleExtrude(360, 250, 25, 95);
% fan dim is r = 7, h = 105
fan = [343 357 143 157 0 105];
circleExtrude(350, 150, 7, 105);
% coffee table dim is 50x50x45
c_table = [200 250 100 150 0 45]; plotObstacleExtrude(c_table,'r');
% kitchen dim is 310x62x270
kitchen = [130 420 0 62 0 270]; plotObstacleExtrude(kitchen,'w');
% kitchen misc dim is 103x103x130
kitchen_misc = [400 503 100 203 0 130]; plotObstacleExtrude(kitchen_misc,'r');
% bathroom dim is 280x183x270
bathroom = [420 700 0 183 0 270]; plotObstacleExtrude(bathroom,'w');
% closet dim is 110x60x180
closet = [440 550 260 320 0 180]; plotObstacleExtrude(closet,'r');
% dresser dim 140x46x91
dresser = [560 700 274 320 0 91]; plotObstacleExtrude(dresser,'r');
obstacles = [bed; air_cond; tv; table; bench1; bench2; desks; chair1; chair2;...
    fan; c_table; kitchen; kitchen_misc; bathroom; closet; dresser];
%% Starting point

Start = [50 210];

plot(Start(1,1), Start(1,2), '.k', 'MarkerSize',30)
%% Goal

Goal = [700 230];
radius=25; circle(Goal(1),Goal(2),radius);
x1 = 700;y1 = 230;str1 = 'Goal';text(x1,y1,str1,'Color','k','FontSize',15);

%% RRT* planning

Cost = [Start 0];

vv=[Start Start];

tree = [Start];
iter=0; %loop counter,keeps track of the number of iterations 
stp=0; %%stop condition initally zero, when it reaches Goal it is set to 1
tic
while (stp==0) 
   
iter=iter+1;

xrand=[xdim*rand ydim*rand];   % sample random point in collision free subspace     
xnearest=Nearest(xrand,tree);  % find the point nearest to the sampled point on the tree           
xnew=Steer(xnearest,xrand);    % steer from the tree vertex in xrand direction to obtain the xnew vertex

if CollisionFree(xnearest,xnew,obstacles)
tree=[tree;xnew]; % add xnew to the vertex set
temp=(gama*log(length(tree))/length(tree))^(1/d);
r=min(eta,temp); % r is the minimum of temp and eta
Xnear=Near(tree,xnew,r); % obtain the Xnear set via the Near function, ***check here
xmin=xnearest; %assign xmin


costrow=find(any(Cost(:,1)==xnearest(1),2));
cost=Cost(costrow,3)+norm(xnearest-xnew);
cmin=cost; % assign cmin
Cost=[Cost;[xnew cost]];
oldc=cost;


for indx=1:size(Xnear,1) % for each xnear in the Xnear set
xnear=Xnear(indx,:);
costrow=find(any(Cost(:,1)==xnear(1),2));
cxnear=Cost(costrow,3)+norm(xnear-xnew); %Connect along a minimum-cost path

if CollisionFree(xnear,xnew,obstacles) &&  (cxnear<cmin)
 xmin=xnear; cmin=cxnear; 

end

end
costrow=find(any(Cost(:,1)==xnew(1),2));
Cost(costrow,:)=[]; 


costrow=find(any(Cost(:,1)==xmin(1),2));
cost=Cost(costrow,3)+norm(xmin-xnew); 
Cost=[Cost;[xnew cost]];
vv=[vv;[xmin xnew]]; % add [xmin xnew] to the edge set
plot([xmin(1) xnew(1)],[xmin(2) xnew(2)],'b') %% plot tree branch

%% Rewire the tree
for indx=1:size(Xnear,1)
    xnear=Xnear(indx,:);
   cost=cost+norm(xnew-xnear); 
   costrow=find(any(Cost(:,1)==xnear(1),2));
  
   if CollisionFree(xnew,xnear,obstacles) &&  (cost<Cost(costrow,3)) %Cost(xnew) + cost([xnew-xnear])< Cost(xnear)
 row=find(any(vv(:,3)==xnear(1),2)); % ***check this, deleting the parent vertices of xnew
 
 plot([vv(row,1) xnear(1)],[vv(row,2) xnear(2)],'Color',backColor)% delete [xparent xnear] from the plot
 vv(row,:)=[];% delete [xparent xnear] from the edge set
 vv=[vv;[xnew xnear]];

 plot([xnew(1) xnear(1)],[xnew(2) xnear(2)],'r')%draw the new branch

end
   
   
end




%%
if norm(xnew-Goal)<radius
    stp=1;
    row=find(any(vv(:,3)==xnew(1),2));
    pathpoints_RRT=[];
    while row>1
        parentx=vv(row,1);
        plot([vv(row,1) vv(row,3)],[vv(row,2) vv(row,4)],':r','LineWidth',3);
        pathpoints_RRT=[pathpoints_RRT;[vv(row,1) vv(row,2)]];
        row=find(any(vv(:,3)==parentx,2));
        
    end
    
end

drawnow update 
end
end

toc
%% increasing number of points on the path
pat=[];
for i=1:size(pathpoints_RRT,1)-1
    n=eta*5;
    r=norm(pathpoints_RRT(i,:)-pathpoints_RRT(i+1,:))/n;
    
    for j=1:n
    pat=[pat;Steer2(pathpoints_RRT(i,:),pathpoints_RRT(i+1,:),j*r)];

    end
end

%% Patch, load robot model
global orig 
patchData = stlread('robot.stl'); %THIS IS THE ONLY LINE THAT'S REALLY DIFFERENT FROM BEFORE.
p1 = patch(patchData,'EdgeColor','none','FaceColor','[.2 0.5 1.0]');
orig=p1.Vertices;
iter=1;
%% smooth the path
ax=pat(1,1);
ay=pat(1,2);
pats=[];
for t=1:size(pat,1)
    f=0.99;
    ax=f*ax+(1-f)*pat(t,1);
    ay=f*ay+(1-f)*pat(t,2);
    pats=[pats;[ax ay]];
end
%% Follow smoothed path
 plot(pats(:,1),pats(:,2),'y','LineWidth',2)
 tic
 for k=size(pat,1):-1:2

     xdiff=pats(k-1,2)-pats(k,2);
     ydiff=pats(k-1,1)-pats(k,1);
 
     th=atan2(ydiff,xdiff);
%      camx=40*sin(th);
%      camy=40*cos(th);
%     
%     origTarget = [pats(k-1,1),pats(k-1,2),2];origPos = [pats(k-1,1)-camx,pats(k-1,2)-camy,10]';
%   campos(origPos);
%   camtarget(origTarget);
%      
%     camva(30);
     
    move2(p1,pats(k,1),pats(k,2),0,th);
    pause(.01)
    iter=iter+1;
 end
 toc
pause(1)
%% change camera view at the end
% for i=1:100
%     posx=((100-origPos(1))/100)*i+origPos(1);
%     posy=((-50-origPos(2))/100)*i+origPos(2);
%     posz=((60-origPos(3))/100)*i+origPos(3);
% campos([posx posy posz]);
% pause(0.02)
% end