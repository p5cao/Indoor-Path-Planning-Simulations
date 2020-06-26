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
gama=200; 

%% drawing walls and obstacles
% x1 --> x2   y1-->y2    ob#=[x1 x2 y1 y2]
global obd
obd=5;
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

plot(Start(1,1), Start(1,2), '.r', 'MarkerSize',30)
%% Goal

Goal = [700 230];
radius=25; circle(Goal(1),Goal(2),radius);
x1 = 700;y1 = 230;str1 = 'Goal';text(x1,y1,str1,'Color','k','FontSize',15);

%% weighted A* Planning
tic
Cost = [Start 0];
vv=[Start Start];
% Heuristic = [Start norm(Goal - Start)];% Use second norm as heuristic function
Expanded = [Start 0];
OPEN = [Start];
f = [0];
CLOSED = [];
iter=0; %loop counter,keeps track of the number of iterations 
stp=0; %%stop condition initally zero, when it reaches goal it is set to 1
epsilon = 2;
search_step = 2;

while (stp == 0)
    % make the index of f and OPEN consistent
    [f_i, idx] = min(f);
    i = OPEN(idx,:);
    % remove i and f_i
    OPEN(idx,:) = [];
    f(idx) = [];
    idx_g= find(Expanded(:,1:2) == i);
    g_i = Expanded(idx_g(1),3);
    % Expand Children nodes from neighbors
    Children = Neib(i, search_step, obstacles);
    % insert into closed list, if inserted into closed list cannot be
    % inserted into OPEN again
    CLOSED = [CLOSED;[i g_i]];
    if norm(i-Goal) <= 5
        stp = 1;
        row=find(any(vv(:,3)==i(1),2));
        pathpoints=[];
        while row>1
            parentx=vv(row,1);
            plot([vv(row,1) vv(row,3)],[vv(row,2) vv(row,4)],':r','LineWidth',3);
            pathpoints=[pathpoints;[vv(row,1) vv(row,2)]];
            row=find(any(vv(:,3)==parentx(1),2));
            row = row(1);
        end
    end
    for ii = 1: length(Children)
        if stp == 1
            break
        end
        iter = iter + 1;
        child = [Children(ii,1) Children(ii,2)];
        % if child not in CLOSED
        if ~any(ismember(CLOSED(:,1:2),child,'rows'))
            if stp == 1
                break
            end
            % if child not in Expanded, expand the child node, init value by
            % infinity
            if ~any(ismember(Expanded(:,1:2),child,'rows'))
                g_j = Inf;
                Expanded = [Expanded; [child g_j]];
                idx_gj= find(Expanded(:,1:2) == child);
            else
                idx_gj= find(Expanded(:,1:2) == child);
                g_j = Expanded(idx_gj(1),3);
            end 
            cost_ij = norm(i-child);
            if g_j > g_i + cost_ij
                g_j = g_i + cost_ij;
                % update g_j in Expanded
                Expanded(idx_gj(1),3) = g_j;
                vv = [vv; i child];
                heuristic = norm(child - Goal);
                f_j = g_j + epsilon * heuristic;
                if any(ismember(OPEN,child,'row'))
                    idx_f = find(OPEN(:,1:2) == child);
                    f(idx_f(1)) = f_j;
                else
                    OPEN = [OPEN; child];
                    f = [f;f_j];
                end
            end
        end
    end
end
toc
%% scatter stars to illustrate searched regions

scatter(OPEN(:,1),OPEN(:,2),'*','b') 

%% increasing number of points on the path
pat=[];
for i=1:size(pathpoints,1)-1
    n=eta*5;
    r=norm(pathpoints(i,:)-pathpoints(i+1,:))/n;
    
    for j=1:n
    pat=[pat;Steer2(pathpoints(i,:),pathpoints(i+1,:),j*r)];

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
    f=0.8;
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
%     origGoal = [pats(k-1,1),pats(k-1,2),2];origPos = [pats(k-1,1)-camx,pats(k-1,2)-camy,10]';
%     campos(origPos);
%     camtarget(origGoal);
%      
%     camva(30);
     
    move2(p1,pats(k,1),pats(k,2),0,th);
    pause(.01)
    iter=iter+1;
 end
 toc
pause(1)
