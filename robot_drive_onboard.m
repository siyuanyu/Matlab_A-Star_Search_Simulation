function [q,M,robot_traj] = robot_drive_onboard(speed,landmarks,xrange,yrange,grid_size,obs_size,start_coords,goal_coords)

%% Initialize

% Example Inputs
% xrange=[-2,5];
% yrange=[-6,6];
% grid_size=1;
% obs_size=0.0001;
% start=[0.5,1.5];
% goal=[0.5,-1.5];

% Create a truth grid and a robot grid
[grid_truth,grid_robot]=grid_create(landmarks(:,2:3),obs_size,grid_size,xrange,yrange);

% Populate the start and goal locations in both truth and robot grids
start=find_grid_location(start_coords,grid_size,xrange,yrange);
goal=find_grid_location(goal_coords,grid_size,xrange,yrange);

grid_truth(start(1,1),start(1,2))=2;
grid_truth(goal(1,1),goal(1,2))=3;

grid_robot(start(1,1),start(1,2))=2;
grid_robot(goal(1,1),goal(1,2))=3;

grid_plot(0,grid_truth,grid_size,obs_size,xrange,yrange);

% Initial Plot
grid_plot(1,grid_robot,grid_size,obs_size,xrange,yrange);
super=suptitle(['Planning While Driving, Grid Size = ' num2str(grid_size) 'm']);
set(super,'FontSize',18,'FontWeight','Bold')

%% Onboard A-Star

% Create and populate a cost grid based on just the distance to the goal
grid_cost=zeros(size(grid_robot));

for ii=1:size(grid_cost,2)
    for jj=1:size(grid_cost,1)
        dx = abs(ii-goal(2));
        dy = abs(jj-goal(1));
        grid_cost(jj,ii)=sqrt(dx^2+dy^2);
    end
end

% Initialize for step calculations
robot_traj=[];
current=start;
iters=0;
iters_cap=500;
%(size(robot_grid,1)*size(robot_grid,2)/2)

% A-Star calculations
q=[start(2) start(1) pi/2];
M=[];

while ~isequal(current,goal) && (iters<iters_cap)
    
    [current,robot_traj,grid_robot,grid_cost] = step_calc(current,goal,robot_traj,grid_robot,grid_cost,grid_truth);
    grid_robot(current(1),current(2))=4;
    
    iters=iters+1;
    last=size(q,1);
    robot_step=[q(last,2) q(last,1);current];
    
    % Show new neighbors
    grid_plot(2,grid_robot,grid_size,obs_size,xrange,yrange);
    
    % Drive as A-Star plans
    [newq,newM] = robot_drive(speed,robot_step,grid_size,0,0,q(last,3),0.1);
    q=[q;newq];
    M=[M;newM];
end

robot_traj=[robot_traj;current];

end

