%% Heuristic Calculations

function [cost,grid_cost,grid_robot] = heuristic(check,goal,robot_traj,grid_robot,grid_cost,grid_truth)

row=check(1);
col=check(2);

%% Path cost (straight-line distance)

h=grid_cost(row,col);

%% Step cost

if grid_truth(row,col)==1
    % obstacle cost
    g=1000;
    grid_robot(row,col)=1;
else
    % free space cost
    g=1;
end

% Identify if the space has been traversed before
count=0;

% Increase the cost of the spaces that have been traversed before
for ii=1:size(robot_traj,1)
     if isequal(check,robot_traj(ii,:))
        if count==0
            g=500;
            count=count+1;
        else
            g=g+500;
            count=count+1;
        end
     end
end

% Increase the cost of the previous space
if size(robot_traj,1)>1
    if isequal(check,robot_traj(size(robot_traj,1)-1,:))
        g=g+100;
    end
end

if isequal(check,goal)
    cost=0;
else
    cost=h+g;
end

end

