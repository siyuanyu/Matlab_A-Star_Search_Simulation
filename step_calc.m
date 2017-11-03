%% Performs A-star Calcs for the current position and suggests the next step

function [next_step,robot_traj,grid_robot,grid_cost] = step_calc(current,goal,robot_traj,grid_robot,grid_cost,grid_truth)

% Initialize
check=[0,0];
neighbors=[];

for ii=-1:1
    for jj=-1:1
        % Skip current space
        if (ii==0) && (jj==0)
        else   
            % Space that is being checked
            check(1)=current(1)+ii;
            check(2)=current(2)+jj;

            % If the space is valid, calculate its cost
            if (check(1) > 0) && (check(1) <= size(grid_truth,1)) && (check(2) > 0) && (check(2) <= size(grid_truth,2))
                [cost,grid_cost,grid_robot]=heuristic(check,goal,robot_traj,grid_robot,grid_cost,grid_truth);
                neighbors=[neighbors;cost,check(1),check(2)];
            end
        end
    end
end

% Finds lowest cost and makes the step
neighbors=sortrows(neighbors,1);
next_step=[neighbors(1,2),neighbors(1,3)];
robot_traj=[robot_traj;current];

end

