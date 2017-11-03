function [q,M] = HW1_PartB(onboard,speed,landmarks,xrange,yrange,grid_size,obs_size,start_coords,goal_coords)

% Onboard
if onboard==1
    [q,M,~] = robot_drive_onboard(speed,landmarks,xrange,yrange,grid_size,obs_size,start_coords,goal_coords);
    
% Offboard
else
    % Plan the path
    [robot_traj,grid_size]=HW1_PartA(landmarks,xrange,yrange,grid_size,obs_size,start_coords,goal_coords);
    super=suptitle(['Driving a Planned Path, Grid Size = ' num2str(grid_size) 'm']);
    set(super,'FontSize',18,'FontWeight','Bold')
    
    % Drive the path using the inverse kinematic controller
    [q,M] = robot_drive(speed,robot_traj,grid_size,0,0,pi/2,0.1);
end

end
