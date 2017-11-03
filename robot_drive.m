function [q,M] = robot_drive(speed,robot_traj,grid_size,v0,w0,thetalast,ts)

M=[];

x0=robot_traj(1,2);
y0=robot_traj(1,1);
ind=1;
q=zeros(100000,3);
q(ind,:)=[x0 y0 thetalast];

% If path is known, no need to stop unless heading needs to change
aa=size(robot_traj,1);
dy=robot_traj(aa,1)-robot_traj(aa-1,1);
dx=robot_traj(aa,2)-robot_traj(aa-1,2);

save_traj=robot_traj;
heading0=heading_calc(dx,dy);

for ii=(aa-1):-1:2
    dy=robot_traj(ii,1)-robot_traj(ii-1,1);
    dx=robot_traj(ii,2)-robot_traj(ii-1,2);
    heading1=heading_calc(dx,dy);
    
    if heading1==heading0
        save_traj(ii,:)=[];
    end 
    heading0=heading1;
end

% New efficient trajectory
robot_traj=save_traj;

% Run the trajectory...
for ii=1:size(robot_traj,1)-1
    
    x0=q(ind,1);
    y0=q(ind,2);
    
    xt=robot_traj(ii+1,2);
    yt=robot_traj(ii+1,1);
    
    [v,w,~] = ik_controller(v0,w0,x0,y0,thetalast,xt,yt,ts,grid_size);
    h = animatedline('Color','r','LineWidth',3);

    for jj=1:length(v)  
        q(ind+1,:) = robot_forwardprop(ts,v(jj),w(jj),q(ind,:));
        ind=ind+1;

        x = q(ind,1);
        y = q(ind,2);
        theta = q(ind,3);
        addpoints(h,x,y)
        heading=line([x x+0.5/grid_size*cos(theta)],[y y+0.5/grid_size*sin(theta)],'Color','c','LineWidth',3);

        M=[M;getframe];
        pause(speed)
        delete(heading)
    end
    
    v0=v(jj);
    w0=w(jj);
    thetalast=q(ind,3);
end

line([x x+0.5/grid_size*cos(theta)],[y y+0.5/grid_size*sin(theta)],'Color','c','LineWidth',3);
q=q(1:ind,:);

hold off

end