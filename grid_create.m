%% Creates the initial occupancy grid

function [grid_truth,grid_blank] = grid_create(obstacles,obs_size,grid_size,xrange,yrange)

% Input Examples...
% grid_size=1;
% xrange=[-2,5];
% yrange=[-6,6];
% grid_truth=grid_create(landmarks,0.3,0.1,[-2,5],[-6,6]);

%% Initializing the full width of the square obstacle

obs_size=2*obs_size;

%% Initializing the occupancy grid

x_size=(xrange(2)-xrange(1))/grid_size;
y_size=(yrange(2)-yrange(1))/grid_size;
grid_blank=zeros(y_size,x_size);
grid_truth=zeros(y_size,x_size);

%% Finding an appropriate resolution for the discretized obstacles

obs_divisions=ceil(obs_size/grid_size);
obs_resolution=obs_size/obs_divisions;

%% Populating the occupancy grid (0=free space,1=occupied)

for ii=1:length(obstacles)
    for jj=0:obs_divisions
        for kk=0:obs_divisions
            xx=obstacles(ii,1)+(jj*obs_resolution)-(obs_size/2);
            yy=obstacles(ii,2)+(kk*obs_resolution)-(obs_size/2);

            [grid_location] = find_grid_location([xx,yy],grid_size,xrange,yrange);
            xgrid=grid_location(1,2);
            ygrid=grid_location(1,1);
            
            if (1 <= xgrid) && (xgrid <= size(grid_truth,2)) && (1 <= ygrid) && (ygrid <= size(grid_truth,1))
                grid_truth(ygrid,xgrid)=1;
            end
        end
    end
end

end
