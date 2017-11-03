%% Finds grid location from global coordinates

function [grid_location] = find_grid_location(coords,grid_size,xrange,yrange)

grid_location=zeros(size(coords,1),2);

for ii=1:size(coords,1)
    xvalue=floor(coords(ii,1)/grid_size)*grid_size;
    yvalue=floor(coords(ii,2)/grid_size)*grid_size;

    xgrid=round((xvalue-xrange(1))/grid_size+1);
    ygrid=round((yvalue-yrange(1))/grid_size+1);

    grid_location(ii,:)=[ygrid,xgrid];    
end

end

