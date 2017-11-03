%% Plots a grid

function [] = grid_plot(status,grid_in,grid_size,obs_size,xrange,yrange)

%% Plot

if status==0
    figs = figure();
    set(figs, 'Position', [100 100 1000 500])
    subplot(1,2,1)
elseif status==1
    subplot(1,2,2)
end

imagesc(grid_in);
colormap([1 1 1;0 0 0;0 1 0;1 0 0;0 0 1])
grid()

hold on

% grid tick size
xgrid_ticks=size(grid_in,2)/(xrange(2)-xrange(1));
ygrid_ticks=size(grid_in,1)/(yrange(2)-yrange(1));

% setting up axis range
axis([0.5*grid_size (size(grid_in,2)+0.5*grid_size) 0.5*grid_size (size(grid_in,1)+0.5*grid_size)])
pbaspect([size(grid_in,2) size(grid_in,1) 1])

% setting up grid ticks
set(gca,'xtick',0.5*grid_size:xgrid_ticks:size(grid_in,2)+0.5*grid_size)
set(gca,'ytick',0.5*grid_size:ygrid_ticks:size(grid_in,1)+0.5*grid_size)

% setting up grid tick labels
set(gca,'XTickLabel',xrange(1):xrange(2))
set(gca,'YTickLabel',yrange(1):yrange(2))

% plot labels
if status==0
    title('Occupancy Grid')
else
    title('Robot Vision')
end

xlabel('X position (m)')
ylabel('Y position (m)')
caxis([0, 4])
colorbar('Ticks',[2/5 6/5 10/5 14/5 18/5],'TickLabels',{'Free Space','Obstacle','Start','Goal','Planned Traj.'})
set(gca,'fontsize',12)

end

