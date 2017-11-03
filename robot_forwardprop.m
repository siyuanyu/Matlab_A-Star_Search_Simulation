%% Robot Frame Forward Propogate
  
function [q] = robot_forwardprop(ts,vcurr,wcurr,q0)

% arclength
s=ts*vcurr;

% angle
alpha=ts*wcurr;

% dx and dy in robot reference frame
if alpha==0
   % no rotation, only movement in robot x direction
   dx_robot=s;
   dy_robot=0;
elseif vcurr > 0
   % dx and dy calculated from rotation and velocity
   r=s/alpha;
   chord=2*r*sin(alpha/2);
   dx_robot=chord*cos(alpha/2);
   dy_robot=chord*sin(alpha/2);
else
    % only rotation
   dx_robot=0;
   dy_robot=0;
end

% change in states in the robot frame
dq_robot=[dx_robot;dy_robot];

% transform to global frame
theta=q0(3);
R=[cos(theta) -sin(theta); sin(theta) cos(theta)];
q(1:2)=q0(1:2)+(R*dq_robot)';
q(3)=q0(3)+alpha;

% constrain theta to less than +/- pi for plotting comparison
if q(3) > pi
   q(3)=q(3)-2*pi;
elseif q(3) < -pi
   q(3)=q(3)+2*pi;
end

end

