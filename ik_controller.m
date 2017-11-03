function [v,w,q] = ik_controller(v0,w0,x0,y0,theta0,xt,yt,ts,grid_size)

%% Initialize

% Max accelerations
vpmax=0.288; %(m/s^2)
wpmax=5.579; %(radians/s^2)

% Tolerances and resolution
tol=grid_size*0.001;
angle_tol=grid_size*0.05;
min_resolution=4;

% Initialize matrices and indexes
ii=0;
jj=1;
v=zeros(100000,1);
w=zeros(100000,1);
q=zeros(100000,3);

vinit=v0;
winit=w0;
q(jj,:)=[x0 y0 theta0];

dx=xt-x0;
dy=yt-y0;
dq=0;

% Calculate the target heading and distance
dqt=sqrt(dx^2+dy^2);
thetat=heading_calc(dx,dy);

%% Controls

    %% Turn Procedure
    if thetat~=theta0
        % Finding appropriate accelerations for our time steps
        dtheta=thetat-theta0;
        
        if 360-abs(dtheta) < abs(dtheta)
            dtheta=-(360-dtheta);
        end
        
        waccel=(dtheta)/(min_resolution*ts)^2;
        if abs(waccel) > wpmax
            for n=1:1000
                waccel=(dtheta)/(n*ts)^2;
                if abs(waccel) <= wpmax
                    break;
                end
            end
        end

        % Finding which direction the acceleration is in
        if waccel>0
            status=1; %ccw
        else
            status=0; %cw
        end

        % Find the necessary v and w control inputs to achieve the goal
        while (status==1 && (q(jj,3)-thetat)<-angle_tol) || (status==0 && (q(jj,3)-thetat)>angle_tol)
            % Accelerate
            if (status==1 && (q(jj,3)-(thetat+theta0)/2)<-angle_tol) || (status==0 && (q(jj,3)-(thetat+theta0)/2)>angle_tol)
                wend=winit+waccel*ts;
                w(ii+1)=(wend+winit)/2;
                v(ii+1)=0;
                q(jj+1,:) = robot_forwardprop(ts,v(ii+1),w(ii+1),q(jj,:));
                ii=ii+1;
                jj=jj+1;
                winit=wend;
            % Deccelerate
            else
                wend=winit-waccel*ts;
                w(ii+1)=(wend+winit)/2;
                v(ii+1)=0;
                q(jj+1,:) = robot_forwardprop(ts,v(ii+1),w(ii+1),q(jj,:));
                ii=ii+1;
                jj=jj+1;
                winit=wend;
            end
        end
    end

    %% Straight Line Procedure

    % Finding appropriate accelerations for our time steps
    vaccel=dqt/(min_resolution*ts)^2;
    if vaccel > vpmax
        for n=1:1000
            vaccel=dqt/(n*ts)^2;
            if vaccel <= vpmax
                break;
            end
        end
    end

    % Find the necessary v and w control inputs to achieve the goal
    while (dqt-dq)>tol
        % Accelerate
        if (dqt/2-dq)>tol
            vend=vinit+vaccel*ts;
            w(ii+1)=0;
            v(ii+1)=(vend+vinit)/2;
            q(jj+1,:) = robot_forwardprop(ts,v(ii+1),w(ii+1),q(jj,:));
            dq=dq+sqrt((q(jj+1,1)-q(jj,1))^2+(q(jj+1,2)-q(jj,2))^2);
            ii=ii+1;
            jj=jj+1;
            vinit=vend;
        % Deccelerate
        else
            vend=vinit-vaccel*ts;
            w(ii+1)=0;
            v(ii+1)=(vend+vinit)/2;
            q(jj+1,:) = robot_forwardprop(ts,v(ii+1),w(ii+1),q(jj,:));
            dq=dq+sqrt((q(jj+1,1)-q(jj,1))^2+(q(jj+1,2)-q(jj,2))^2);
            ii=ii+1;
            jj=jj+1;
            vinit=vend;
        end
    end
    
%% Output v and w and states

v=v(1:ii);
w=w(1:ii);
q=q(1:jj,:);


