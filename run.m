%% Initialize
clear
clc
close all
load('ds1_landmarks.mat')

% Parameters
% xrange=[-2,5];
% yrange=[-6,6];

% Format...
% HW1_PartA(landmarks,xrange,yrange,grid_size,obs_size,start,goal)

%% Runs

    %% Problem 3: Grid Size = 1m (Figures 1-3)

   %[~,~]=HW1_PartA(landmarks,xrange,yrange,grid_size,obs_size,start,goal)
    [~,~]=HW1_PartA(ds1_landmarks,[-2,5],[-6,6],1,0.0001,[0.5,-1.5],[0.5,1.5]);
    [~,~]=HW1_PartA(ds1_landmarks,[-2,5],[-6,6],1,0.0001,[4.5,3.5],[4.5,-1.5]);
    [~,~]=HW1_PartA(ds1_landmarks,[-2,5],[-6,6],1,0.0001,[-0.5,5.5],[1.5,-3.5]);

    %% Problem 5: Grid Size = 0.1m (Figures 4-6)
   
   %[~,~]=HW1_PartA(landmarks,xrange,yrange,grid_size,obs_size,start,goal)
    [~,~]=HW1_PartA(ds1_landmarks,[-2,5],[-6,6],0.1,0.3,[2.45,-3.55],[0.95,-1.55]);
    [~,~]=HW1_PartA(ds1_landmarks,[-2,5],[-6,6],0.1,0.3,[4.95,-0.05],[2.45,0.25]);
    [~,~]=HW1_PartA(ds1_landmarks,[-2,5],[-6,6],0.1,0.3,[-0.55,1.45],[1.95,3.95]);
    
    %% Problem 7: Drive a planned path, Grid Size = 0.1m (Figure 7)
    
    % Use 0.01 for faster running...
    SPEED=0.1;
    %SPEED=0.1;
    
   %[~]=HW1_PartB(onboard?,landmarks,xrange,yrange,grid_size,obs_size,start,goal)
    [q1,Movie7a]=HW1_PartB(0,SPEED,ds1_landmarks,[-2,5],[-6,6],0.1,0.3,[2.45,-3.55],[0.95,-1.55]);
    [q2,Movie7b]=HW1_PartB(0,SPEED,ds1_landmarks,[-2,5],[-6,6],0.1,0.3,[4.95,-0.05],[2.45,0.25]);
    [q3,Movie7c]=HW1_PartB(0,SPEED,ds1_landmarks,[-2,5],[-6,6],0.1,0.3,[-0.55,1.45],[1.95,3.95]);
    
    %% Problem 8: Drive while planning, Grid Size = 0.1m (Figure 8)
    
   %[~]=HW1_PartB(onboard?,landmarks,xrange,yrange,grid_size,obs_size,start,goal)
    [q4,Movie8a]=HW1_PartB(1,SPEED,ds1_landmarks,[-2,5],[-6,6],0.1,0.3,[2.45,-3.55],[0.95,-1.55]);
    [q5,Movie8b]=HW1_PartB(1,SPEED,ds1_landmarks,[-2,5],[-6,6],0.1,0.3,[4.95,-0.05],[2.45,0.25]);
    [q6,Movie8c]=HW1_PartB(1,SPEED,ds1_landmarks,[-2,5],[-6,6],0.1,0.3,[-0.55,1.45],[1.95,3.95]);
    
    %% Problem 9: Drive while planning, Grid Size = 0.1m (Figure 9)
    
   %[~]=HW1_PartB(onboard?,landmarks,xrange,yrange,grid_size,obs_size,start,goal)
    [q7,Movie9a]=HW1_PartB(1,SPEED,ds1_landmarks,[-2,5],[-6,6],1,0.0001,[0.5,-1.5],[0.5,1.5]);
    [q8,Movie9b]=HW1_PartB(1,SPEED,ds1_landmarks,[-2,5],[-6,6],1,0.0001,[4.5,3.5],[4.5,-1.5]);
    [q9,Movie9c]=HW1_PartB(1,SPEED,ds1_landmarks,[-2,5],[-6,6],1,0.0001,[-0.5,5.5],[1.5,-3.5]);