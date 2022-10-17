clear all;
close all;
clc;

%% Calculate mean and std deviation values for all insertions from a given trial
%% Select appropriate dataset (with manual selection of intervals for trial and extra)
%% Select if no_compensation (00) or compensated trials
%% Select if MPC3 algorithm (with angles) or previous version

%% Select Dataset
trial = 1:5;
extra = 'a':'h';
folder = '2022-10-17';
name = 'exp_b3_';
open_loop = false;  %True = Open-loop insertion (no compensation): trial?-00?
mpc3 = true;        %True = Used MPC3 (with angles)

%% Initialize vectors
if open_loop == true
    N = length(extra);
else
    N = length(trial);
end
if mpc3 == true
    err_step = zeros(5,N);
    [err_angle_v, err_angle_h] = deal(zeros(N,1));
else
    err_step = zeros(3,N);
end
[err_x, err_y, err_z, err_2d, err_3d] = deal(zeros(N,1));   

%% Get error for each trial
for j=1:N
    %% Load data
    if open_loop == true
        load(strcat(folder,'/',name,num2str(0,'%2.2d'),extra(j),'.mat'));
    else
        load(strcat(folder,'/',name,num2str(trial(j),'%2.2d'),'.mat'));
    end

    %% Loop all steps
    k_key = find(key); % Samples when key was pressed
    ns = length(k_key);
    
    %% Get final error
    if size(yp{1},2) == 3
        err_step = Z(1:3,k_key(ns)) - target;
    else
        err_step = Z(1:5,k_key(ns)) - [target;0;0];
    end    
    err_x(j) = abs(err_step(1,end));
    err_y(j) = abs(err_step(2,end));
    err_z(j) = abs(err_step(3,end));

    err_2d(j) = sqrt(err_step(1,end)^2+err_step(3,end)^2);
    err_3d(j) = sqrt(err_step(1,end)^2+err_step(2,end)^2+err_step(3,end)^2);

    if size(err_step,2) == 5
        err_angle_v(j) = abs(err_step(4,end));
        err_angle_h(j) = abs(err_step(5,end));
    end

    if open_loop == true
        fprintf('Trial 00%s \t Err 2D[mm] = %0.4f\n', extra(j), err_2d(j));
    else
        fprintf('Trial %2.2d \t Err 2D[mm] = %0.4f\n', trial(j), err_2d(j));
    end
end

fprintf('Final error 2D[mm] = %0.4f +- %0.4f\n', mean(err_2d), std(err_2d));


