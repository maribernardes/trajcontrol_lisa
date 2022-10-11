clear all;
close all;
clc;

%% Select Dataset
trial = 0;
extra = 'a':'g';
folder = '2022-10-04';
name = 'trialr_';

for j=1:length(extra)
    %% Load data
    load(strcat(folder,'/',name,num2str(trial,'%2.2d'),extra(j),'.mat'));

    %% Loop key
    k_key = find(key); % Samples when key was pressed
    ns = length(k_key);
    T = zeros(1,ns);
    for i=1:ns
        k = k_key(i);   %sample
        if size(yp{1},2) == 3
            err_step(i,:) = Z(1:3,k) - target;
        else
            err_step(i,:) = Z(1:5,k) - [target;0;0];
        end
    end

    err_x(j) = abs(err_step(end,1));
    err_y(j) = abs(err_step(end,2));
    err_z(j) = abs(err_step(end,3));

    err_2d(j) = sqrt(err_step(end,1)^2+err_step(end,3)^2);
    err_3d(j) = sqrt(err_step(end,1)^2+err_step(end,2)^2+err_step(end,3)^2);

    if size(err_step,2) == 5
        err_angle_v(i) = abs(err_step(end,4));
        err_angle_h(i) = abs(err_step(end,5));
    end

    fprintf('Trial %2.2d%s \t Err 2D[mm] = %0.4f\n', trial, extra(j), err_2d(j));
end

fprintf('Final error 2D[mm] = %0.4f +- %0.4f\n', mean(err_2d), std(err_2d));


