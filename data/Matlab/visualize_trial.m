clear; close all; clc;

%% Load Dataset
trial_start = 01;
trial_end = 15;

extra = '';
folder = '2022-09-26';
name = 'trialq_';

N = trial_end-trial_start+1;
entry_point = zeros(2,N);
err_2d = zeros(1,N);

for trial=trial_start:trial_end
    load(strcat(folder,'/',name,num2str(trial,'%2.2d'),extra,'.mat'));

    %% Configure simulationb
    S = size(up,2);  % data size
    safe_limit = 6;
    base_init = base(:,1);

    %% Loop key
    k_key = find(key); % Samples when key was pressed
    ns = length(k_key);
    k = k_key(ns);
    if size(yp{1},2) == 3
        err_step = Z(1:3,k) - target;
    else
        err_step = Z(1:5,k) - [target;0;0];
    end
    entry_point(:,trial) = [target(1); target(3)];
    err_2d(trial) = sqrt(err_step(1)^2+err_step(3)^2);
    clearvars -except err_2d entry_point folder name extra trial_start trial_end trial
end

figure(1);
text(entry_point(1,:), entry_point(2,:), string(1:length(err_2d)));
hold on;
plot(entry_point(1,:), entry_point(2,:), '.k');
title(['Insertions in ' name])
set(gca, 'Xdir', 'reverse')
axis equal
box on
xlim([-51,21])
ylim([10, 54])
for i=1:length(err_2d)
    h = scatter(entry_point(1,i), entry_point(2,i), 300*err_2d(i));
end
