%With X = [x_base; y_base; z_base] and Z = [x_tip; y_tip; z_tip; horizangle_tip vertiangle_tip]\n');

clear; close all; clc;
global safe_limit;
global base_init;
global target;
global T;
global K;

%% Load Dataset
trial = 01;
folder = '2022-07-26';
name = 'trialc_';
load(strcat(folder,'/',name,num2str(trial,'%2.2d'),'.mat'));

%% Configure simulation
N = size(X,2);  % data size
alpha = 0.65; 
K = 0.05;
safe_limit = 6;
base_init = base(:,1);

%% Loop key
k_key = find(key); % Samples when key was pressed
Nk = length(k_key);
T = zeros(1,Nk);
for i=1:Nk
    k = k_key(i);   %sample
    T(:,i) = t(k);
end

%% Loop all measurements
err = zeros(3,N);
cmd1 = zeros(3,N);
cmd2 = zeros(3,N);

for i=1:N
    cmd1(:,i) = cmd(:,i);
    err(:,i) = tip(1:3,i) - target; 
    if key(i) ~= 0 % key hit: Update Jacobian and calculate new Z_hat
        %Calculate cmd
        Jc = J{i}(1:3,:);
        cmd2(:,i) = calculate_err(Jc, tip(1:3,i) - target, base(:,i));
    else
        cmd2(:,i) = cmd2(:,i-1);
    end
end


%% Plots
% Control output
figure(2);
plot(t, cmd1(1,:), '.-', t, cmd2(1,:), '.-')
hold on
plot(t, cmd1(3,:), '.-', t, cmd2(3,:), '.-')
plot(t, Z(1,:),'.-', t, Z(3,:), '.-')
plot_baseline('--k');
plot_safe_limit('k');
plot_key('--g');
title('Control output'),xlabel('time [s]'),ylabel('cmd [mm]'),  legend('X exp', 'X sim','Z exp','Z sim', 'robot init')

function cmd = calculate_err(Jc, err, base)
    global safe_limit;
    global base_init;
    global K;
    cmd = base + K*pinv(Jc)*err;
    % Include saturation from safe_limit
    cmd(1) = min(cmd(1), base_init(1)+safe_limit);
    cmd(1) = max(cmd(1), base_init(1)-safe_limit);
    cmd(3) = min(cmd(3), base_init(3)+safe_limit);
    cmd(3) = max(cmd(3), base_init(3)-safe_limit);  
end

function plot_target(line)
    global target;
    yline(target(1), line)
    yline(target(3), line)
end

function plot_baseline(line)
    global base_init;
    yline(base_init(1), line)
    yline(base_init(3), line)
end

function plot_safe_limit(line)
    global base_init;
    global safe_limit;
    yline(base_init(1)+safe_limit,line)
    yline(base_init(1)-safe_limit,line)
    yline(base_init(3)+safe_limit,line) 
    yline(base_init(3)-safe_limit,line)
end

function plot_key(line)
    global T
    for i=1:length(T)
        xline(T(i), line);
    end
end