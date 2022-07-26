%With X = [x_base; y_base; z_base] and Z = [x_tip; y_tip; z_tip; horizangle_tip vertiangle_tip]\n');

clear; close all; clc;
global safe_limit;
global base_init;
global target;
global T;
global K;

%% Load Dataset
trial = 02;
folder = '2022-07-20';
name = 'checkcmd_';
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
cmd3 = zeros(3,N);
cmd4 = zeros(3,N);
cmd5 = zeros(3,N);
cmd6 = zeros(3,N);
cmd1(:,1) = cmd(:,1);
cmd2(:,1) = cmd(:,1);
cmd3(:,1) = cmd(:,1);
cmd4(:,1) = cmd(:,1);
cmd5(:,1) = cmd(:,1);
cmd6(:,1) = cmd(:,1);
for i=1:N
    cmd1(:,i) = cmd(:,i);
    err(:,i) = tip(1:3,i) - target; 
    if key(i) == 1 % key hit: Update Jacobian and calculate new Z_hat
        %Calculate cmd
        Jc = J{i-2}(1:3,:);
        cmd2(:,i) = calculate_err(Jc, tip(1:3,i) - target, base(:,i));
        cmd3(:,i) = calculate_err(Jc, tip(1:3,i-1) - target, base(:,i));
        cmd4(:,i) = calculate_err(Jc, tip(1:3,i-2) - target, base(:,i));
        cmd5(:,i) = calculate_err(Jc, tip(1:3,i+1) - target, base(:,i));
        cmd6(:,i) = calculate_err(Jc, tip(1:3,i+2) - target, base(:,i));
    else
        if (i>1)
            cmd2(:,i) = cmd2(:,i-1);
            cmd3(:,i) = cmd3(:,i-1);
            cmd4(:,i) = cmd4(:,i-1);
            cmd5(:,i) = cmd5(:,i-1);
            cmd6(:,i) = cmd6(:,i-1);
        end
    end
end


%% Plot control error to target
figure(1);
plot(t, err(1,:),'.-', t, err(2,:),'.-', t, err(3,:),'.-')
hold on;
plot_key('--g');
title('Error to target'),xlabel('time [s]'),ylabel('err [mm]'),  legend('X', 'Y', 'Z')

% Control output
figure(2);
plot(t, cmd1(1,:), '.-', t, cmd2(1,:), '.-')
hold on
plot(t, cmd1(3,:), '.-', t, cmd2(3,:), '.-')
plot(t, Z(1,:),'.-', t, Z(3,:), '.-')
plot_baseline('--k');
plot_safe_limit('k');
plot_key('--g');
title('Control output'),xlabel('time [s]'),ylabel('cmd [mm]'),  legend('X exp', 'X sim','Z exp','Z sim',  'tipX', 'tipZ', 'initial')


% Control sensitivity
figure(7);
plot(t, cmd1(3,:), '.-')
hold on
plot(t, cmd2(3,:), '.-')
plot(t, cmd3(3,:), '.-')
plot(t, cmd4(3,:), '.-')
plot(t, cmd5(3,:), '.-')
plot(t, cmd6(3,:), '.-')
plot(t, Z(3,:),'.-')
plot_baseline('--k');
plot_safe_limit('k');
plot_key('--g');
title('Control sensitivity'),xlabel('time [s]'),ylabel('cmd [mm]'),  legend('Z exp', 'Z sim1','Z sim2', 'Z sim3','Z sim4','Z sim5', 'tipX', 'initial')

% Tip and target
figure(3);
plot(t, Z(1,:),'.-', t, Z(3,:), '.-')
hold on
plot_target('--r')
plot_key('--g');
title('Tip vs Target'),xlabel('time [s]'),ylabel('X [mm]'),   legend('X','Z')

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