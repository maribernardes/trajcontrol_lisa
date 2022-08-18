clear; close all; clc;
global target;
global safe_limit;
global base_init;

INSERTION_STEP = -5;

%% Load Dataset
trial = 02;
folder = '2022-08-17';
name = 'trialh_';
load(strcat(folder,'/',name,num2str(trial,'%2.2d'),'.mat'));

%% Configure simulationb
N = size(up,2);  % data size
safe_limit = 6;
base_init = base(:,1);

%% Loop key
k_key = find(key); % Samples when key was pressed
ns = length(k_key);
T = zeros(1,ns);
for i=1:ns
    k = k_key(i);   %sample
    Jc_step{i} = J{k}(1:3,:);
    Z_step(i,:) = Z(:,k);
    X_step(i,:) = X(:,k);
    cmd_step(i,:) = cmd(:, k+1);
end

past_u = [X_step(1,1) X_step(1,3)];
past_y = Z_step(1,1:3);

future_u = [past_u; up{1}(1:N,:)];
future_y = [past_y; yp{1}(1:N,:)];

depth = [0:-5:-100];

%% Loop prediction steps
for i=1:N    
%     i
%     Jc_step{i}
%     target
%     Z_step(i,1:3)'
%     if i==1
%         delta_u = [up{i}(1,1);-5; up{i}(1,2)] - base_init
%     else
%         delta_u = [up{i}(1,1)-up{i-1}(1,1);-5; up{i}(1,2)-up{i-1}(1,2)]     
%     end
%     delta_tip = Jc_step{i}*delta_u        
    

    subplot(4,1,1);
    plot(depth(1:i), past_y(:,1), '.-m', depth(i:N+1), future_y(:,1), '.-b');
    plot_target_X('--r');
    title('X - Horizontal');
    ylabel('Tip X [mm]'); legend('executed','prediction',  'target');
    set(gca,'Xdir','reverse');
    xlim([-100 0]);
    
    subplot(4,1,2);
    plot(depth(1:i), past_u(:,1), '.-m', depth(i:N+1), future_u(:,1), '.-b');
    plot_safe_limit_X('k');
    ylabel('Base X [mm]'); legend('executed','prediction',  'safe limit');
    set(gca,'Xdir','reverse');
    xlim([-100 0]);

    
    subplot(4,1,3);
    plot(depth(1:i), past_y(:,3), '.-m', depth(i:N+1), future_y(:,3), '.-b');
    plot_target_Z('--r');
    title('Z - Vertical');
    ylabel('Tip Z [mm]'); legend('executed','prediction',  'target');
    set(gca,'Xdir','reverse');
    xlim([-100 0]);
    
    subplot(4,1,4);
    plot(depth(1:i), past_u(:,2), '.-m', depth(i:N+1), future_u(:,2), '.-b');
    plot_safe_limit_Z('k');
    xlabel('Depth [mm]'),ylabel('Base Z [mm]'); legend('executed', 'prediction', 'safe limit');
    set(gca,'Xdir','reverse');
    xlim([-100 0]);

    if (i~=N)        
        future_u = [X_step(i+1,1), X_step(i+1,3); up{i+1}(1:N-i,:)];
        future_y = [Z_step(i+1,1:3); yp{i+1}(1:N-i,:)];
    end
    past_u = [past_u; [X_step(i+1,1) X_step(i+1,3)]];
    past_y = [past_y; Z_step(i+1,1:3)];

    pause;
    
end

subplot(4,1,1);
plot(depth, Z_step(:,1), '.-m');
plot_target_X('--r');
title('X - Horizontal');
ylabel('Tip X [mm]'); legend('executed', 'target');
set(gca,'Xdir','reverse');
xlim([-100 0]);

subplot(4,1,2);
plot(depth, cmd_step(:,1), '.-m');
plot_safe_limit_X('k');
ylabel('Base X [mm]'); legend('executed', 'safe limit');
set(gca,'Xdir','reverse');
xlim([-100 0]);


subplot(4,1,3);
plot(depth, Z_step(:,3), '.-m');
plot_target_Z('--r');
title('Z - Vertical');
ylabel('Tip Z [mm]'); legend('executed', 'target');
set(gca,'Xdir','reverse');
xlim([-100 0]);

subplot(4,1,4);
plot(depth, cmd_step(:,3), '.-m');
plot_safe_limit_Z('k');
xlabel('Depth [mm]'),ylabel('Base Z [mm]'); legend('executed', 'safe limit');
set(gca,'Xdir','reverse');
xlim([-100 0]);


function plot_target(line)
    global target;
    plot_target_X(line);
    plot_target_Z(line);
end

function plot_target_X(line)
    global target;
    yline(target(1), line);
    ylim([target(1)-15, target(1)+15])
end

function plot_target_Z(line)
    global target;
    yline(target(3), line);
    ylim([target(3)-15, target(3)+15])
end

function plot_safe_limit(line)
    global base_init;
    global safe_limit;
    plot_safe_limit_X(line);
    plot_safe_limit_Z(line);
end

function plot_safe_limit_X(line)
    global base_init;
    global safe_limit;
    yline(base_init(1)+safe_limit,line);
    yline(base_init(1)-safe_limit,line);
    ylim([base_init(1)-safe_limit-3, base_init(1)+safe_limit+3])

end

function plot_safe_limit_Z(line)
    global base_init;
    global safe_limit;
    yline(base_init(3)+safe_limit,line);
    yline(base_init(3)-safe_limit,line);
    ylim([base_init(3)-safe_limit-3, base_init(3)+safe_limit+3])
end