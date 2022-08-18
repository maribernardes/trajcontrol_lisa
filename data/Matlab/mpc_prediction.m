clear; close all; clc;
global target;
global safe_limit;
global base_init;

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

past_u = [X_step(1,1) X_step(1,2)];
past_y = Z_step(1,1:3);

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

    past_u = [past_u; up{i}(1,:)];
    past_y = [past_y; yp{i}(1,:)];
    
    subplot(4,1,1);
    plot(past_y(:,2), past_y(:,1), '.-m', yp{i}(1:N-i+1,2),yp{i}(1:N-i+1,1), '.-b');
    plot_target_X('--r');
    title('X - Horizontal');
    ylabel('Tip X [mm]'); legend('executed', 'prediction', 'target')
    set(gca,'Xdir','reverse');
    xlim([-100 0]);
    
    subplot(4,1,2);
    plot(past_y(:,2), past_u(:,1), '.-m', yp{i}(1:N-i+1,2),up{i}(1:N-i+1,1), '.-b');
    plot_safe_limit_X('k');
    ylabel('Base X [mm]'); legend('executed', 'prediction', 'safe limit')
    set(gca,'Xdir','reverse');
    xlim([-100 0]);

    
    subplot(4,1,3);
    plot(past_y(:,2), past_y(:,3), '.-m',yp{i}(1:N-i+1,2),yp{i}(1:N-i+1,3), '.-b');
    plot_target_Z('--r');
    title('Z - Vertical');
    ylabel('Tip Z [mm]'); legend('executed', 'prediction', 'target')
    set(gca,'Xdir','reverse');
    xlim([-100 0]);
    
    subplot(4,1,4);
    plot(past_y(:,2), past_u(:,2), '.-m', yp{i}(1:N-i+1,2),up{i}(1:N-i+1,2), '.-');
    plot_safe_limit_Z('k');
    xlabel('Depth [mm]'),ylabel('Base Z [mm]'); legend('executed', 'prediction', 'safe limit')
    set(gca,'Xdir','reverse');
    xlim([-100 0]);

    pause;
    
end

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