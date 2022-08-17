clear; close all; clc;
global target;
global safe_limit;
global base_init;

%% Load Dataset
trial = 00;
folder = 'tests';
name = 'test_';
load(strcat(folder,'/',name,num2str(trial,'%2.2d'),'.mat'));

%% Configure simulationb
N = size(up,2);  % data size
safe_limit = 6;
base_init = base(:,1);

%% Loop key
for i=1:N    
    
    subplot(4,1,1)
    plot(yp{i}(1:N-i+1,2),yp{i}(1:N-i+1,1), '.-')
    plot_target_X('--r')
    title('Tip prediction')
    ylabel('Tip X [mm]')
    set(gca,'Xdir','reverse')
    xlim([-100 0])
    
    subplot(4,1,2)
    plot(yp{i}(1:N-i+1,2),yp{i}(1:N-i+1,3), '.-')
    plot_target_Z('--r')
    ylabel('Tip Z [mm]')
    set(gca,'Xdir','reverse')
    xlim([-100 0])
    
    subplot(4,1,3)
    plot(yp{i}(1:N-i+1,2),up{i}(1:N-i+1,1), '.-')
    plot_safe_limit_X('k');
    title('Control prediction')
    ylabel('base X [mm]')
    set(gca,'Xdir','reverse')
    xlim([-100 0])

    subplot(4,1,4)
    plot(yp{i}(1:N-i+1,2),up{i}(1:N-i+1,2), '.-')
    plot_safe_limit_Z('k');
    xlabel('Depth [mm]'),ylabel('base Z [mm]')
    set(gca,'Xdir','reverse')
    xlim([-100 0])

    pause;
    
end

function plot_target(line)
    global target;
    plot_target_X(line)
    plot_target_Z(line)
end

function plot_target_X(line)
    global target;
    yline(target(1), line)
end

function plot_target_Z(line)
    global target;
    yline(target(3), line)
end

function plot_safe_limit(line)
    global base_init;
    global safe_limit;
    plot_safe_limit_X(line)
    plot_safe_limit_Z(line)
end

function plot_safe_limit_X(line)
    global base_init;
    global safe_limit;
    yline(base_init(1)+safe_limit,line)
    yline(base_init(1)-safe_limit,line)
end

function plot_safe_limit_Z(line)
    global base_init;
    global safe_limit;
    yline(base_init(3)+safe_limit,line) 
    yline(base_init(3)-safe_limit,line)
end