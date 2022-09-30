%% Visualization of experiments with controller_sequence: sequence of robot points
% Plots needle tip trajectories at a given insertion depth
% Plots blobs of tip measured positions after robot reaches a given point
% Plots the last tip position before new point is set to robot
% Plots the mean tip position for each blob

clear all;
close all;
clc;
             
%% Load Dataset
depth = [00, 20, 40, 60, 80, 100];
extra = '';
folder = 'aurora';
name = 'depth_';

% Aurora to robot registration matrix
reg = [-108.068583185999;-261.481863275764;-2.76831580891808;-0.714003121489975;-0.700040075101114;0.0118511171421545;0.00172822957967015];

%% Desired frame
frame = 'stage';  % stage / aurora

depthcolors = ['r';'g'];
cmapd = repmat(depthcolors,ceil(length(depth)/2),1); % Make colors.

%% Load data
for d=1:length(depth)
    
    load(strcat(folder,'/',name,num2str(depth(d),'%2.2d'),extra,'.mat'));

    % Total number of samples
    n = size(tip,2);          

    %% Loop key
    % Indexes when new cmd was sent
    new_cmd = ([-5; diff(cmd(1,:))']/-5) | ([-5; diff(cmd(3,:))']/-5);
    k_key = find(new_cmd); % Samples when cmd was changed
    ns = length(k_key);
    aurora_disp = zeros(3,ns);


    %% Initialize figures
    switch frame
        case 'aurora'
            dummy(:,1) = aurora_tip(1:3,1)';   
        case 'stage'
            dummy(:,1) = tip(1:3,1)';   
    end    

    blobcolors = 3;
    cmap = repmat(jet(blobcolors),ceil(ns/blobcolors),1); % Make colors.

    figure(1)
    title(['Experimental 3D data - ' frame ' frame'])
    plot3(dummy(1,1),dummy(2,1),dummy(3,1), '.w');
    hold on;
    xlabel('X'); 
    ylabel('Y');
    zlabel('Z');
    box on;

    figure(2)
    title(['Experimental 2D data - ' frame ' frame'])
    switch frame
        case 'aurora'
            plot(dummy(1,1),dummy(2,1), '.w');
            ylabel('Y'); 
        case 'stage'
            plot(dummy(1,1),dummy(3,1), '.w');
            ylabel('Z'); 
    end    
    hold on;
    xlabel('X'); 
    box on;

    %% Each designated point
    for k=1:ns
        a = k_key(k);
        if k==ns
            b = n;
        else
            b = k_key(k+1)-1;
        end

        P = zeros(3,a-b+1);
        Paur = zeros(3,a-b+1);
        blobP = [];
        blobPaur = [];

        for i=a:b
            Paur(:,i-a+1) = aurora_tip(1:3,i);
            P(:,i-a+1) = tip(1:3,i);
            stage_err = norm((stage(:,i)-[cmd(1,i);cmd(3,i)]));
            if (stage_err<1.0e-10)
                blobP = [blobP, P(:,i-a+1)];
                blobPaur = [blobPaur, Paur(:,i-a+1)];
            end
        end
        disp(:,k) = [std(blobP(1,:)); std(blobP(2,:)); std(blobP(3,:))];
        dispaur(:,k) = [std(blobPaur(1,:)); std(blobPaur(2,:)); std(blobPaur(3,:))];

    %% Plot tip measurements (clusters of points)
        switch frame
            case 'stage'
                figure(1)
                plot3(P(1,:), P(2,:), P(3,:), cmapd(d));
                plot3(P(1,i-a+1), P(2,i-a+1), P(3,i-a+1), 'ob');      
                plot3(mean(blobP(1,:)), mean(blobP(2,:)), mean(blobP(3,:)), '*b');    
                scatter3(blobP(1,:), blobP(2,:), blobP(3,:), 10, cmap(k,:), 'filled')

                figure(2)
                plot(P(1,:), P(3,:), cmapd(d));
                plot(P(1,i-a+1), P(3,i-a+1), 'ob');      
                plot(mean(blobP(1,:)), mean(blobP(3,:)), '*b');  
                scatter(blobP(1,:), blobP(3,:), 10, cmap(k,:), 'filled')
            case 'aurora'
                figure(1)
                plot3(Paur(1,:), Paur(2,:), Paur(3,:), cmapd(d));
                plot3(Paur(1,i-a+1), Paur(2,i-a+1), Paur(3,i-a+1), 'ob');      
                plot3(mean(blobPaur(1,:)), mean(blobPaur(2,:)), mean(blobPaur(3,:)), '*b');    
                scatter3(blobPaur(1,:), blobPaur(2,:), blobPaur(3,:), 10, cmap(k,:), 'filled')

                figure(2)
                plot(Paur(1,:), Paur(2,:), cmapd(d));
                plot(Paur(1,i-a+1), Paur(2,i-a+1), 'ob');      
                plot(mean(blobPaur(1,:)), mean(blobPaur(2,:)), '*b');  
                scatter(blobPaur(1,:), blobPaur(2,:), 10, cmap(k,:), 'filled')

        end


    end

end
    figure(1)
    legend({'tip trajectory','tip at final position','tip mean position'});

    fprintf('Dispersion X [mm] = %0.2f +- %0.2f\n', mean(disp(1,:)), std(disp(1,:)));
    fprintf('Dispersion Y [mm] = %0.2f +- %0.2f\n', mean(disp(2,:)), std(disp(2,:)));
    fprintf('Dispersion Z [mm] = %0.2f +- %0.2f\n', mean(disp(3,:)), std(disp(3,:)));


    fprintf('Dispersion X [mm] = %0.2f +- %0.2f\n', mean(disp(1,:)), std(disp(1,:)));
    fprintf('Dispersion Y [mm] = %0.2f +- %0.2f\n', mean(disp(2,:)), std(disp(2,:)));
    fprintf('Dispersion Z [mm] = %0.2f +- %0.2f\n', mean(disp(3,:)), std(disp(3,:)));

