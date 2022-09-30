clear all;
close all;
clc;
             
%% Load Dataset
depth = 00;
extra = '';
folder = 'aurora';
name = 'depth_';
load(strcat(folder,'/',name,num2str(depth,'%2.2d'),extra,'.mat'));

%% Desired frame
frame = 'aurora';
% frame = 'stage';

% Aurora to robot registration matrix
reg = [-108.068583185999;-261.481863275764;-2.76831580891808;-0.714003121489975;-0.700040075101114;0.0118511171421545;0.00172822957967015];

% Total number of samples
n = size(tip,2);          

%% Loop key
% Indexes when new cmd was sent
new_cmd = ([-5; diff(cmd(1,:))']/-5) | ([-5; diff(cmd(3,:))']/-5);
k_key = find(new_cmd); % Samples when cmd was changed
ns = length(k_key);
aurora_disp = zeros(3,ns);


% Initialize figures
figure(1)
switch frame
    case 'aurora'
        dummy(:,1) = tip(1:3,1)';   
        title('Experimental data - Aurora frame')        
    case 'stage'
        dummy(:,1) = tip(1:3,1)';   
        title('Experimental data - Robot frame')
    case 'stagetf'
        dummy(:,1) = pose_transform(aurora_tip(:,1)', reg);    
        title('Experimental data - Robot tf frame')
end    
plot3(dummy(1,1),dummy(2,1),dummy(3,1), '.w');
hold on;
xlabel('X'); 
ylabel('Y');
zlabel('Z');
box on;
colors = 3;
cmap = repmat(jet(colors),ceil(ns/colors),1); % Make colors.

% Each designated point
for k=1:ns
    a = k_key(k);
    if k==ns
        b = n;
    else
        b = k_key(k+1)-1;
    end
    
    P = zeros(3,a-b+1);
    Ptf = zeros(3,a-b+1);
    Zaurora = zeros(7,a-b+1);
    Ztf = zeros(7,a-b+1);
    blobP = [];
    blobPtf = [];
    
    for i=a:b
        Paur(:,i-a+1) = aurora_tip(1:3,i);
        P(:,i-a+1) = tip(1:3,i);
        Zaurora(:,i-a+1) = aurora_tip(:,i);
        Ztf(:,i-a+1) = pose_transform(Zaurora(:,i-a+1)', reg);    
        Ptf(:,i-a+1) = Ztf(1:3,i-a+1);
        stage_err = norm((stage(:,i)-[cmd(1,i);cmd(3,i)]));
        if (stage_err<1.0e-10)
            blobP = [blobP, P(:,i-a+1)];
            blobPtf = [blobPtf, Ptf(:,i-a+1)];
        end
    end
    disp(:,k) = [std(blobP(1,:)); std(blobP(2,:)); std(blobP(3,:))];
    disptf(:,k) = [std(blobPtf(1,:)); std(blobPtf(2,:)); std(blobPtf(3,:))];

    %% Plot tip measurements (clusters of points)
    figure(1)
    switch frame
        case 'aurora'
            plot3(P(1,:), P(2,:), P(3,:), 'r');
            plot3(P(1,i-a+1), P(2,i-a+1), P(3,i-a+1), 'ob');      
            plot3(mean(blobP(1,:)), mean(blobP(2,:)), mean(blobP(3,:)), '*b');    
            scatter3(blobP(1,:), blobP(2,:), blobP(3,:), 10, cmap(k,:), 'filled')
        case 'stage'
            plot3(Ptf(1,:), Ptf(2,:), Ptf(3,:), 'r');
            plot3(Ptf(1,i-a+1), Ptf(2,i-a+1), Ptf(3,i-a+1), 'ob');      
            plot3(mean(blobPtf(1,:)), mean(blobPtf(2,:)), mean(blobPtf(3,:)), '*b');    
            scatter3(blobPtf(1,:), blobPtf(2,:), blobPtf(3,:), 10, cmap(k,:), 'filled')
    end
end
figure(1)
legend({'tip trajectory','tip at final position','tip mean position'});

function x_new = pose_transform(x_orig, x_tf)
    %Define frame transformation
    p_tf = quaternion(0, x_tf(1), x_tf(2), x_tf(3));
    q_tf= quaternion(x_tf(4), x_tf(5), x_tf(6), x_tf(7));

    %Define original position and orientation
    p_orig = quaternion(0, x_orig(1), x_orig(2), x_orig(3));
    q_orig = quaternion(x_orig(4), x_orig(5), x_orig(6), x_orig(7));

    %Transform to new frame
    q_new = q_tf*q_orig;
    p_new = q_tf*p_orig*conj(q_tf) + p_tf;

    vec_new = compact(p_new);
    x_new = [vec_new(2:4) compact(q_new)]';
end
