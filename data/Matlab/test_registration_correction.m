
% Verify if doulbe registration fixed systematic error

clear all;
close all;
clc;

%% Choose testing set 
DATASET = 2;            % Choose between 1(20mm) or 2 (100mm)

if (DATASET==1)
    name = 'test_same_position_';
    horizontal = [0, 5, 10, 15, 20, 30];    %Horizontal values for each trial in the testing set (X)
    depth = 20;                             %Depth position for all trials in the testing set (Y)    
    %Obtained from dataset 2
    reg_2 = [-103.603620563402;-139.232145977963;159.383298035792;0.994801208901106;0.101664242606796;0.00591002761965436;9.00978356635246e-05];
else
    name = 'new_test_same_position_';
    horizontal = [5, 10, 15, 20, 25, 30];   %Horizontal values for each trial in the testing set (X)
    depth = 100;                            %Depth position for all trials in the testing set (Y)    
    %Obtained from dataset 1
    reg_2 = [-103.859846689018;-24.9244976679864;244.423606185821;0.999990541686985;-0.00250130480472793;0;0.00355809089889465];
end
vertical = 0;                               %Vertical position for all trials in the testing set (Z)

% Initial registration matrix (from matlab registration procedure - with different insertion depths)
reg = [-108.068583185999;-261.481863275764;-2.76831580891808;-0.714003121489975;-0.700040075101114;0.0118511171421545;0.00172822957967015];


%% Get mean values for each different horizontal position
Zm = zeros(length(horizontal),3);    %Mean values for each horizontal position (aurora frame)
for k=1:length(horizontal)
    
    %% Get .csv file into table T
    filename = strcat(name, int2str(horizontal(k)), 'mm_X_', int2str(depth), 'mm_depth');

    % Read data from table
    T = readtable(strcat(filename,'.csv'), 'HeaderLines',1);

    %% Extract values from table
    check = ne(T{:,17},0) & ne(T{:,18},0) & ne(T{:,19},0);
    n = height(T);                          % last sample   
    i = find(check(1:n), 1, 'first');       % first sample
    
    Z = T{i:n, 8:10};            % Aurora data (not filtered and in Aurora coordinates)
    Zm(k,:) = mean(Z,1);     % Mean point of Aurora data
    
end

%% Register from Aurora to robot with new transform
% Get aurora raw values and register to robot frame
pose = zeros(7,length(horizontal));
pose_2 = zeros(7,length(horizontal));
for k=1:length(horizontal)
    pose(:,k) = pose_transform([Zm(k,:) 1 0 0 0]', reg);    
    pose_2(:,k) = pose_transform([Zm(k,:) 1 0 0 0]', reg_2);    
end
Pm = pose(1:3,:)';    % Mean point registered to robot frame
Pm2 = pose_2(1:3,:)';    % Mean point registered to robot frame

Pe = [-horizontal' repmat(-depth,length(horizontal),1) repmat(vertical,length(horizontal),1)]; %Expected points

%% Plot results
figure(1);
plot3(Pe(:,1), Pe(:,2), Pe(:,3), 'o-b');
hold on;
grid on;
title(strcat('Registration correction (', int2str(depth), 'mm depth)'))
xlabel('X'); 
ylabel('Y');
zlabel('Z');
plot3(Pm(:,1), Pm(:,2), Pm(:,3), 'o-r');
plot3(Pm2(:,1), Pm2(:,2), Pm2(:,3), 'o-g');
legend({'expected', 'With 1st registration','With 2nd registration'})
view(90,0)

%% Registration errors
errX = sqrt((Pm(:,1)-Pe(:,1)).^2);
errY = sqrt((Pm(:,2)-Pe(:,2)).^2);
errZ = sqrt((Pm(:,3)-Pe(:,3)).^2);    
fprintf('First registration error\n')
fprintf('errX = %f +- %f  -  max = %f\n', mean(errX), std(errX), max(errX));
fprintf('errY = %f +- %f  -  max = %f\n', mean(errY), std(errY), max(errY));
fprintf('errZ = %f +- %f  -  max = %f\n', mean(errZ), std(errZ), max(errZ));

errX = sqrt((Pm2(:,1)-Pe(:,1)).^2);
errY = sqrt((Pm2(:,2)-Pe(:,2)).^2);
errZ = sqrt((Pm2(:,3)-Pe(:,3)).^2);    
fprintf('Second registration error\n')
fprintf('errX = %f +- %f  -  max = %f\n', mean(errX), std(errX), max(errX));
fprintf('errY = %f +- %f  -  max = %f\n', mean(errY), std(errY), max(errY));
fprintf('errZ = %f +- %f  -  max = %f\n', mean(errZ), std(errZ), max(errZ));

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
