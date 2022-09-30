%% EXPERIMENT:
% We manually placed the robot at different horizontal (X) positions while
% keeping Z and Y constant. For each position, we acquired many Aurora
% readings using ROS2_bridge and saved to .csv file
% THIS CODE: reads the values for each position and get a mean value as the
% most likely tip position. Then, it registers them to the robot frame using
% the registration transform from previous experiments and compares to
% expected values (Fig 1). Since the calculations showed some systematic
% error, we calculated a second tranformation to further correction

clear all;
close all;
clc;

%% Choose testing set 
DATASET = 1;            % Choose between 1(20mm) or 2 (100mm)

if (DATASET==1)
    name = 'test_same_position_';
    horizontal = [0, 5, 10, 15, 20, 30];    %Horizontal values for each trial in the testing set (X)
    depth = 20;                             %Depth position for all trials in the testing set (Y)
else
    name = 'new_test_same_position_';
    horizontal = [5, 10, 15, 20, 25, 30];   %Horizontal values for each trial in the testing set (X)
    depth = 100;                            %Depth position for all trials in the testing set (Y)    
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
    
    Z = T{i:n, 8:10};        % Aurora data (not filtered and in Aurora coordinates)
    Zm(k,:) = mean(Z,1);     % Mean point of Aurora data 
end

%% Register from Aurora to robot with original transform
pose = zeros(7,length(horizontal));
for k=1:length(horizontal)
    pose(:,k) = pose_transform([Zm(k,:) 1 0 0 0]', reg);    
end
Pm = pose(1:3,:)';    % Mean point registered to robot frame

Pe = [-horizontal' repmat(-depth,length(horizontal),1) repmat(vertical,length(horizontal),1)]; %Expected points

%% Plot tip measurements
figure(1)
plot3(Pm(:,1), Pm(:,2), Pm(:,3), 'o-r');
hold on;
grid on;
title(strcat('Experimental data (', int2str(depth), 'mm depth) after original registration'))
xlabel('X'); 
ylabel('Y');
zlabel('Z');
view(150,30)
plot3(Pe(:,1), Pe(:,2), Pe(:,3), 'o-b');
legend({'recorded points', 'expected points'})

%% Current registration error
errX = sqrt((Pm(:,1)-Pe(:,1)).^2);
errY = sqrt((Pm(:,2)-Pe(:,2)).^2);
errZ = sqrt((Pm(:,3)-Pe(:,3)).^2);    
fprintf('First registration error\n')
fprintf('errX = %f +- %f  -  max = %f\n', mean(errX), std(errX), max(errX));
fprintf('errY = %f +- %f  -  max = %f\n', mean(errY), std(errY), max(errY));
fprintf('errZ = %f +- %f  -  max = %f\n', mean(errZ), std(errZ), max(errZ));

%% Calculate corrected registration transform
%Aurora points
A_orig = Zm';
%Robot points
Pe2 = Pe;
% Pe2(:,1) = Pm(:,1); %Disconsider error in x
B_orig = Pe2';

%Mean Center Data
Ac = mean(A_orig,2);
Bc = mean(B_orig,2);
A = A_orig-repmat(Ac,1,size(A_orig,2));
B = B_orig-repmat(Bc,1,size(B_orig,2));

%Calculate Optimal Rotation
M = A*B';
N = [M(1,1)+M(2,2)+M(3,3)  M(2,3)-M(3,2)          M(3,1)-M(1,3)          M(1,2)-M(2,1);...
     M(2,3)-M(3,2)         M(1,1)-M(2,2)-M(3,3)   M(1,2)+M(2,1)          M(3,1)+M(1,3);...
     M(3,1)-M(1,3)         M(1,2)+M(2,1)         -M(1,1)+M(2,2)-M(3,3)   M(2,3)+M(3,2);...
     M(1,2)-M(2,1)         M(3,1)+M(1,3)          M(2,3)+M(3,2)         -M(1,1)-M(2,2)+M(3,3)];
[u,v] = eig(N);
[m,ind] = max(diag(v));
q = u(:,ind);
R = (q(1)^2-q(2:4)'*q(2:4))*eye(3) + 2*q(2:4)*q(2:4)'+2*q(1)*[0 -q(4) q(3); q(3) 0 -q(2); -q(3) q(2) 0];

%Calculate Optimal Translation
t = Bc - R*Ac;
reg_2 = [t;q];
csvwrite('registration_matlab_2.csv',reg_2)

%% Transform points with new reg_2
pose_2 = zeros(7,length(horizontal));
for k=1:length(horizontal)
    pose_2(:,k) = pose_transform([Zm(k,:) 1 0 0 0]', reg_2);    
end
Pm2 = pose_2(1:3,:)';    % Mean point registered to robot frame

figure(2);
plot3(Pe(:,1), Pe(:,2), Pe(:,3), 'o-b');
hold on;
grid on;
title(strcat('Result of extra registration (', int2str(depth), 'mm depth)'))
xlabel('X'); 
ylabel('Y');
zlabel('Z');
plot3(Pm(:,1), Pm(:,2), Pm(:,3), 'o-r');
plot3(Pm2(:,1), Pm2(:,2), Pm2(:,3), 'o-g');
legend({'expected', 'With 1st registration','With extra registration'})
view(90,0)

%% Final error
errX = sqrt((Pm2(1,:)-Pe(1,:)).^2);
errY = sqrt((Pm2(2,:)-Pe(2,:)).^2);
errZ = sqrt((Pm2(3,:)-Pe(3,:)).^2);
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