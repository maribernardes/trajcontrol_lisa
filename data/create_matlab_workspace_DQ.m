clear; close all; clc;
include_namespace_dq;

%% Get .csv file into table T
% Filename
folder = '2022-07-20';
name = 'trialb_04';

%% Load Dataset
% Read data from table
T = readtable(strcat(folder,'/',name,'.csv'), 'HeaderLines',1);

% Define sample sizes
i = find(T{:,3}, 1,'first');   % First sample (when "target" is published)
j = height(T);                 % Last sample

%% Extract values from table
% Timestamp
sec = T{i:j,1};
nanosec = T{i:j,2};

% Aurora readings from igtl_bridge
auroraTip = T{i:j,13:15};                 %Needle Tip (5DOF)
auroraqTip = T{i:j,16:19};

% Published topics 
% Tip = '/sensor/tip_filtered'
% Base = '/stage/state/needle_pose'
Tip = T{i:j,22:24};     %Needle Tip (5DOF)
qTip = T{i:j,25:28};
Base = T{i:j,40:42};    %[StageX DepthY StageZ]
base_init = T{i-1,40:42};

% Jacobian (by lines)
J1 = T{i:j,49:51};
J2 = T{i:j,52:54};
J3 = T{i:j,55:57};
J4 = T{i:j,58:60};
J5 = T{i:j,61:63};

% Control outputs
cmd = T{i:j,66:68}';
% Target
target = T{i,3:5}';

%% Define variables to be saved in .mat
% Total number of samples
k = length(Tip);

%Time vector
t = zeros(1,k);
% Aurora sensor values (in Dual Quaternions)
x_needletip(1,k) = DQ();
% Aurora sensor values (in vector format [x, y, z, qw, qx, qy, qz])
aurora_tip = zeros(7,k);
% Sensor values in stage frame - published by ros2 topics (in vector format)
tip = zeros(7,k);
base = zeros(3,k);
% Sensor values in stage frame - obtained with transform in matlab (in vector format)
X = zeros(3,k);
Z = zeros(5,k);
% Jacobian matrix
J = cell(1,k);

%% Other initializations
% Aurora to robot registration matrix
reg = [-4.508352564583567812e+01;
1.895709482583472436e+02;
5.193024919983328402e+01;
-7.026693933008296700e-01;
7.114455521878115807e-01;
9.762195488940694776e-03;
2.376873925854971949e-03];

initcmd = find(cmd, 1,'first');
for i=1:initcmd-1
    cmd(:,i) = base(1:3,i); 
end

for i=1:k
    %Set time vector in seconds
    t(i) = sec(i)-sec(1) + (nanosec(i)-nanosec(1))*1e-9;
    
    %Force unit norm (due to decimal place rounding from Aurora)
    r_needletip = DQ(qTip(i,:)/norm(qTip(i,:)));
    
    %Translation is provided in mm by Aurora
    p_needletip = DQ(Tip(i,:));
    x_needletip(:,i) = r_needletip + E_ * 0.5 * p_needletip * r_needletip;
    aurora_tip(:,i) = [auroraTip(i,:) auroraqTip(i,:)]';

    tip(:,i) = [Tip(i,:) qTip(i,:)]';
    base(:,i) = Base(i,:)';
    
    X(:,i) = Base(i,:)';
    [horiz, vert]= get_needle_angles(qTip(i,:));
    Z(:,i) = [Tip(i,:), horiz, vert];
    
    J{i} = [J1(i,:); J2(i,:); J3(i,:); J4(i,:); J5(i,:)];
    
end

key = [0; diff(base(2,:))']/-5;

save(strcat('Matlab/',folder,'/',name,'.mat'),'x_needletip','tip','base','aurora_tip','t', 'X', 'Z', 'J', 'target', 'base_init', 'cmd', 'key')

function [horizontal, vertical]= get_needle_angles(quat)
    q = quaternion(quat);
    uz = quaternion(0, 0, 0, 1);
    v = compact(q*uz*q');

    horizontal = atan2(v(2),-v(3));                 % needle points in -Y direction
    vertical = atan2(v(4), sqrt(v(2)^2 + v(3)^2));
end

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
