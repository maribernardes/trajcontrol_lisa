clear; close all; clc;

K = 0.05;
safe_limit = 6;

tip = [-63.972777914311	-5.44769585935768	27.0974993764381]';
target = [-63.8104888439106	-100	27.1129301727601]';
base_init = [-62.92875	0	28.20625]';
base = [-62.92875	-5	28.20625]';

Jinit = [0.379984946889211	-0.005465427929897	-0.177065215615603;	
    -0.064474115801993	1.01486337600203	0.153897446353083;	
    0.046506906709345	-0.03027478365847	0.576795717367411;	
    -0.011020244312185	-0.002140440469812	0.00729270157317;	
    -0.004193352971666	-0.003021861581593	-0.008709163839286];

J = Jinit;
Jc = J(1:3,:);

err = tip-target
deltaU = K*pinv(Jc)*err
cmd = base + deltaU;

% Include saturation from safe_limit
cmd(1) = min(cmd(1), base_init(1)+safe_limit);
cmd(1) = max(cmd(1), base_init(1)-safe_limit);
cmd(3) = min(cmd(3), base_init(3)+safe_limit);
cmd(3) = max(cmd(3), base_init(3)-safe_limit);  

cmd
