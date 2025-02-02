clear all, clc
addpath('functions')

%% Inputs
% Define global variables
global N na kd kp n1 m g e3 Tsim w Q v

% Quadrotor parameters
g = 9.81;
m(1) = 0.42;
m(2) = 0.42;
m(3) = 0.42;
m(4) = 0.42;
    
% Unit vector along z
e3 = [0; 0; 1];

% Define graph
N{1} = [];
N{2} = [1]; % Cannot be changed
N{3} = [2];
N{4} = [3];

na = length(N); % Number of agents

% Control gain
kp  = 1.9*ones(na,1); % 0.5
kd  = 3*ones(na,1); % 1.2
n1  = 20;

% Frequency
w = .5; % [rad/s]
v = 0.4;

check_gains(kd, kp, N, na);

Tsim = 80;
wbar = waitbar(0, 'Simulating system...');

%% First motion

% Desired initial positions
Pdes0(:,:,1) = [                         0; v*0;                          0];
Pdes0(:,:,2) = [       (3-4/Tsim*0)*sin(0); v*0;        (3-4/Tsim*0)*cos(0)];
Pdes0(:,:,3) = [(3-4/Tsim*0)*sin(0-2*pi/3); v*0; (3-4/Tsim*0)*cos(0-2*pi/3)];
Pdes0(:,:,4) = [(3-4/Tsim*0)*sin(0-4*pi/3); v*0; (3-4/Tsim*0)*cos(0-4*pi/3)];

% Intitial positions
rng('default') % dont let random be too random
P_0 = 3*(rand(3,1,na) - 0.5);
P_0(:,1,1) = Pdes0(:,:,1);
P_0(2,1,2) = 0;
P_0(2,1,3) = 0;
P_0(2,1,4) = 0;

% Initial velocities
V_0 = 2*(rand(3,1,na) - 0.5);
V_0(:,1,1) = [0; v; 0];
V_0(2,1,2) = v;
V_0(2,1,3) = v;
V_0(2,1,4) = v;

% Intial rotations
Rinit = eye(3);
R_0 = zeros(3,3,na);
for i = 1:na
    R_0(:,:,i) = Rinit;
end

Q = 1;
[t1, x1] = ode45('dv_BPE_Rescaling', [0, Tsim/2], [P_0(:); V_0(:); R_0(:); Pdes0(:)]);

Pdes0(:,:,1) = [                                      0; v*Tsim/2;                                       0];
Pdes0(:,:,2) = [       (-1+4/Tsim*Tsim/2)*sin(w*Tsim/2); v*Tsim/2;        (-1+4/Tsim*Tsim/2)*cos(w*Tsim/2)];
Pdes0(:,:,3) = [(-1+4/Tsim*Tsim/2)*sin(w*Tsim/2-2*pi/3); v*Tsim/2; (-1+4/Tsim*Tsim/2)*cos(w*Tsim/2-2*pi/3)];
Pdes0(:,:,4) = [(-1+4/Tsim*Tsim/2)*sin(w*Tsim/2-4*pi/3); v*Tsim/2; (-1+4/Tsim*Tsim/2)*cos(w*Tsim/2-4*pi/3)];
Q = 2;
[t2, x2] = ode45('dv_BPE_Rescaling', [Tsim/2, Tsim], [x1(end,1:end-3*na)'; Pdes0(:)]);
close(wbar);

%%
close all
x = [x1; x2];
t = [t1; t2];
i1 = length(x1);
visualize_rescaling(x, t, N, na, false, '',' formation with changing shapes', -151.2478, 10.4676, i1)