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
N{4} = [1,3];

na = length(N); % Number of agents

% Control gain
kp  = 1.9*ones(na,1); % 0.5
kd  = 3*ones(na,1); % 1.2
n1  = 20;

% Frequency
w = 1; % [rad/s]
v = 0.2;

check_gains(kd, kp, N, na);

Tsim = 82;
wbar = waitbar(0, 'Simulating system...');

%% First motion

% Desired initial positions
Pdes0(:,:,1) = [              1; v*0;              1];
Pdes0(:,:,2) = [-.75*sin(w*0)-1; v*0; .75*sin(w*0)+1];
Pdes0(:,:,3) = [             -1; v*0;             -1];
Pdes0(:,:,4) = [              1; v*0;             -1];

% Intitial positions
rng('default') % dont let random be too random
P_0 = Pdes0 + 2*(rand(3,1,na) - 0.5);
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
[t, x] = ode45('dv_BPE_Square', [0, Tsim], [P_0(:); V_0(:); R_0(:); Pdes0(:)]);
close(wbar);

%%
close all
visualize_square(x, t, N, na, false, '',' formation with changing shapes', -167.6036, 5.7501) % Dont forget to change position of numbers
