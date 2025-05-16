% Magnetic Leviation 
clear all
close all
clc
addpath LTI-toolbox/
addpath Given

%% Finding x0

% For this run, please remove all control. We simply want to observe what
% x0 is.
h = 0.001;
Tsim = 30;
t = [0:h:Tsim]';

u = ones(size(t));
simin = [t, u];

x = simout.Data;
x0 = sum(x)/length(x);
%% Identification data generation run

h = 0.001;
Tsim = 30;
t = [0:h:Tsim]';
u = zeros(size(t));
t0 = 4;
u(t<t0) = -0.5 + 0.5*t(t<t0) / t0;
len_nonzero = length(find(u));
amp = 0.005;
freq = 0.005;
id_signal = amp*prbn(length(u) - len_nonzero,freq) - amp/2;
u(len_nonzero+1:end) = id_signal;
simin = [t, u];

%% Save the identification data (DO NOT RUN)

y = simout.Data;
save("Data/id_data.mat","u","y")

%% Validation data generation run

h = 0.001;
Tsim = 30;
t = [0:h:Tsim]';
u = zeros(size(t));
t0 = 4;
u(t<t0) = -0.5 + 0.5*t(t<t0) / t0;
id_signal = 0.005*chirp(t(begin_time:end),0.1,t(end),10);
u(begin_time:end) = id_signal;
simin = [t, u];

%% Save the validation data (DO NOT RUN)
y = simout.Data;
save("Data/val_data.mat","u","y")
%% Subspace Identification Part 1: Determining model order
id_data = load("Data/id_data.mat");
u = id_data.u;
y = id_data.y;
begin_time = 15/h+1;
end_period = 2/h;
u_id = u(begin_time:end-end_period);
y_id = y(begin_time:end-end_period);

s = 20; % Expected maximum order of our model

[S,R] = dordpi(u_id,y_id,s);

figure, semilogy(1:20,S,'x')

%% Subspace Identification Part 2: Identifying A and C matrices

n = 3;
[Ae,Ce] = dmodpi(R,n);

%% Subspace Identification Part 3: Identifying B, D and x0

[Be,De] = dac2bd(Ae,Ce,u_id,y_id);
x0e = dinit(Ae,Be,Ce,De,u_id,y_id);

%% Subspace Identification Part 4: Validating

val_data = load("Data/val_data.mat");
y = val_data.y;
u = val_data.u;
u_val = u(begin_time:end-end_period);
y_val = y(begin_time:end-end_period);
y_val = y_val - y_val_avg;
y_sim = dltisim(Ae,Be,Ce,De,u_val,x0e);
vaf(y_sim,y_val)

figure
plot(t(begin_time:end-end_period),y_sim)
hold on
plot(t(begin_time:end-end_period),y_val)
legend("y_sim"," y_id")
%% Observer Synthesis
sys = ss(Ae,Be,Ce,De);
out_std = 1.5e-1;
plant_std = 3e-1;
Q_n = plant_std^2;
R_n = out_std^2;

[kalmf,L,~,Mx,Z] = kalman(sys,Q_n,R_n);
kalmf = kalmf(2:end,:);

%% Controller Synthesis

Lc = dcgain(sys);

Q = eye(n);
R = 1;

K = dlqr(Ae,Be,Q,R);

%% Running the hardware 

h = 0.001;
Tsim = 30;
t = [0:h:Tsim]';
u = zeros(size(t));
t0 = 4;
u(t<t0) = -0.5 + 0.5*t(t<t0) / t0;
simin = [t, u];

%% Initializing vectors (Linear System)

h = 0.001;
Tsim = 30;
t = [0:h:Tsim]';
u = zeros(size(t));
t0 = 15;
u(t>t0) = 1;
simin = [t, u];

Cfull = eye(3);