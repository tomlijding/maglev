clear;clc;clf;

h = 0.001;
Tsim = 30;
t = [0:h:Tsim]';
u = zeros(size(t));
t0 = 4
u(t<t0) = -0.5 + 0.5*t(t<t0) / t0;

simin = [t, u]

