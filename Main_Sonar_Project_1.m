clear; clc; close all;

% Obtain transmitted and measured sonar signals
[s,r,fs] = sonarMeasurement();

% Time vectors
ts = (0:length(s)-1)/fs;
tr = (0:length(r)-1)/fs;

% Plot transmitted signal
figure;
plot(ts*1e3,s,'LineWidth',1.2)
grid on
xlabel('Time [ms]')
ylabel('Amplitude')
title('Transmitted Signal $s[n]$','Interpreter','latex')

% Plot received signal
figure;
plot(tr*1e3,r,'LineWidth',1.0)
grid on
xlabel('Time [ms]')
ylabel('Amplitude')
title('Measured Received Signal $r[n]$','Interpreter','latex')