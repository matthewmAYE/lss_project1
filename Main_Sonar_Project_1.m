clear;
clc;
close all;

% Generate transmitted and received sonar signals
[s, r, fs] = sonarMeasurement();

% Plot transmitted signal
figure;
plot(s);
xlabel('Sample');
ylabel('Amplitude');
title('Transmitted Sonar Signal');
grid on;

% Plot received signal
figure;
plot(r);
xlabel('Sample');
ylabel('Amplitude');
title('Received Sonar Signal');
grid on;

%% Part (c): Cross-correlation
% Compute cross-correlation between received and transmitted signals
[R,lags] = xcorr(r,s);
% Keep only nonnegative lags
idx = lags >= 0;
lags_pos = lags(idx);
R_pos = abs(R(idx));
% Plot cross-correlation magnitude
figure;
plot(lags_pos,R_pos,'LineWidth',1.2)
grid on
xlabel('Lag [samples]')
ylabel('|R_{rs}[k]|')
title('Cross-Correlation Magnitude')
hold on
% Find local peaks without using findpeaks
peak_idx = find(R_pos(2:end-1) > R_pos(1:end-2) & ...
    R_pos(2:end-1) >= R_pos(3:end)) + 1;
% Get peak magnitudes
peak_values_all = R_pos(peak_idx);
% Check that at least two peaks exist
if length(peak_idx) < 2
    error('Fewer than two local peaks found. Inspect the plot.');
end
% Sort peaks from strongest to weakest
[~,order] = sort(peak_values_all,'descend');
% Select two strong peaks separated by a minimum lag
minSeparation = max(1,round(length(s)/10));
selected_idx = [];
for k = 1:length(order)
    candidate = peak_idx(order(k));
    if isempty(selected_idx) || ...
            all(abs(lags_pos(candidate) - ...
            lags_pos(selected_idx)) >= minSeparation)
        selected_idx(end+1) = candidate;
        if length(selected_idx) == 2
            break;
        end
    end
end
if length(selected_idx) < 2
    error('Could not find two separated peaks. Inspect the plot.');
end
% Sort selected peaks by increasing lag
[peak_lags,order2] = sort(lags_pos(selected_idx));
peak_values = R_pos(selected_idx(order2));
% Mark selected peaks
plot(peak_lags,peak_values,'ro','MarkerSize',8,'LineWidth',1.5)
legend('Cross-correlation','Selected peaks')
hold off
% Estimated round-trip delays in samples
N1_hat = peak_lags(1);
N2_hat = peak_lags(2);
fprintf('\n--- Part (c): Estimated Delays ---\n');
fprintf('Estimated delay 1 = %d samples\n',N1_hat);
fprintf('Estimated delay 2 = %d samples\n',N2_hat);

%% Part (d): Calculate obstacle distances
c = 343;       % Speed of sound (m/s)

d1 = c*N1_hat/(2*fs);
d2 = c*N2_hat/(2*fs);

fprintf('\n--- Part (d): Estimated Distances ---\n');
fprintf('Distance to object 1 = %.3f m\n',d1);
fprintf('Distance to object 2 = %.3f m\n',d2);

%% Part (e): Safety decision
dsafe = 1.5;   % Safety distance (m)

fprintf('\n--- Part (e): Safety Decision ---\n');

if d1 <= dsafe || d2 <= dsafe
    fprintf('The robot must stop.\n');
else
    fprintf('The robot can continue moving.\n');
end