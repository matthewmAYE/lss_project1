%% Industrial Monitoring Project: LSS Project 1
% Main script

clear;
clc;
close all;


env = factoryEnvironment();

r_robot = 0.40;
v_max   = 1.0;
lambda  = 1.7e-3;
q0      = 1;
q_crit  = 0.60;

q = @(t) q0*exp(-lambda*t);         % battery model q(t)

% Latest time at qcrit:
%   q0*exp(-lambda*t_crit) = q_crit  ->  t_crit = ln(q0/q_crit)/lambda

t_crit = log(q0/q_crit)/lambda;




T_sensing = 60;



% 4*T_sensing = 240 s
% which uses most of the 300.5 s battery budget.
%
%   P0 = A  = ( 2.0,  2.0)  (assume we start at point (2,2))
%   P1 = M1 = ( 5.0, 12.0)   monitoring 1
%   P2      = (13.0, 12.0)
%   P3 = M2 = (17.5,  9.0)   monitoring 2
%   P4      = (22.5,  9.0)
%   P5 = M3 = (26.5, 12.0)   monitoring 3
%   P6 = M4 = (27.5, 18.0)   monitoring 4
%   P7 = B  = (38.0, 18.0)   reached by moving straight right from M4
%
% Segment   from -> to    duration   velocity (vx, vy)    speed
%   1       P0 -> P1       12.50 s    (0.24,  0.80)       0.835 m/s
%   2       P1 -> P2        8.00 s    (1.00,  0.00)       1.000 m/s
%   3       P2 -> P3        6.00 s    (0.75, -0.50)       0.901 m/s
%   4       P3 -> P4        5.00 s    (1.00,  0.00)       1.000 m/s
%   5       P4 -> P5        5.00 s    (0.80,  0.60)       1.000 m/s
%   6       P5 -> P6        6.25 s    (0.16,  0.96)       0.973 m/s
%   7       P6 -> P7       10.50 s    (1.00,  0.00)       1.000 m/s



T_travel = 12.5 + 8 + 6 + 5 + 5 + 6.25 + 10.5;

% Monitoring location points M1 toM4
M = [  5.0  12.0;
      17.5   9.0;
      26.5  12.0;
      27.5  18.0 ];

t0  = 0;
t1  = t0 + 12.5;
t2  = t1 + T_sensing;
t3  = t2 + 8;
t4  = t3 + 6;
t5  = t4 + T_sensing;
t6  = t5 + 5;
t7  = t6 + 5;
t8  = t7 + T_sensing;
t9  = t8 + 6.25;
t10 = t9 + T_sensing;
t11 = t10 + 10.5;

T_mission = t11;
assert(abs(T_mission - (T_travel + 4*T_sensing)) < 1e-9);




% ----------------------------------------------------------------

u = @(t) double(t >= 0);
r = @(t) t .* u(t);

x = @(t) 2*u(t) ...
       + 0.24*( r(t-t0)  - r(t-t1)  ) ...
       + 1.0 *( r(t-t2)  - r(t-t3)  ) + 0.75*( r(t-t3) - r(t-t4) ) ...
       + 1.0 *( r(t-t5)  - r(t-t6)  ) + 0.8 *( r(t-t6) - r(t-t7) ) ...
       + 0.16*( r(t-t8)  - r(t-t9)  ) ...
       + 1.0 *( r(t-t10) - r(t-t11) );

y = @(t) 2*u(t) ...
       + 0.8 *( r(t-t0)  - r(t-t1)  ) ...
       - 0.5 *( r(t-t3)  - r(t-t4)  ) ...
       + 0.6 *( r(t-t6)  - r(t-t7)  ) ...
       + 0.96*( r(t-t8)  - r(t-t9)  );


dt = 1e-3;                     % 1 ms sampling: robot moves <= 1 mm per sample
t  = 0:dt:T_mission;
xt = x(t);
yt = y(t);

tSw     = [t0 t1 t2 t3 t4 t5 t6 t7 t8 t9 t10 t11];   % switching times
stopInt = [t1 t2; t4 t5; t7 t8; t9 t10];          % sensing intervals
q_final = q(T_mission);

fprintf('======================= MISSION CHECKS =======================\n');

% Req. 3: robot disk inside factory and clear of all obstacles
wallClear = min([min(xt), 40 - max(xt), min(yt), 20 - max(yt)]) - r_robot;
fprintf('Min clearance to walls        = %.3f m\n', wallClear);

nC = size(env.circles,1);
clrC = zeros(1, nC);
for k = 1:nC
    c = env.circles(k,:);
    clrC(k) = min(hypot(xt - c(1), yt - c(2))) - c(3) - r_robot;
    fprintf('Min clearance to circle    C%d = %.3f m\n', k, clrC(k));
end

nR = size(env.rectangles,1);
clrR = zeros(1, nR);
for k = 1:nR
    rc = env.rectangles(k,:);
    dx = max(max(rc(1) - xt, 0), xt - rc(2));     % distance to rectangle
    dy = max(max(rc(3) - yt, 0), yt - rc(4));
    clrR(k) = min(hypot(dx, dy)) - r_robot;
    fprintf('Min clearance to rectangle R%d = %.3f m\n', k, clrR(k));
end
req3 = wallClear >= 0 && all(clrC >= 0) && all(clrR >= 0);
assert(req3, 'Robot leaves the factory or hits an obstacle');

% Req. 4: speed on each constant-velocity interval (midpoint derivative)
names = {'move A->M1','STOP M1','move M1->P2','move P2->M2','STOP M2', ...
         'move M2->P4','move P4->M3','STOP M3','move M3->M4','STOP M4', ...
         'move M4->B'};
h     = 1e-4;
nInt  = numel(tSw) - 1;
vxInt = zeros(1, nInt);
vyInt = zeros(1, nInt);
fprintf('\n%-12s %8s %8s %7s %7s %7s\n', 'Interval', 't_a [s]', 't_b [s]', 'vx', 'vy', '|v|');
for k = 1:nInt
    tm = 0.5*(tSw(k) + tSw(k+1));
    vxInt(k) = (x(tm+h) - x(tm-h))/(2*h);
    vyInt(k) = (y(tm+h) - y(tm-h))/(2*h);
    fprintf('%-12s %8.2f %8.2f %7.3f %7.3f %7.3f\n', names{k}, tSw(k), tSw(k+1), ...
            vxInt(k), vyInt(k), hypot(vxInt(k), vyInt(k)));
end
vInt     = hypot(vxInt, vyInt);
vSampled = hypot(diff(xt), diff(yt))/dt;          % whole-mission check
fprintf('Max speed: %.4f m/s per interval, %.4f m/s sampled (v_max = %.1f)\n', ...
        max(vInt), max(vSampled), v_max);
req4 = all(vInt <= v_max + 1e-9) && max(vSampled) <= v_max + 1e-6;
assert(req4, 'Speed limit exceeded');

% Req. 5: four distinct stops with x' = y' = 0
stopK = [2 5 8 10];                               % stop interval indices
req5  = size(unique(M,'rows'),1) == 4;
fprintf('\n');
for k = 1:4
    idx   = t >= stopInt(k,1) & t <= stopInt(k,2);
    drift = max(hypot(xt(idx) - M(k,1), yt(idx) - M(k,2)));
    fprintf('M%d = (%5.2f, %5.2f): [%6.2f, %6.2f] s, x''=%g, y''=%g, drift %.1e m\n', ...
            k, M(k,1), M(k,2), stopInt(k,1), stopInt(k,2), ...
            vxInt(stopK(k)), vyInt(stopK(k)), drift);
    req5 = req5 && drift < 1e-9 && abs(vxInt(stopK(k))) < 1e-9 && abs(vyInt(stopK(k))) < 1e-9;
end
assert(req5, 'Monitoring stops are not four distinct stationary intervals');

pf = {'FAIL', 'PASS'};
fprintf('\nReq. 3 (inside factory, no collisions): %s\n', pf{req3 + 1});
fprintf('Req. 4 (speed <= v_max):                %s\n', pf{req4 + 1});
fprintf('Req. 5 (four stationary stops):         %s\n', pf{req5 + 1});
fprintf('==============================================================\n');





% (ii) Plot of the trajectory on the factory map


% factoryEnvironment() left its figure current with hold on
set(gcf, 'Name', '(ii) Robot trajectory');

% Robot body along the path (disk drawn every 0.5 s)
phi = linspace(0, 2*pi, 60);
for tk = 0:0.5:T_mission
    fill(x(tk) + r_robot*cos(phi), y(tk) + r_robot*sin(phi), ...
         [0.30 0.60 1.00], 'EdgeColor', 'none', 'FaceAlpha', 0.06, ...
         'HandleVisibility', 'off');
end

% Centre path and monitoring locations
hPath = plot(xt, yt, 'b-', 'LineWidth', 2);
hMon  = plot(M(:,1), M(:,2), 'p', 'MarkerSize', 16, ...
             'MarkerFaceColor', [0.95 0.75 0.10], 'MarkerEdgeColor', 'k');
for k = 1:4
    text(M(k,1) + 0.3, M(k,2) + 0.8, sprintf('$M_%d$', k), ...
         'Interpreter', 'latex', 'FontSize', 15);
end
legend([hPath hMon], {'Robot centre path $p(t)$', 'Monitoring locations $M_1$--$M_4$'}, ...
       'Interpreter', 'latex', 'Location', 'southoutside', 'Orientation', 'horizontal');
title(sprintf(['Robot trajectory with VOC monitoring stops ' ...
               '($T_{sensing}$ = %g s, $T_{mission}$ = %g s)'], ...
               T_sensing, T_mission), 'Interpreter', 'latex');



% (iii) Plots of x(t) and y(t) as functions of time

tPlot   = -5:0.01:305;
stopCol = [0.95 0.75 0.10];

figure('Color', 'w', 'Name', '(iii) x(t) and y(t)');

subplot(2,1,1); hold on;
for k = 1:4
    patch([stopInt(k,1) stopInt(k,2) stopInt(k,2) stopInt(k,1)], ...
          [0 0 43 43], stopCol, 'FaceAlpha', 0.18, 'EdgeColor', 'none');
    text(mean(stopInt(k,:)), 40, sprintf('$M_%d$', k), ...
         'Interpreter', 'latex', 'HorizontalAlignment', 'center');
end
plot(tPlot, x(tPlot), 'b', 'LineWidth', 2);
plot([T_mission T_mission], [0 43], 'k--');
grid on; box on; xlim([-5 305]); ylim([0 43]);
ylabel('$x(t)$ [m]', 'Interpreter', 'latex');
title('Position signals $x(t)$ and $y(t)$ (shaded: VOC sensing intervals)', ...
      'Interpreter', 'latex');
set(gca, 'FontSize', 12, 'TickLabelInterpreter', 'latex');

subplot(2,1,2); hold on;
for k = 1:4
    patch([stopInt(k,1) stopInt(k,2) stopInt(k,2) stopInt(k,1)], ...
          [0 0 20 20], stopCol, 'FaceAlpha', 0.18, 'EdgeColor', 'none');
end
plot(tPlot, y(tPlot), 'r', 'LineWidth', 2);
plot([T_mission T_mission], [0 20], 'k--');
grid on; box on; xlim([-5 305]); ylim([0 20]);
xlabel('$t$ [s]', 'Interpreter', 'latex');
ylabel('$y(t)$ [m]', 'Interpreter', 'latex');
set(gca, 'FontSize', 12, 'TickLabelInterpreter', 'latex');




tStair = [-5, 0, reshape([tSw(1:end-1); tSw(2:end)], 1, []), T_mission, 305];
vStair = [ 0, 0, reshape([vInt; vInt], 1, []),               0,         0 ];

figure('Color', 'w', 'Name', '(iv) Speed v(t)'); hold on;
for k = 1:4
    patch([stopInt(k,1) stopInt(k,2) stopInt(k,2) stopInt(k,1)], ...
          [0 0 1.35 1.35], stopCol, 'FaceAlpha', 0.18, 'EdgeColor', 'none', ...
          'HandleVisibility', 'off');
end
plot(tStair, vStair, 'b-', 'LineWidth', 2.2);
plot([-5 305], [v_max v_max], 'r--', 'LineWidth', 1.4);
grid on; box on; xlim([-5 305]); ylim([0 1.35]);
xlabel('$t$ [s]', 'Interpreter', 'latex');
ylabel('$v(t)$ [m/s]', 'Interpreter', 'latex');
legend({'$v(t)=\sqrt{\dot{x}^2(t)+\dot{y}^2(t)}$', '$v_{max}$ = 1.0 m/s'}, ...
       'Interpreter', 'latex', 'Location', 'north', 'Orientation', 'horizontal');
title('Robot translational speed (shaded: sensing intervals, $v = 0$)', ...
      'Interpreter', 'latex');
set(gca, 'FontSize', 12, 'TickLabelInterpreter', 'latex');


%% RESULTS (i) - (vii)


fprintf('\n\n######################## RESULTS ########################\n');


% Generated from the velocity on each interval, so they always match
% the x(t) and y(t) actually used. At each switching time t_k the
% velocity jumps by dv_k, which gives a term dv_k * r(t - t_k):
%   x(t) = x_A u(t) + sum_k dvx_k r(t - t_k)
%   y(t) = y_A u(t) + sum_k dvy_k r(t - t_k)

fprintf('\n(i) Analytical position signals\n');
fprintf('    u(t) = unit step,  r(t) = t u(t) = unit ramp,  t in s, x and y in m\n\n');

sigName = {'x', 'y'};
vIntAll = {vxInt, vyInt};
termsPerLine = 4;
req2 = true;

for s = 1:2
    v  = round([0, vIntAll{s}, 0]*1e6)/1e6;   % velocity before, during, after
    dv = diff(v);                             % jump at each tSw(k)

    str    = sprintf('    %s(t) = %gu(t)', sigName{s}, env.A(s));
    indent = repmat(' ', 1, 11);
    nTerms = 0;
    sigRec = env.A(s)*u(t);                   % rebuilt signal (self-check)

    for k = 1:numel(tSw)
        c = dv(k);
        if abs(c) < 1e-9, continue; end
        if c < 0, sgn = ' - '; else, sgn = ' + '; end
        if abs(abs(c) - 1) < 1e-9, coef = ''; else, coef = sprintf('%g', abs(c)); end
        if tSw(k) == 0, arg = 't'; else, arg = sprintf('t-%g', tSw(k)); end
        if nTerms > 0 && mod(nTerms, termsPerLine) == 0
            str = [str, newline, indent];  %#ok<AGROW>
        end
        str    = [str, sgn, coef, 'r(', arg, ')'];  %#ok<AGROW>
        nTerms = nTerms + 1;
        sigRec = sigRec + c*r(t - tSw(k));
    end
    fprintf('%s\n\n', str);

    if s == 1, sigTrue = xt; else, sigTrue = yt; end
    req2 = req2 && max(abs(sigRec - sigTrue)) < 1e-6;
end
assert(req2, 'Printed x(t)/y(t) do not reproduce the trajectory');
fprintf('    For t >= %g s every ramp cancels and the robot stays at B.\n', T_mission);

fprintf('\n(ii)  Figure "(ii) Robot trajectory": path on the factory map, M1-M4 marked\n');
fprintf('(iii) Figure "(iii) x(t) and y(t)": position signals versus time\n');
fprintf('(iv)  Figure "(iv) Speed v(t)": speed with v_max = %.1f m/s (max reached %.3f m/s)\n', ...
        v_max, max(vInt));

fprintf('\n(v) Monitoring locations and sensing duration\n');
for k = 1:4
    fprintf('    M%d = (%5.2f, %5.2f) m   stationary for t in [%6.2f, %6.2f] s\n', ...
            k, M(k,1), M(k,2), stopInt(k,1), stopInt(k,2));
end
fprintf('    T_sensing = %g s at every location\n', T_sensing);
fprintf(['    Justification: the SGAS707 datasheet gives a typical response time\n' ...
         '    of < 1 minute to 90%% of full scale (t90 <= 60 s). After 60 s\n' ...
         '    stationary the reading has reached >= 90%% of its steady-state value,\n' ...
         '    so it is a valid VOC measurement. The heater (VH = 3.5 V, ~400 mW,\n' ...
         '    150 C) stays on for the whole mission, so no warm-up time is added.\n' ...
         '    Battery limit on each stop: (t_crit - T_travel)/4 = %.1f s >= %g s.\n'], ...
         (t_crit - T_travel)/4, T_sensing);


fprintf('\n(vi) Mission timing\n');
fprintf('    T_travel  = %.2f s\n', T_travel);
fprintf('    T_sensing = %.2f s  (4 x T_sensing = %.2f s)\n', T_sensing, 4*T_sensing);
fprintf('    T_mission = T_travel + 4*T_sensing = %.2f + %.2f = %.2f s\n', ...
        T_travel, 4*T_sensing, T_mission);


fprintf('\n(vii) Battery\n');
fprintf('    q(T_mission) = q0*exp(-lambda*T_mission) = exp(-%.1e x %.2f) = %.4f\n', ...
        lambda, T_mission, q_final);
fprintf('    q(T_mission) = %.4f >= q_crit = %.2f  ->  constraint satisfied\n', ...
        q_final, q_crit);
fprintf('    t_crit = ln(q0/q_crit)/lambda = %.2f s;  margin = %.2f s\n', ...
        t_crit, t_crit - T_mission);

fprintf('#########################################################\n');
