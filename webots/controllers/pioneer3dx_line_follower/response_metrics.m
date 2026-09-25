% response_metrics.m
% Computes performance measures of a run from robot_log.csv:
%   - IAE (integral of the absolute error) for each half of the run;
%   - peak-to-peak amplitude and period of any oscillation in the last
%     10 s of each half (i.e. the behaviour the response converges to);
%   - rise time, overshoot and settling time of the step at t = 30 s,
%     only when the robot had settled before the step.
% It also plots the response.

win  = 10;      % window at the end of each half used to measure oscillation (s)
band = 0.05;    % settling band: +/-5 % of the step size

d = dlmread('robot_log.csv', ',', 1, 0);   % time,target,y,error,steer
t = d(:,1);  r = d(:,2);  y = d(:,3);  e = d(:,4);

% Locate the set-point step: first sample where the target changes
k0 = find(diff(r) ~= 0, 1) + 1;
if isempty(k0)
  error('No set-point step found in robot_log.csv');
end
t_step = t(k0);  r_old = r(k0-1);  r_new = r(k0);  dr = r_new - r_old;
tol = band * abs(dr);                      % settling tolerance (m)

halves = {1:k0-1, k0:numel(t)};
names  = {'Before the step', 'After the step '};
pp = zeros(1,2);

fprintf('\n%-16s %10s %12s %12s %12s\n', '', 'IAE (m.s)', 'p-p (m)', 'period (s)', 'mean err (m)');
for h = 1:2
  k  = halves{h};
  th = t(k);  yh = y(k);  eh = e(k);
  iae = trapz(th, abs(eh));

  % Oscillation in the last 'win' seconds of this half
  w  = th >= th(end) - win;
  yw = yh(w);  tw = th(w);
  pp(h) = max(yw) - min(yw);
  me = mean(eh(w));

  if pp(h) < 2*tol
    per = NaN;                             % no significant oscillation
  else
    s  = sign(yw - mean(yw));
    ic = find(s(1:end-1) .* s(2:end) < 0); % mean crossings
    if numel(ic) >= 3
      per = 2 * mean(diff(tw(ic)));
    else
      per = Inf;                           % slower than the window
    end
  end
  fprintf('%-16s %10.3f %12.3f %12.2f %12.4f\n', names{h}, iae, pp(h), per, me);
end
fprintf('(period NaN: no oscillation; Inf: period longer than %d s)\n\n', win);

% Step-response measures, only if the robot had settled before the step
pre = t < t_step & t >= t_step - win;
settled_before = pp(1) < 2*tol && abs(mean(y(pre)) - r_old) < tol;

if ~settled_before
  fprintf('Step measures not computed: the robot had not settled before the step.\n');
  ttl = 'Not settled before the step';
else
  z  = (y(k0:end) - r_old) / dr;           % 0 = old set point, 1 = new
  tt = t(k0:end) - t_step;

  i10 = find(z >= 0.1, 1);  i90 = find(z >= 0.9, 1);
  if isempty(i10) || isempty(i90), tr = NaN; else, tr = tt(i90) - tt(i10); end

  os = max(0, max(z) - 1) * 100;

  out = find(abs(z - 1) > band);
  if isempty(out), ts = 0;
  elseif out(end) == numel(z), ts = NaN;   % never settled
  else, ts = tt(out(end) + 1); end

  fprintf('Rise time (10-90 %%): %.2f s\n', tr);
  fprintf('Overshoot:           %.1f %%\n', os);
  if isnan(ts)
    fprintf('Settling time:       did not settle within +/-%.0f %%\n', band*100);
  else
    fprintf('Settling time:       %.2f s\n', ts);
  end
  ttl = sprintf('t_r = %.2f s, overshoot = %.1f %%, t_s = %.2f s', tr, os, ts);
end

% Plot
figure;
hp = plot(t, y, t, r, '--'); hold on;
hb = plot([t_step t(end)], [r_new-tol r_new-tol], 'r:', ...
          [t_step t(end)], [r_new+tol r_new+tol], 'r:');
hold off;
xlabel('time (s)'); ylabel('y (m)'); grid on;
legend([hp(1) hp(2) hb(1)], 'y', 'target', 'settling band');
title(ttl);
