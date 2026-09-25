figure; 
d = dlmread('robot_log.csv', ',', 1, 0);   % skip the header row
plot(d(:,1), d(:,3), d(:,1), d(:,2), '--');
xlabel('time (s)'); ylabel('y (m)'); legend('y', 'target'); ylim([0 1 ]); grid on;