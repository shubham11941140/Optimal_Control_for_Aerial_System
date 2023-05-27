t = 0:0.01:1.55;

% Define position as a function of time
x = t.^3 - 1.117*t;


% Plot x versus t
plot(t, x);

% Add labels and title
xlabel('Time');
ylabel('Position');
title('Optimal Trajectory 1');
