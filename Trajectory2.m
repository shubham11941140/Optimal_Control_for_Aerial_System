t = 0:0.01:0.64;

% Define position as a function of time
x = t.^3 + 2.687*t;


% Plot x versus t
plot(t, x);

% Add labels and title
xlabel('Time');
ylabel('Position');
title('Optimal Trajectory 2');
