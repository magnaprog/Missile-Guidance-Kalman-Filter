% Missile Guidance with Kalman Filter in Cartesian Space

clc; close all;
clear; 

% Define simulation parameters
dt = 0.1;            % Time step
timesteps = 100;     % Number of simulation steps

% Updated simulation parameters
omega_target = 0.1;   % Frequency of target motion (adjust as needed)

% Initialize state variables for the moving target
x_target = 50;        % Initial x position of the target
y_target = 50;        % Initial y position of the target
z_target = 0;         % Initial z position of the target
vx_target = 5;        % Initial velocity of the target in x
vy_target = 2;        % Initial velocity of the target in y
vz_target = 0;        % Initial velocity of the target in z

x_missile = 0;       % Initial x position of the missile
y_missile = 0;       % Initial y position of the missile
z_missile = 0;       % Initial z position of the missile
vx_missile = 10;     % Initial velocity of the missile in x
vy_missile = 5;      % Initial velocity of the missile in y
vz_missile = 2;      % Initial velocity of the missile in z

% Process and measurement noise
Q = eye(6);          % Process noise covariance
R = 10*eye(3);       % Measurement noise covariance

% Measurement noise covariance
R = 10 * eye(3);       % Measurement noise covariance for target position
R_IR = 1 * eye(3);      % Measurement noise covariance for IR sensor (adjust as needed)

% Initialize Kalman filter parameters
A = eye(6);          % State transition matrix
B = eye(6);          % Control matrix
H = eye(3,6);        % Measurement matrix

% Initial state estimate
x_hat = [x_missile; y_missile; z_missile; vx_missile; vy_missile; vz_missile];

% Initialize state covariance matrix
P = eye(6);

% Initialize storage for simulation results
true_state = zeros(6, timesteps);
measured_state = zeros(3, timesteps);
estimated_state = zeros(6, timesteps);

% Simulation loop
for k = 1:timesteps
    % Update target position with sinusoidal motion
    x_target = x_target + vx_target * dt;
    y_target = y_target + vy_target * dt;
    z_target = z_target + vz_target * dt;
    
    x_target = x_target + 5 * sin(omega_target * k * dt);  % Sinusoidal motion in x
    y_target = y_target + 5 * cos(omega_target * k * dt);  % Sinusoidal motion in y

    true_state(:, k) = [x_missile; y_missile; z_missile; vx_missile; vy_missile; vz_missile];
    
    % IR sensor measurement update (add noise)
    measurement = [x_target - x_missile; y_target - y_missile; z_target - z_missile] + sqrt(R_IR) * randn(3,1);
    measured_state(:, k) = measurement;
    
    % Kalman filter prediction
    x_hat_minus = A * x_hat;
    P_minus = A * P * A' + Q;

    % Kalman filter update
    K = P_minus * H' / (H * P_minus * H' + R_IR);
    x_hat = x_hat_minus + K * (measurement - H * x_hat_minus);
    P = (eye(6) - K * H) * P_minus;

    estimated_state(:, k) = x_hat;
end

% Plotting
figure;

% Plot the true and estimated positions
subplot(2,1,1);
plot3(true_state(1,:), true_state(2,:), true_state(3,:), 'b-', 'LineWidth', 2);
hold on;
plot3(estimated_state(1,:), estimated_state(2,:), estimated_state(3,:), 'r--', 'LineWidth', 2);
plot3(measured_state(1,:), measured_state(2,:), measured_state(3,:), 'go', 'MarkerSize', 8);
grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('True and Estimated Missile Position');

% Plot the velocity
subplot(2,1,2);
plot(1:timesteps, true_state(4,:), 'b-', 'LineWidth', 2);
hold on;
plot(1:timesteps, estimated_state(4,:), 'r--', 'LineWidth', 2);
plot(1:timesteps, true_state(5,:), 'g-', 'LineWidth', 2);
plot(1:timesteps, estimated_state(5,:), 'c--', 'LineWidth', 2);
plot(1:timesteps, true_state(6,:), 'm-', 'LineWidth', 2);
plot(1:timesteps, estimated_state(6,:), 'y--', 'LineWidth', 2);
grid on;
xlabel('Time Steps');
ylabel('Velocity');
legend('True Vx', 'Estimated Vx', 'True Vy', 'Estimated Vy', 'True Vz', 'Estimated Vz');
title('True and Estimated Missile Velocity');

hold off;

% Plotting and Animation
figure;

% Create a plot for the target
target_plot = plot3(x_target, y_target, z_target, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
hold on;

% Create a plot for the missile
missile_plot = plot3(x_missile, y_missile, z_missile, 'b-', 'LineWidth', 2);

grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Missile Guidance with Kalman Filter');

% Animation loop
for k = 1:timesteps
    % Update target position with sinusoidal motion for animation
    x_target = x_target + vx_target * dt;
    y_target = y_target + vy_target * dt;
    z_target = z_target + vz_target * dt;
    
    x_target = x_target + 5 * sin(omega_target * k * dt);  % Sinusoidal motion in x
    y_target = y_target + 5 * cos(omega_target * k * dt);  % Sinusoidal motion in y
    
    % Update missile position in the plot
    set(missile_plot, 'XData', estimated_state(1, 1:k), 'YData', estimated_state(2, 1:k), 'ZData', estimated_state(3, 1:k));
    
    % Update target position in the plot
    set(target_plot, 'XData', x_target, 'YData', y_target, 'ZData', z_target);

    % Pause for a short duration to create the animation effect
    pause(0.1);
end

hold off;
