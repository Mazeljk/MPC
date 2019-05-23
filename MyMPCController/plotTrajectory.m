function [] = plotTrajectory(out)
r_Trajectory = [25 * sin([0:0.005:2 * pi]'), 35 - 25 * cos([0:0.005:2 * pi]')];
plot(r_Trajectory(:, 1), r_Trajectory(:, 2), 'b-');
hold on;
title('Trajectory Tracking');

predicted_Trajectory = out.simout(:, 1:2);
for i = 2 : length(predicted_Trajectory)
    plot(predicted_Trajectory(i - 1:i, 1), predicted_Trajectory(i - 1:i, 2), 'r:', 'LineWidth', 2);
    hold on;
    pause(0.05);
end
legend('Reference Trajectory', 'Predicted Trajectory');