function fig = initialize_robot_figure(robot,varargin)

if nargin>1
    % figure exists and we should activate axes to plot in
%     axes(varargin{1})
    fig = varargin{1};
else
    % create plot in new figure window
    fig = figure('Color',[1 1 1]);
end
robot.plot([0,0])
view([0 90])
hold on;

% plot workspace limitation
rad = sum(robot.a);
x_data = -0.9 + rad*sin(0:0.01:2*pi);
y_data = 1 + rad*cos(0:0.01:2*pi);
plot(x_data, y_data, 'k--','linewidth', 1);
axis equal
axis([-2.5 0.5 -0.45 1.2])
set(fig, 'Position', [0.0208 0.4750 0.4646 0.4358])
end
