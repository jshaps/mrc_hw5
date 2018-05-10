% Create a bag file object with the file name
bag = rosbag('catkin_ws/src/mrc_hw5/mrc_hw5_data/waypoint.bag')
   
% Display a list of the topics and message types in the bag file
bag.AvailableTopics
   
% Since the messages on topic /odom are of type Odometry,
% let's see some of the attributes of the Odometry
% This helps determine the syntax for extracting data
msg_odom = rosmessage('nav_msgs/Odometry')
showdetails(msg_odom)
   
% Get just the topic we are interested in
bagselect = select(bag,'Topic','/odom');
   
% Create a time series object based on the fields of the turtlesim/Pose
% message we are interested in
ts = timeseries(bagselect,'Pose.Pose.Position.X','Pose.Pose.Position.Y',...
    'Twist.Twist.Linear.X','Twist.Twist.Angular.Z',...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X',...
    'Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z');

% The time vector in the timeseries (ts.Time) is "Unix Time"
% which is a bit cumbersome.  Create a time vector that is relative
% to the start of the log file
tt = ts.Time-ts.Time(1);
% Plot the X position vs time
% figure(1);
% clf();
% plot(tt,ts.Data(:,1))
% xlabel('Time [s]')
% ylabel('X [m]')
% 
% % Plot the Y position vs time
% figure(2);
% clf();
% plot(tt,ts.Data(:,2))
% xlabel('Time [s]')
% ylabel('Y [m]')

X = ts.Data(:,1);
Y = ts.Data(:,2);

% Plot X and Y position of Robot
figure();
clf();
plot(X,Y)
hold on
plot(X(1),Y(1),'g*')
plot(X(end),Y(end),'r*')
xlabel('X [m]')
ylabel('Y [m]')
grid on
legend('Position','start','end')
title('Turtlebot Position over Time')
saveas(gcf, 'waypoint_odom_xy.png')

% Plot heading angle (yaw) of robot as function of time
theta = ts.Data(:,8)*(180/pi);
figure()
clf();
plot(tt,theta)
grid on
xlabel('time [s]')
ylabel('Yaw [degrees]')
title('Yaw as a functin of time')
saveas(gcf, 'waypoint_odom_yaw.png')


% Plot forward velocity as function of time
u = ts.Data(:,3);        % grab linear velocity
figure()
clf();
plot(tt,u)
grid on
xlabel('Time [s]')
ylabel('Forward velocity [m/s]')
title('Turtlebot Forward Velcoity')
saveas(gcf, 'waypoint_odom_u.png')


% Quiver plot with location, yaw and speed

quat = [ts.data(:,5),ts.data(:,6),ts.data(:,7),ts.data(:,8)];
eul = quat2eul(quat);
theta = eul(:,1);
u = ts.data(:,3).*cos(theta);
v = ts.data(:,3).*sin(theta);
ii = 1:10:length(ts.data(:,3));

figure();
clf();
plot(X,Y)
quiver(X(ii),Y(ii),u(ii),v(ii))
hold on;
grid on
title('Quiver Plot');
plot(X(1),Y(1),'g*');
plot(X(end),Y(end),'r*');
xlabel('X [m]')
ylabel('Y [m]')
saveas(gcf,'waypoint_odom_quiver.png')