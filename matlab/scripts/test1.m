close all;
clear;
clc
%https://ww2.mathworks.cn/help/robotics/ug/path-following-for-differential-drive-robot.html

%https://ww2.mathworks.cn/help/robotics/ref/controllerpurepursuit-sys
%tem-object.html?s_tid=srchtitle
%https://ww2.mathworks.cn/help/robotics/ref/differentialdrivekinemati

%cs.html?s_tid=srchtitle
setenv('ROS_MASTER_URI','http://192.168.1.102:11311');
rosinit();
try
cmdpub = rospublisher('/raw_cmd_vel',rostype.geometry_msgs_Twist);
cmdmsg = rosmessage(cmdpub);
odomsub = rossubscriber('/odom');
odommsg = rosmessage(odomsub);
path = [0 0;
-2.00 0.00;
-2.00 1.0];
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';
robot = differentialDriveKinematics();
robot.WheelRadius = 0.1525;
robot.TrackWidth = 0.71;
robot.VehicleInputs = 'VehicleSpeedHeadingRate';
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.2; %
controller.MaxAngularVelocity = 0.3; %
controller.LookaheadDistance = 0.3;
goalRadius = 0.2;
distanceToGoal = norm(robotInitialLocation - robotGoal);
% Initialize the simulation loop
sampleTime = 0.02;
vizRate = rateControl(1/sampleTime);
% Initialize the figure
figure
% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;
while( distanceToGoal > goalRadius )
% Compute the controller outputs, i.e., the inputs to the robot
[v, omega] = controller(robotCurrentPose);
cmdmsg.Linear.X = v;
cmdmsg.Angular.Z = omega;
send(cmdpub,cmdmsg);
% Get the robot's velocity using controller inputs
%vel = derivative(robot, robotCurrentPose, [v omega]);
% Update the current pose
%robotCurrentPose = robotCurrentPose + vel*sampleTime;
odommsg = receive(odomsub,10);
[r1,r2,r3]=quat2angle([odommsg.Pose.Pose.Orientation.X ...
odommsg.Pose.Pose.Orientation.Y ...
odommsg.Pose.Pose.Orientation.Z ...
odommsg.Pose.Pose.Orientation.W], ...
'ZYX');
robotCurrentPose = [odommsg.Pose.Pose.Position.X; ...
odommsg.Pose.Pose.Position.Y; ...
r3];
% Re-compute the distance to the goal
distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
% Update the plot
hold off
% Plot path each instance so that it stays persistent while robot mesh
% moves
plot(path(:,1), path(:,2),"k--d")
hold all
% Plot the path of the robot as a set of transforms
plotTrVec = [robotCurrentPose(1:2); 0];
plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize",frameSize);
light;
xlabel('X');
ylabel('Y');
xlim([-13 13]);
ylim([-13 13]);
waitfor(vizRate);
end
catch
rosshutdown;
end
cmdmsg.Linear.X = 0;
cmdmsg.Angular.Z = 0;
send(cmdpub,cmdmsg);
waitfor(vizRate);
rosshutdown;