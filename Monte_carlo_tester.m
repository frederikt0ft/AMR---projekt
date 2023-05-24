rosshutdown
%connect 1 den
setenv('ROS_MASTER_URI','http://192.168.238.43:11311')
% 2 mig
setenv('ROS_IP','192.168.238.157')
% 3 den mig
rosinit('http://192.168.238.43:11311','NodeHost','192.168.238.157')

%%   ----LOAD MAP-----
% Load the image
img = imread(['Real_map.png']);
% Convert the image to grayscale
gray_img = rgb2gray(img);
% Convert the grayscale image to a binary image
binary_img = imbinarize(gray_img);
    
% Create a binary occupancy map from the binary image
scale_factor = 21;   %500 pixels pr. meter
map = occupancyMap(binary_img,scale_factor);

% Create a new binary occupancy map from the resized matrix
%new_map = binaryOccupancyMap(map);

% Show the map
show(new_map)
%%  ----LAVER NODES---------
%Create points
NumNodes = 1500;
prmComplex = mobileRobotPRM(new_map,NumNodes);
prmComplex.ConnectionDistance = 2.0;
show(prmComplex)
%% -----LAVER PATH 1-------
%Define start location

startLocation1 = [41.0 28.0];
endLocation1 = [2.0 10.0];
path1 = findpath(prmComplex,startLocation1,endLocation1)
show(prmComplex)
%%

% -----LAVER PATH 2-------
startLocation2 = [2.0 9.0];
endLocation2 = [21.0 3.0];
path2 = findpath(prmComplex,startLocation2,endLocation2);
show(prmComplex)
%%  Show planned path 1 and 2
%1st path
figure
plot(path1(:,1), path1(:,2),'k--d')
xlim([-0 45])
ylim([-0 30])
 
%2nd path
hold on
plot(path2(:,1), path2(:,2),'k--d')
xlim([-0 45])
ylim([-0 30])
hold off
%% ----------- INITILIZATION ---------
robotInitialLocation = path1(1,:);
robotGoal = path1(end,:);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
%% -----------Controller------------- 
controller = controllerPurePursuit;
controller.Waypoints = path1;
controller.DesiredLinearVelocity = 0.6;
controller.MaxAngularVelocity = 0.5;
controller.LookaheadDistance = 1.2;
goalRadius = 0.5;
distanceToGoal = norm(robotInitialLocation - robotGoal);

%% --- Test ---
odomSub = rossubscriber("/odom","DataFormat","struct");
r_pose = receive(odomSub);
fprintf("-----------------------------------------------")
init_position = [r_pose.Pose.Pose.Position.X r_pose.Pose.Pose.Position.Y];
r_init_orientation = [r_pose.Pose.Pose.Orientation.X r_pose.Pose.Pose.Orientation.Y r_pose.Pose.Pose.Orientation.Z r_pose.Pose.Pose.Orientation.W];
eul = quat2eul(r_init_orientation);
init_orientation = eul(3); %Venstre om 0 til pi. Højre om 0 til -pi
init_pose = [init_position init_orientation]'

FREDE_OG_ASKES_INIT_O = init_orientation
%% --- GØR TURLTEBOT KLAR TIL KAMP ---
% TIL TURTLEBOT - DEFINITIVT!
vel_pub = rospublisher('/cmd_vel');
velmsg = rosmessage(vel_pub);
odomSub = rossubscriber("/odom","DataFormat","struct");

% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
%figure;
show(new_map);

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth*2;
s = path1(1,:)

current_pose = robotCurrentPose
r_pose = receive(odomSub);

init_position = [r_pose.Pose.Pose.Position.X r_pose.Pose.Pose.Position.Y] - [startLocation1]     %INITITAL POS DETERMINED
r_init_orientation = [r_pose.Pose.Pose.Orientation.X r_pose.Pose.Pose.Orientation.Y r_pose.Pose.Pose.Orientation.Z r_pose.Pose.Pose.Orientation.W]

eul = quat2eul(r_init_orientation);
init_orientation = eul(3);                                                    %INITAL ORIENTATION DETERMINED
init_pose = [init_position init_orientation]';

theta = eul(3);
rot_mat = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
hold on

%% camera initilization
% 
% roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch

% Connect to the camera.
imgSub = rossubscriber('/raspicam_node/image/compressed');
imgMsg = receive(imgSub);
imgMsg.Format = 'rgb8; jpeg compressed rgb8';

vel_pub = rospublisher('/cmd_vel');
velmsg = rosmessage(vel_pub);
x = 0.0;
velmsg.Angular.Z = 0.2;

green_hue_range = [0.20, 0.50];
green_saturation_range = [0.2 1];
green_value_range = [0.2 1];

%sound_pub = rospublisher('/sound');
%soundmsg = rosmessage(sound_pub);

%img = readImage(imgMsg);
%imshow(img)

Z = inf;




%% monte carlo
mcl = monteCarloLocalization;
mcl.UseLidarScan = true;

sm = likelihoodFieldSensorModel;
sm.Map = occupancyMap(map,20);
mcl.SensorModel = sm;

ranges = 10*ones(1,300);
ranges(1,130:170) = 1.0;
angles = linspace(-pi/2,pi/2,300);
odometryPose = [0 0 0];

scan = lidarScan(ranges,angles);
%% object dettection init
    vel_pub = rospublisher('/cmd_vel');
    velmsg = rosmessage(vel_pub);
    odomSub = rossubscriber("/odom","DataFormat","struct");
    scansub = rossubscriber('/scan');
%%



r_pose = receive(odomSub);
[isUpdated,estimatedPose,covariance] = mcl(r_pose,scan)
estimatedPose

%% MAIN CODE
figure;
while( distanceToGoal > goalRadius )
    


    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    velmsg.Angular.Z = omega;	% Angular velocity (rad/s)
    velmsg.Linear.X = v;
    send(vel_pub,velmsg);

    
    
    % Get the robot's velocity using controller inputs
    r_pose = receive(odomSub);
    [isUpdated,estimatedPose,covariance] = mcl(r_pose,scan)
    position_x = estimatedPose.Pose.Pose.Position.X;        %X_POS
    position_y = estimatedPose.Pose.Pose.Position.Y;        %y_POS
    position = [position_x position_y];
    x = estimatedPose.Pose.Pose.Orientation.X;
    y = estimatedPose.Pose.Pose.Orientation.Y;
    z = estimatedPose.Pose.Pose.Orientation.Z;
    w = estimatedPose.Pose.Pose.Orientation.W;
    eul = quat2eul([x y z w]);
    orientation = eul(3);                            %VINKEL

    robotCurrentPose = [position orientation]' - init_pose + [0 0 FREDE_OG_ASKES_INIT_O]'            %OUTPUTS X_POS Y_POS VINKEL(rad)  -shIFTED START
    %vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    %robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    s(end+1,:) = robotCurrentPose(1:2);
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path1(:,1), path1(:,2),"k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    robotCurrentPose_temp = robotCurrentPose
    waitfor(vizRate);



    linescan = receive(scansub); %Receive message
    ranges = linescan.Ranges; % Extract scan
    angles = linescan.AngleMin:linescan.AngleIncrement:linescan.AngleMax;
    obstacle = ranges([1:15, 345:360]);
    
    %if mean(obstacle) < 0.25
        %a=1
%         t0 = clock;
%         while etime(clock, t0) < 3
%             velmsg.Linear.X = 0.0;
%             velmsg.Angular.Z = 0.5;
%             send(vel_pub,velmsg);
%         end
%     end
end














