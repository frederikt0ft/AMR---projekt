rosshutdown
%connect 1 den
setenv('ROS_MASTER_URI','http://192.168.106.58:11311')
% 2 mig
setenv('ROS_IP','192.168.106.157')
% 3 den mig
rosinit('http://192.168.106.58:11311','NodeHost','192.168.106.157')

%%   ----LOAD MAP-----
% Load the image
img = imread(['real_map_v5.png']);
% Convert the image to grayscale
gray_img = rgb2gray(img);
% Convert the grayscale image to a binary image
binary_img = imbinarize(gray_img);
    
% Create a binary occupancy map from the binary image
scale_factor = 19;   %500 pixels pr. meter
map = binaryOccupancyMap(binary_img,scale_factor);

% Create a new binary occupancy map from the resized matrix
new_map = binaryOccupancyMap(map);
mapMCL = binaryOccupancyMap(map);

% Show the map
show(new_map)
%%  ----LAVER NODES---------
%Create points
NumNodes = 4500;
prmComplex = mobileRobotPRM(new_map,NumNodes);
prmComplex.ConnectionDistance = 2;
show(prmComplex)


%% -----LAVER PATH 1-------

startLocation1 = [47.0 26.0];

endLocation1 = [8 10];
path1 = findpath(prmComplex,startLocation1,endLocation1)
show(prmComplex)

%% -----LAVER PATH 2-------

endLocation2 = [32.5 3.5];
path2 = findpath(prmComplex,endLocation1,endLocation2);
show(prmComplex)
%%  Show planned path 1 and 2
%1st path
figure
plot(path1(:,1), path1(:,2),'k--d')
xlim([-0 50])
ylim([-0 30])
 
%2nd path
hold on
plot(path2(:,1), path2(:,2),'k--d')
xlim([-0 50])
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
controller.DesiredLinearVelocity = 0.7;
controller.MaxAngularVelocity = 0.5;
controller.LookaheadDistance = 0.9;
goalRadius = 0.1;
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
figure
%show(new_map);

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
close all
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




%% monte carlo initialization

    odometryModel = odometryMotionModel;
    odometryModel.Noise = [0.2 0.2 0.2 0.2];

    rangeFinderModel = likelihoodFieldSensorModel;
    rangeFinderModel.SensorLimits = [0.45 3];
    rangeFinderModel.Map = mapMCL;

    % Query the Transformation Tree (tf tree) in ROS.
    tftree = rostf;
    waitForTransform(tftree,'base_footprint','odom');
    sensorTransform = getTransform(tftree,'base_footprint', 'odom');

    % Get the euler rotation angles.
    laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
    laserRotation = quat2eul(laserQuat, 'ZYX');

    rangeFinderModel.SensorPose = [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];

    % Receive sensor measurement
    laserSub = rossubscriber('/scan');
    [velPub,velMsg] = rospublisher('/cmd_vel','geometry_msgs/Twist');

    % Define Monte Carlo

    amcl = monteCarloLocalization;
    amcl.UseLidarScan = true;

    % Assign the MotionModel and SensorModel properties in the amcl object.
    amcl.MotionModel = odometryModel;
    amcl.SensorModel = rangeFinderModel;

    % The particle filter only updates the particles when the robot's movement exceeds
    % the UpdateThresholds
    amcl.UpdateThresholds = [0.11,0.11,0.11];
    amcl.ResamplingInterval = 1;

    % Configure AMCL Object for Localization with Initial Pose Estimate.
    amcl.ParticleLimits = [500 5000];
    amcl.GlobalLocalization = false;
    amcl.InitialPose = [startLocation1 0];
    amcl.InitialCovariance = eye(3)*0.5;


    % Setup Helper for Visualization and Driving TurtleBot.
    visualizationHelper = ExampleHelperAMCLVisualization(map);
    wanderHelper = ExampleHelperAMCLWanderer(laserSub, sensorTransform, velPub, velMsg);
    

    lidarSub = rossubscriber("/scan","DataFormat","struct");








%% object dettection init
    vel_pub = rospublisher('/cmd_vel');
    velmsg = rosmessage(vel_pub);
    odomSub = rossubscriber("/odom","DataFormat","struct");
    scansub = rossubscriber('/scan');
%% 

r_pose = receive(odomSub);
    position_x = r_pose.Pose.Pose.Position.X;        %X_POS
    position_y = r_pose.Pose.Pose.Position.Y;        %y_POS
    position = [position_x position_y];
    x = r_pose.Pose.Pose.Orientation.X;
    y = r_pose.Pose.Pose.Orientation.Y;
    z = r_pose.Pose.Pose.Orientation.Z;
    w = r_pose.Pose.Pose.Orientation.W;
    eul = quat2eul([x y z w]);
    orientation = eul(3);                            %VINKEL

    robotCurrentPose = [position orientation]' - [8 10 0]';

robotCurrentPose_temp = robotCurrentPose


%% MAIN CODE
while( distanceToGoal > goalRadius )

    % Collect the lidar information 
    scan = receive(lidarSub);
    [data, scan_angle] = rosReadCartesian(scan,'RangeLimits', [0.12 3.5]);
    scan_x = data(:,1);
    scan_y = data(:,2);

    % Compute the distance to the closest wall
    scan_dist = sqrt(scan_x.^2 + scan_y.^2);
    [scan_minDist, scan_minIndex] = min(scan_dist);

    % Compute the angle to the closest wall
    degrees = rad2deg(scan_angle);
    scan_minDeg = degrees(scan_minIndex);

    disp(['Min Dist, Min Deg ', '[',num2str(scan_minDist), ' ', num2str(scan_minDeg),']']);
        

    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose_temp);
    velmsg.Angular.Z = omega;	% Angular velocity (rad/s)
    velmsg.Linear.X = v;
    send(vel_pub,velmsg);
    
    % Get the robot's velocity using controller inputs
    r_pose = receive(odomSub);
    position_x = r_pose.Pose.Pose.Position.X;        %X_POS
    position_y = r_pose.Pose.Pose.Position.Y;        %y_POS
    position = [position_x position_y];
    x = r_pose.Pose.Pose.Orientation.X;
    y = r_pose.Pose.Pose.Orientation.Y;
    z = r_pose.Pose.Pose.Orientation.Z;
    w = r_pose.Pose.Pose.Orientation.W;
    eul = quat2eul([x y z w]);
    orientation = eul(3);                            %VINKEL

    robotCurrentPose = [position orientation]' - init_pose; %+ [0 0 FREDE_OG_ASKES_INIT_O]' %OUTPUTS X_POS Y_POS VINKEL(rad)  -shIFTED START
    
    % Receive laser scan and odometry message.
    scanMsg = receive(laserSub);
    odompose = odomSub.LatestMessage;


    % Create lidarScan object to pass to the AMCL object.
    scan = lidarScan(scanMsg);
    [isUpdated, estimatedPose, estimatedCovariance] = amcl(robotCurrentPose, scan);

    error = estimatedPose - [startLocation1, 0];

    % robotCurrentPose_updated = robotCurrentPose - error'

    %[v, omega] = controller(error');

    % robotCurrentPose = robotCurrentPose_updated
    robotCurrentPose = estimatedPose';

    distanceToGoal = norm(estimatedPose(1:2) - robotGoal)








    
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
    plot(path2(:,1), path2(:,2),"k--d")
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
     
%      if mean(obstacle) < 0.25
%          t0 = clock;
%          while etime(clock, t0) < 3
%              velmsg.Linear.X = 0.0;
%              velmsg.Angular.Z = 0.5;
%              send(vel_pub, velmsg);
%          end
%      end

    
end

fprintf("first GOAL DONE")

figure;

while Z > 0.50

    
    
    imgMsg = receive(imgSub);
    imgMsg.Format = 'rgb8; jpeg compressed rgb8';
    img = readImage(imgMsg);            % Aqcuire webcam image
    
    img_mirror = flip(img,2);
    img_flip = flip(img_mirror,1);
    img_hsv = rgb2hsv(img_flip);
    
    img_green = img_hsv(:, :, 1) >= green_hue_range(1) & img_hsv(:, :, 1) <= green_hue_range(2) ...
        & img_hsv(:, :, 2) >= green_saturation_range(1) & img_hsv(:, :, 2) <= green_saturation_range(2) ...
        & img_hsv(:, :, 3) >= green_value_range(1) & img_hsv(:, :, 3) <= green_value_range(2);
    
    [centers, radii] = imfindcircles(img_green, [35 1000], "Sensitivity", 0.95);
    
    send(vel_pub,velmsg);
    
    if centers > 0
        Z = (500*0.09)/radii(1);
        text_str = ['distance: ' num2str(Z,'%0.2f') 'm'];
        img_1 = insertText(img_flip, [77 107],text_str, FontSize=18,BoxColor="red",...
        BoxOpacity=0.4,TextColor="white");
        imshow(img_1)
        hold on
        viscircles(centers(1, :), radii(1), 'color', 'r');

        % The center is 320

        if (Z < 0.04)
            velmsg.Linear.X = 0;

        elseif (320 > centers(1))
            velmsg.Angular.Z = 0.1;
            velmsg.Linear.X = 0;

            if (150 < centers(1))
                velmsg.Linear.X = 0.05;

            elseif (300 < centers(1))
                velmsg.Angular.Z = 0;
            end

        elseif (320 < centers(1))
            velmsg.Angular.Z = -0.1;
            velmsg.Linear.X = 0;

            if (490 > centers(1))
                velmsg.Linear.X = 0.05;

            elseif (340 > centers(1))
                velmsg.Angular.Z = 0;
            end
        end
    
    else
        velmsg.Linear.X = 0;
        velmsg.Angular.Z = 0.2;
        imshow(img_flip)

    end
end

t0 = clock;
while etime(clock, t0) < 3
    velmsg.Linear.X = 0.0;
    velmsg.Angular.Z = 0.5;
    send(vel_pub,velmsg);
end

%%
robotCurrentPose = [position orientation]' - endLocation1;
% ----------- INITILIZATION ---------
robotInitialLocation = path2(1,:);
robotGoal = path2(end,:);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]' - endLocation1;;
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

% -----------Controller------------- 
controller = controllerPurePursuit;
controller.Waypoints = path2;
controller.DesiredLinearVelocity = 0.1;
controller.MaxAngularVelocity = 0.5;
controller.LookaheadDistance = 0.2;
goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

% --- Test ---
odomSub = rossubscriber("/odom","DataFormat","struct");
r_pose = receive(odomSub);
fprintf("-----------------------------------------------")
init_position = [r_pose.Pose.Pose.Position.X r_pose.Pose.Pose.Position.Y];
r_init_orientation = [r_pose.Pose.Pose.Orientation.X r_pose.Pose.Pose.Orientation.Y r_pose.Pose.Pose.Orientation.Z r_pose.Pose.Pose.Orientation.W];
eul = quat2eul(r_init_orientation);
init_orientation = eul(3); %Venstre om 0 til pi. Højre om 0 til -pi
init_pose = [init_position init_orientation]'

FREDE_OG_ASKES_INIT_O = init_orientation

% --- GØR TURLTEBOT KLAR TIL KAMP ---
% TIL TURTLEBOT - DEFINITIVT!
vel_pub = rospublisher('/cmd_vel');
velmsg = rosmessage(vel_pub);
odomSub = rossubscriber("/odom","DataFormat","struct");

% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure
show(new_map);

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth;
s = path2(1,:)

current_pose = robotCurrentPose
r_pose = receive(odomSub);

init_position = [r_pose.Pose.Pose.Position.X r_pose.Pose.Pose.Position.Y] - [endLocation1]     %INITITAL POS DETERMINED
r_init_orientation = [r_pose.Pose.Pose.Orientation.X r_pose.Pose.Pose.Orientation.Y r_pose.Pose.Pose.Orientation.Z r_pose.Pose.Pose.Orientation.W]

eul = quat2eul(r_init_orientation);
init_orientation = eul(3);                                                    %INITAL ORIENTATION DETERMINED
init_pose = [init_position init_orientation]';

theta = eul(3);
rot_mat = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];



% camera initilization
% 
close all
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

%

while( distanceToGoal > goalRadius )
    

    % Collect the lidar information 
    scan = receive(lidarSub);
    [data, scan_angle] = rosReadCartesian(scan,'RangeLimits', [0.12 3.5]);
    scan_x = data(:,1);
    scan_y = data(:,2);

    % Compute the distance to the closest wall
    scan_dist = sqrt(scan_x.^2 + scan_y.^2);
    [scan_minDist, scan_minIndex] = min(scan_dist);

    % Compute the angle to the closest wall
    degrees = rad2deg(scan_angle);
    scan_minDeg = degrees(scan_minIndex);

    disp(['Min Dist, Min Deg ', '[',num2str(scan_minDist), ' ', num2str(scan_minDeg),']']);
        

    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose_temp);
    velmsg.Angular.Z = omega;	% Angular velocity (rad/s)
    velmsg.Linear.X = v;
    send(vel_pub,velmsg);
    
    % Get the robot's velocity using controller inputs
    r_pose = receive(odomSub);
    position_x = r_pose.Pose.Pose.Position.X;        %X_POS
    position_y = r_pose.Pose.Pose.Position.Y;        %y_POS
    position = [position_x position_y];
    x = r_pose.Pose.Pose.Orientation.X;
    y = r_pose.Pose.Pose.Orientation.Y;
    z = r_pose.Pose.Pose.Orientation.Z;
    w = r_pose.Pose.Pose.Orientation.W;
    eul = quat2eul([x y z w]);
    orientation = eul(3);                            %VINKEL

    robotCurrentPose = [position orientation]' - [8 10 0]'; %+ [0 0 FREDE_OG_ASKES_INIT_O]' %OUTPUTS X_POS Y_POS VINKEL(rad)  -shIFTED START
    
    % Receive laser scan and odometry message.
    scanMsg = receive(laserSub);
    odompose = odomSub.LatestMessage;


    % Create lidarScan object to pass to the AMCL object.
    scan = lidarScan(scanMsg);
    [isUpdated, estimatedPose, estimatedCovariance] = amcl(robotCurrentPose, scan);

    error = estimatedPose - [startLocation1, 0];

    % robotCurrentPose_updated = robotCurrentPose - error'

    %[v, omega] = controller(error');

    % robotCurrentPose = robotCurrentPose_updated
    robotCurrentPose = estimatedPose';

    distanceToGoal = norm(estimatedPose(1:2) - robotGoal)








    
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
    plot(path2(:,1), path2(:,2),"k--d")
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
     
%      if mean(obstacle) < 0.25
%          t0 = clock;
%          while etime(clock, t0) < 3
%              velmsg.Linear.X = 0.0;
%              velmsg.Angular.Z = 0.5;
%              send(vel_pub, velmsg);
%          end
%      end

    



end

figure;

%
while Z > 0.50

    
    
    imgMsg = receive(imgSub);
    imgMsg.Format = 'rgb8; jpeg compressed rgb8';
    img = readImage(imgMsg);            % Aqcuire webcam image
    
    img_mirror = flip(img,2);
    img_flip = flip(img_mirror,1);
    img_hsv = rgb2hsv(img_flip);
    
    img_green = img_hsv(:, :, 1) >= green_hue_range(1) & img_hsv(:, :, 1) <= green_hue_range(2) ...
        & img_hsv(:, :, 2) >= green_saturation_range(1) & img_hsv(:, :, 2) <= green_saturation_range(2) ...
        & img_hsv(:, :, 3) >= green_value_range(1) & img_hsv(:, :, 3) <= green_value_range(2);
    
    [centers, radii] = imfindcircles(img_green, [35 1000], "Sensitivity", 0.95);
    
    send(vel_pub,velmsg);
    
    if centers > 0
        Z = (500*0.09)/radii(1);
        text_str = ['distance: ' num2str(Z,'%0.2f') 'm'];
        img_1 = insertText(img_flip, [77 107],text_str, FontSize=18,BoxColor="red",...
        BoxOpacity=0.4,TextColor="white");
        imshow(img_1)
        hold on
        viscircles(centers(1, :), radii(1), 'color', 'r');

        % The center is 320

        if (Z < 0.04)
            velmsg.Linear.X = 0;

        elseif (320 > centers(1))
            velmsg.Angular.Z = 0.1;
            velmsg.Linear.X = 0;

            if (150 < centers(1))
                velmsg.Linear.X = 0.05;

            elseif (300 < centers(1))
                velmsg.Angular.Z = 0;
            end

        elseif (320 < centers(1))
            velmsg.Angular.Z = -0.1;
            velmsg.Linear.X = 0;

            if (490 > centers(1))
                velmsg.Linear.X = 0.05;

            elseif (340 > centers(1))
                velmsg.Angular.Z = 0;
            end
        end
    
    else
        velmsg.Linear.X = 0;
        velmsg.Angular.Z = 0.2;
        imshow(img_flip)

    end
end

t0 = clock;
while etime(clock, t0) < 6
    velmsg.Linear.X = 0.0;
    velmsg.Angular.Z = 0.5;
    send(vel_pub,velmsg);
end















