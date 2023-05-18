rosshutdown
%connect 1 den
setenv('ROS_MASTER_URI','http://192.168.5.186:11311')
% 2 mig
setenv('ROS_IP','192.168.5.155')
% 3 den mig
rosinit('http://192.168.5.186:11311','NodeHost','192.168.5.155')

%%   ----LOAD MAP-----
% Load the image
img = imread('C:\Users\frede\OneDrive\Dokumenter\6_semester\ANS\Projekt\Testmap1.png');
% Convert the image to grayscale
gray_img = rgb2gray(img);
% Convert the grayscale image to a binary image
binary_img = imbinarize(gray_img);
    
% Create a binary occupancy map from the binary image
scale_factor = 500;   %500 pixels pr. meter
map = binaryOccupancyMap(binary_img,scale_factor);

% Create a new binary occupancy map from the resized matrix
new_map = binaryOccupancyMap(map);

% Show the map
show(new_map);
%%  ----LAVER NODES---------
%Create points
NumNodes = 300;
prmComplex = mobileRobotPRM(new_map,NumNodes);
prmComplex.ConnectionDistance = 0.6;
show(prmComplex)
%% -----LAVER PATH 1-------
%Define start location

startLocation1 = [1.0 0.15];
endLocation1 = [1 0.9];
path1 = findpath(prmComplex,startLocation1,endLocation1)
show(prmComplex)


%% -----LAVER PATH 2-------
startLocation2 = [1 1.1];
endLocation2 = [1.5 1.9];
path2 = findpath(prmComplex,startLocation2,endLocation2);
show(prmComplex)
%%  Show planned path 1 and 2
%1st path
figure
plot(path1(:,1), path1(:,2),'k--d')
xlim([-0 2])
ylim([-0 2])
 
%2nd path
hold on
plot(path2(:,1), path2(:,2),'k--d')
xlim([-0 2])
ylim([-0 2])
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
controller.DesiredLinearVelocity = 0.1;
controller.MaxAngularVelocity = 0.5;
controller.LookaheadDistance = 0.2;
goalRadius = 0.3;
distanceToGoal = norm(robotInitialLocation - robotGoal);

%% --- Test ---
odomSub = rossubscriber("/odom","DataFormat","struct");
r_pose = receive(odomSub);
fprintf("-----------------------------------------------")
init_position = [r_pose.pose.pose.position.x r_pose.pose.pose.position.y];
r_init_orientation = [r_pose.pose.pose.orientation.x r_pose.pose.pose.orientation.y r_pose.pose.pose.orientation.z r_pose.pose.pose.orientation.w];
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
show(new_map);

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/5;
s = path1(1,:)

current_pose = robotCurrentPose
r_pose = receive(odomSub);

init_position = [r_pose.pose.pose.position.x r_pose.pose.pose.position.y] - [startLocation1]     %INITITAL POS DETERMINED
r_init_orientation = [r_pose.pose.pose.orientation.x r_pose.pose.pose.orientation.y r_pose.pose.pose.orientation.z r_pose.pose.pose.orientation.w]

eul = quat2eul(r_init_orientation);
init_orientation = eul(3);                                                    %INITAL ORIENTATION DETERMINED
init_pose = [init_position init_orientation]';

theta = eul(3);
rot_mat = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
hold on
%% ------- GO COMMAND -----------------------------
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    velmsg.Angular.Z = omega;	% Angular velocity (rad/s)
    velmsg.Linear.X = v;
    send(vel_pub,velmsg);
    
    % Get the robot's velocity using controller inputs
    r_pose = receive(odomSub);
    position_x = r_pose.pose.pose.position.x;        %X_POS
    position_y = r_pose.pose.pose.position.y;        %y_POS
    position = [position_x position_y];
    x = r_pose.pose.pose.orientation.x;
    y = r_pose.pose.pose.orientation.y;
    z = r_pose.pose.pose.orientation.z;
    w = r_pose.pose.pose.orientation.w;
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
end
fprintf("first GOAL DONE")
%---------------------------------PATH 2---------------------------------

%%
robotInitialLocation = path2(1,:);
robotGoal = path2(end,:);
distanceToGoal = norm(robotInitialLocation - robotGoal);
s = path2(1,:)
current_pose = robotCurrentPose
r_pose = receive(odomSub);

init_position = [r_pose.pose.pose.position.x r_pose.pose.pose.position.y] - [robotCurrentPose_temp(1) robotCurrentPose_temp(2)];     %INITITAL POS DETERMINED
r_init_orientation = [r_pose.pose.pose.orientation.x r_pose.pose.pose.orientation.y r_pose.pose.pose.orientation.z r_pose.pose.pose.orientation.w]

eul = quat2eul(r_init_orientation);
init_orientation = eul(3);                                                    %INITAL ORIENTATION DETERMINED
init_pose = [init_position init_orientation]';

theta = eul(3);
rot_mat = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];


odomSub = rossubscriber("/odom","DataFormat","struct");
r_pose = receive(odomSub);
fprintf("-----------------------------------------------")
init_position = [r_pose.pose.pose.position.x r_pose.pose.pose.position.y] - [robotCurrentPose_temp(1) robotCurrentPose_temp(2)];
r_init_orientation = [r_pose.pose.pose.orientation.x r_pose.pose.pose.orientation.y r_pose.pose.pose.orientation.z r_pose.pose.pose.orientation.w];
eul = quat2eul(r_init_orientation);
init_orientation = eul(3); %Venstre om 0 til pi. Højre om 0 til -pi
init_pose = [init_position init_orientation]'

FREDE_OG_ASKES_INIT_O = init_orientation


while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    velmsg.Angular.Z = omega;	% Angular velocity (rad/s)
    velmsg.Linear.X = v;
    send(vel_pub,velmsg);
    
    % Get the robot's velocity using controller inputs
    r_pose = receive(odomSub);
    position_x = r_pose.pose.pose.position.x;        %X_POS
    position_y = r_pose.pose.pose.position.y;        %y_POS
    position = [position_x position_y];
    x = r_pose.pose.pose.orientation.x;
    y = r_pose.pose.pose.orientation.y;
    z = r_pose.pose.pose.orientation.z;
    w = r_pose.pose.pose.orientation.w;
    eul = quat2eul([x y z w]);
    orientation = eul(3);                            %VINKEL

    robotCurrentPose = [position orientation]' - init_pose + [0 0 FREDE_OG_ASKES_INIT_O]'            %OUTPUTS X_POS Y_POS VINKEL(rad)
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

   
    waitfor(vizRate);
end


