%roslaunch turtlebot3_bringup turtlebot3_robot.launch
% assuming the Turtlebot ip is 192.168.1.200
setenv('ROS_MASTER_URI','http://192.168.87.43:11311')
% assuming your own ip is 192.168.1.100
setenv('ROS_IP','192.168.87.164')
rosinit('http://192.168.87.43:11311','NodeHost','192.168.87.164');
%% 
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
%% Real time analysis of video

figure;
while Z > 0.45
    
    imgMsg = receive(imgSub);
    imgMsg.Format = 'rgb8; jpeg compressed rgb8';
    img = readImage(imgMsg);            % Aqcuire webcam image
    
    img_mirror = flip(img,2);
    img_flip = flip(img_mirror,1);
    img_hsv = rgb2hsv(img_flip);
    
    img_green = img_hsv(:, :, 1) >= green_hue_range(1) & img_hsv(:, :, 1) <= green_hue_range(2) ...
        & img_hsv(:, :, 2) >= green_saturation_range(1) & img_hsv(:, :, 2) <= green_saturation_range(2) ...
        & img_hsv(:, :, 3) >= green_value_range(1) & img_hsv(:, :, 3) <= green_value_range(2);
    
    [centers, radii] = imfindcircles(img_green, [25 1000], "Sensitivity", 0.95);
    
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

%% Real time analysis of video mark 2

velmsg.Angular.Z = 0.1;
velmsg.Linear.X = 0;

figure;
while Z > 0.4
    
    imgMsg = receive(imgSub);
    imgMsg.Format = 'rgb8; jpeg compressed rgb8';
    img = readImage(imgMsg);            % Aqcuire webcam image
    
    img_mirror = flip(img,2);
    img_flip = flip(img_mirror,1);
    img_hsv = rgb2hsv(img_flip);
    
    img_green = img_hsv(:, :, 1) >= green_hue_range(1) & img_hsv(:, :, 1) <= green_hue_range(2) ...
        & img_hsv(:, :, 2) >= green_saturation_range(1) & img_hsv(:, :, 2) <= green_saturation_range(2) ...
        & img_hsv(:, :, 3) >= green_value_range(1) & img_hsv(:, :, 3) <= green_value_range(2);
    
    [centers, radii] = imfindcircles(img_green, [25 1000], "Sensitivity", 0.95);
    
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
        
        if (300 < center(1)) && (340 > center(1))
            
            while(1)
                velmsg.Linear.Z = 0.0;
                velmsg.Linear.X = 0.1;
                send(vel_pub,velmsg);

                if Z < 0.45
                    break;
                end
            end
        end

    else
        velmsg.Linear.X = 0;
        velmsg.Angular.Z = 0.1;
        imshow(img_flip)

    end
end

%%
scansub = rossubscriber('/scan');
figure;
for i = 1:1000
    linescan = receive(scansub);
    ranges = linescan.Ranges;

    angles = linescan.AngleMin:linescan.AngleIncrement:linescan.AngleMax;

    plot(rad2deg(angles), ranges)
end

%%
d
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

img = readImage(imgMsg);

figure;
    
img_mirror = flip(img,2);
img_flip = flip(img_mirror,1);
img_hsv = rgb2hsv(img_flip);
    
img_green = img_hsv(:, :, 1) >= green_hue_range(1) & img_hsv(:, :, 1) <= green_hue_range(2) ...
    & img_hsv(:, :, 2) >= green_saturation_range(1) & img_hsv(:, :, 2) <= green_saturation_range(2) ...
    & img_hsv(:, :, 3) >= green_value_range(1) & img_hsv(:, :, 3) <= green_value_range(2);
    
[centers, radii] = imfindcircles(img_green, [25 1000], "Sensitivity", 0.95);
    
send(vel_pub,velmsg);

if centers > 0
    imshow(img_flip)
    hold on 
    viscircles(centers(1, :), radii(1), 'color', 'r');
else
    imshow(img_flip)
end

%%
% distance in meters/radius 
first = 1/49;
second = 0.43/103;
third = 0.48/89;


