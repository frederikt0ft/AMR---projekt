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
velmsg.Angular.Z = 0.2;
velmsg.Linear.X = 0;

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
        Z = (500 * 0.09)/radii(1);
        text_str = ['distance: ' num2str(Z, '%0.2f') 'm'];
        update_img = insertText(img_flip, [77 107],text_str, FontSize=18,BoxColor="red",...
        BoxOpacity = 0.4, TextColor = "white");
        imshow(update_img)
        hold on
        viscircles(centers(1, :), radii(1), 'color', 'r');

        % function for searching for circles in images and acting
        % accordingly
        [velmsg.Linear.X, velmsg.Angular.Z] = circles_search(centers(1, :));
    
    else
        velmsg.Linear.X = 0;
        velmsg.Angular.Z = 0.2;
        imshow(img_flip)

    end
end

%% Function for searching for circles and acting accordingly

function [Linear, Angular] = circles_search(center)

    % first check wether the circle is left of the center -> act
    if (320 > center)
        Angular = 0.1;
        Linear = 0;
        
        if (150 < center)
            Linear = 0.05;
        
        elseif (300 < center)
            Angular = 0;
        end
    
    % check whether the circle is right of the center -> act
    elseif (320 < center)
        Angular = -0.1;
        Linear = 0;
        
        if (490 > center)
            Linear = 0.05;
        
        elseif (340 > center)
            Angular = 0;
        end

    end
end
