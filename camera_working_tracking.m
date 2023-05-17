%roslaunch turtlebot3_bringup turtlebot3_robot.launch
% assuming the Turtlebot ip is 192.168.1.200
setenv('ROS_MASTER_URI','http://192.168.218.180:11311')
% assuming your own ip is 192.168.1.100
setenv('ROS_IP','192.168.218.105')
rosinit('http://192.168.218.180:11311','NodeHost','192.168.218.105');
%% 

% roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch

% Connect to the camera.
imgSub = rossubscriber('/raspicam_node/image/compressed');
imgMsg = receive(imgSub);
imgMsg.Format = 'rgb8; jpeg compressed rgb8';
img = readImage(imgMsg);
imshow(img)

%% Real time analysis of video

figure;
for i = 1:1000
%% 

    tic
    
    imgMsg = receive(imgSub);
    imgMsg.Format = 'rgb8; jpeg compressed rgb8';
    img = readImage(imgMsg);            % Aqcuire webcam image
    
    img_mirror = flip(img,2);
    img_flip = flip(img_mirror,1);
    img_hsv = rgb2hsv(img_flip);
    
    green_hue_range = [0.20, 0.50];
    green_saturation_range = [0.2 1];
    green_value_range = [0.2 1];
    
    img_green = img_hsv(:, :, 1) >= green_hue_range(1) & img_hsv(:, :, 1) <= green_hue_range(2) ...
        & img_hsv(:, :, 2) >= green_saturation_range(1) & img_hsv(:, :, 2) <= green_saturation_range(2) ...
        & img_hsv(:, :, 3) >= green_value_range(1) & img_hsv(:, :, 3) <= green_value_range(2);
    
    
    [centers, radii] = imfindcircles(img_green, [50 1000], "Sensitivity", 0.95)


    if centers > 0
        Z = (500*0.09)/radii(1);
        text_str = ['distance: ' num2str(Z,'%0.2f') 'm'];
        img_1 = insertText(img_flip, [77 107],text_str, FontSize=18,BoxColor="red",...
        BoxOpacity=0.4,TextColor="white");
        imshow(img_1)
        hold on
        viscircles(centers(1, :), radii(1), 'color', 'r');
    else
        imshow(img_flip)

    end
    
    toc
end

%%

% Radius of green circle ≈ 9 cm


