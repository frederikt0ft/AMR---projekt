close all
clear
clc

%%

webcamlist
webcam

% Connect to selected webcam
cam = webcam(1)

%% Real time analysis of video

figure;
for i = 1:1000

    tic
    
    img = snapshot(cam);            % Aqcuire webcam image
        
    img_hsv = rgb2hsv(img);
    
    green_hue_range = [0.20, 0.50];
    green_saturation_range = [0.2 1];
    green_value_range = [0.2 1];
    
    img_green = img_hsv(:, :, 1) >= green_hue_range(1) & img_hsv(:, :, 1) <= green_hue_range(2) ...
        & img_hsv(:, :, 2) >= green_saturation_range(1) & img_hsv(:, :, 2) <= green_saturation_range(2) ...
        & img_hsv(:, :, 3) >= green_value_range(1) & img_hsv(:, :, 3) <= green_value_range(2);
    
    
    [centers, radii] = imfindcircles(img_green, [50 1000], "Sensitivity", 0.95);

    if centers > 0
        imshow(img)
        hold on
        viscircles(centers(1, :), radii(1), 'color', 'r');
    else
        imshow(img)
    end
    
    toc
end

%%

% Radius of green circle ≈ 9 cm


