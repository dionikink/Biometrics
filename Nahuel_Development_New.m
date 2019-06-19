%% Import images
clear all;
imageDir = fullfile('RomanEmperors', 'Brutus', 'Photos');
imds = imageDatastore(imageDir);

%% Display the images.
% figure
% montage(imds.Files, 'Size', [5, 4]);
% title('Input Image Sequence');

%% Convert the images to grayscale.
% Images now holds all the images in a cell
images = cell(1, numel(imds.Files));

for i = 1:numel(imds.Files)
    I = readimage(imds, i);
    images{i} = imresize(rot90(rgb2gray(I),3),0.5);
end

%% Load camera parameters
cameraParams=computeIntrinsicMatrix(imds.Files{1});

%% Position camera of first image
% Undistort the first image.
%           I = undistortImage(images{1}, cameraParams); 
I = images{1};
% Detect features. Increasing 'NumOctaves' helps detect large-scale
% features in high-resolution images. Use an ROI to eliminate spurious
% features around the edges of the image.
border = 50;
roi = [border, border, size(I, 2)- 2*border, size(I, 1)- 2*border];
Octaves = 15;
%prevPoints   = detectSURFFeatures(I, 'NumOctaves', Octaves, 'ROI', roi);
prevPoints = detectMSERFeatures(I,'ROI', roi, 'RegionAreaRange', [400 14000], 'ThresholdDelta', 1)
% imshow(I);
% hold on;
% plot(prevPoints);
% Extract features. Using 'Upright' features improves matching, as long as
% the camera motion involves little or no in-plane rotation.
prevFeatures = extractFeatures(I, prevPoints, 'Upright', true);

% Create an empty viewSet object to manage the data associated with each
% view.
vSet = viewSet;

%   Add the first view. Place the camera associated with the first view
%   at the origin, oriented along the Z-axis.
viewId = 1;
vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', ...
    eye(3, 'like', prevPoints.Location), 'Location', ...
    zeros(1, 3, 'like', prevPoints.Location));

%% Add the other cameras
prevI = images{1};
for i = 2:3%numel(images)
    % Undistort the current image.
    %I = undistortImage(images{i}, cameraParams);
    I = images{i};
    
    % Detect, extract and match features.
    %currPoints   = detectSURFFeatures(I, 'NumOctaves', Octaves, 'ROI', roi);
    currPoints = detectMSERFeatures(I,'ROI', roi, 'RegionAreaRange', [300 14000], 'ThresholdDelta', 0.8)
%     figure
%     imshow(I);
%     hold on;
%     plot(currPoints);
    currFeatures = extractFeatures(I, currPoints, 'Upright', true); 
    indexPairs = matchFeatures(prevFeatures, currFeatures, 'Method', 'Approximate', ...
        'MaxRatio', 1, 'MatchThreshold', 1, 'Unique',  true);
    % Select matched points.
    matchedPoints1 = prevPoints(indexPairs(:, 1));
    matchedPoints2 = currPoints(indexPairs(:, 2));
    [~, inlierPoints1, inlierPoints2] = estimateGeometricTransform(matchedPoints1, matchedPoints2, ...
        'projective', 'MaxDistance', 500)
    figure; showMatchedFeatures(prevI,I,matchedPoints1,matchedPoints2,'montage');
    legend('matched points 1','matched points 2');
    figure; showMatchedFeatures(prevI,I,inlierPoints1,inlierPoints2,'montage');
    legend('matched points 1','matched points 2');
    
    
    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [relativeOrient, relativeLoc, inlierIdx] = Devel_helperEstimateRelativePose(...
        inlierPoints1, inlierPoints2, cameraParams);
    
    % Add the current view to the view set.
    vSet = addView(vSet, i, 'Points', currPoints);
    
    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, 'Matches', indexPairs(inlierIdx,:));
    
    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1);
    prevOrientation = prevPose.Orientation{1};
    prevLocation    = prevPose.Location{1};
        
    % Compute the current camera pose in the global coordinate system 
    % relative to the first view.
    orientation = relativeOrient * prevOrientation;
    location    = prevLocation + relativeLoc * prevOrientation;
    vSet = updateView(vSet, i, 'Orientation', orientation, ...
        'Location', location);
    
    % Find point tracks across all views.
    tracks = findTracks(vSet);
        % HIER IS ALLES AL KAPOT
    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);
    
    % Refine the 3-D world points and camera poses.
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
        'PointsUndistorted', true);

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevFeatures = currFeatures;
    prevPoints   = currPoints;  
    prevI = I;
end

%% Plot poses of cameras
camPoses = poses(vSet);
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on;
pcshow(xyzPoints, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);

%% Dense reconstruction

% Read and undistort the first image
% I = undistortImage(images{1}, cameraParams); 

% Detect corners in the first image.
prevPoints = detectMinEigenFeatures(I, 'MinQuality', 0.001);

% Create the point tracker object to track the points across views.
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 6);

% Initialize the point tracker.
prevPoints = prevPoints.Location;
initialize(tracker, prevPoints, I);

% Store the dense points in the view set.
vSet = updateConnection(vSet, 1, 2, 'Matches', zeros(0, 2));
vSet = updateView(vSet, 1, 'Points', prevPoints);

% Track the points across all views.
for i = 2:numel(images)
    i
    % Read and undistort the current image.
%    I = undistortImage(images{i}, cameraParams); 
    
    % Track the points.
    [currPoints, validIdx] = step(tracker, I);
    
    % Clear the old matches between the points.
    if i < numel(images)
        vSet = updateConnection(vSet, i, i+1, 'Matches', zeros(0, 2));
    end
    vSet = updateView(vSet, i, 'Points', currPoints);
    
    % Store the point matches in the view set.
    matches = repmat((1:size(prevPoints, 1))', [1, 2]);
    matches = matches(validIdx, :);        
    vSet = updateConnection(vSet, i-1, i, 'Matches', matches);
end

% Find point tracks across all views.
tracks = findTracks(vSet);

% Find point tracks across all views.
camPoses = poses(vSet);

% Triangulate initial locations for the 3-D world points.
xyzPoints = triangulateMultiview(tracks, camPoses,...
    cameraParams);

% Refine the 3-D world points and camera poses.
[xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(...
    xyzPoints, tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
    'PointsUndistorted', true);

%% Plot Dense reconstruction
% Display the refined camera poses.
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on

% Exclude noisy 3-D world points.
goodIdx = (reprojectionErrors < 5);

% Display the dense 3-D world points.
pcshow(xyzPoints(goodIdx, :), 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on
hold off

% Specify the viewing volume.
loc1 = camPoses.Location{1};
xlim([loc1(1)-5, loc1(1)+4]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+20]);
camorbit(0, -30);

title('Dense Reconstruction');