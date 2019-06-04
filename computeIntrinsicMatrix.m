function cameraParams = computeIntrinsicMatrix(image)
%COMPUTEINTRINSICMATRIX Returns a cameraParameters object containing the intrinsic
%parameters of the camera.
    data = imfinfo(image);
    
    sensorX = 22.3;
    sensorY = 14.9;

    x = data.DigitalCamera.CPixelXDimension;
    y = data.DigitalCamera.CPixelYDimension;
    focalLength = data.DigitalCamera.FocalLength;
    
    px = sensorX / x;
    py = sensorY / y;
    
    fx = focalLength / px;
    fy = focalLength / py;
    
    cx = round(x / 2);
    cy = round(y / 2);
    
    cameraParams = cameraIntrinsics([fx, fy], [cx, cy], [x, y]);
end

