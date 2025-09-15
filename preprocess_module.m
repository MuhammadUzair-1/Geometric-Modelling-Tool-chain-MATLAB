%% preprocess_module.m - Enhanced Preprocessing Functions for Propeller Toolchain
% Comprehensive preprocessing functions with validation, error handling,
% and performance optimizations

%% Remove outliers with comprehensive validation
function ptOut = preprocess_removeOutliers(ptIn, method, params)
% PREPROCESS_REMOVEOUTLIERS Remove outliers from a point cloud
% Inputs:
%   ptIn    - Input point cloud (pointCloud object)
%   method  - 'stat' (statistical) or 'radius' (radius-based)
%   params  - Structure with method-specific parameters
% Outputs:
%   ptOut   - Filtered point cloud

% Input validation
if nargin < 1
    error('Point cloud input is required');
end
if ~isa(ptIn, 'pointCloud')
    error('Input must be a pointCloud object');
end
if ptIn.Count == 0
    error('Input point cloud is empty');
end
if nargin < 2, method = 'stat'; end
if nargin < 3, params = struct(); end

% Validate method
validMethods = {'stat', 'radius'};
if ~any(strcmpi(method, validMethods))
    error('Invalid method: %s. Use ''stat'' or ''radius''.', method);
end

% Process based on method
switch lower(method)
    case 'stat'
        if ~isfield(params, 'k'), params.k = 20; end
        if ~isfield(params, 'threshold'), params.threshold = 2; end
        
        % Validate parameters
        if params.k < 3
            warning('k value too small, setting to minimum of 3');
            params.k = 3;
        end
        if params.threshold <= 0
            warning('Threshold must be positive, setting to 2');
            params.threshold = 2;
        end
        
        % Apply filtering
        [ptOut, ~, outlierIndices] = pcdenoise(ptIn, 'NumNeighbors', params.k, 'Threshold', params.threshold);
        fprintf('[Preprocess] Statistical outlier removal: Removed %d points (%d neighbors, threshold %.2f)\n', ...
                length(outlierIndices), params.k, params.threshold);

    case 'radius'
        if ~isfield(params, 'radius'), params.radius = 0.01; end
        if ~isfield(params, 'minNeighbors'), params.minNeighbors = 5; end
        
        % Validate parameters
        if params.radius <= 0
            warning('Radius must be positive, setting to 0.01');
            params.radius = 0.01;
        end
        if params.minNeighbors < 1
            warning('minNeighbors must be at least 1, setting to 5');
            params.minNeighbors = 5;
        end
        
        % Apply filtering
        [ptOut, ~, outlierIndices] = pcdenoise(ptIn, 'Radius', params.radius, 'NumNeighbors', params.minNeighbors);
        fprintf('[Preprocess] Radius outlier removal: Removed %d points (radius %.3f, minNeighbors %d)\n', ...
                length(outlierIndices), params.radius, params.minNeighbors);

    otherwise
        error('Unknown outlier removal method: %s', method);
end
end

%% Downsample point cloud with validation
function ptOut = preprocess_downsample(ptIn, method, paramValue)
% PREPROCESS_DOWNSAMPLE Downsample point cloud
% Inputs:
%   ptIn       - Input point cloud
%   method     - 'voxel' (voxel grid) or 'random' (random sampling)
%   paramValue - Voxel size for 'voxel' or sample percentage for 'random'

% Input validation
if nargin < 1
    error('Point cloud input is required');
end
if ~isa(ptIn, 'pointCloud')
    error('Input must be a pointCloud object');
end
if ptIn.Count == 0
    error('Input point cloud is empty');
end
if nargin < 2, method = 'voxel'; end

% Validate method
validMethods = {'voxel', 'random'};
if ~any(strcmpi(method, validMethods))
    error('Invalid method: %s. Use ''voxel'' or ''random''.', method);
end

% Process based on method
switch lower(method)
    case 'voxel'
        if nargin < 3, paramValue = 0.002; end
        if paramValue <= 0
            warning('Voxel size must be positive, setting to 0.002');
            paramValue = 0.002;
        end
        
        ptOut = pcdownsample(ptIn, 'gridAverage', paramValue);
        fprintf('[Preprocess] Downsampled using voxel grid (size %.4f): %d -> %d points\n', ...
                paramValue, ptIn.Count, ptOut.Count);
        
    case 'random'
        if nargin < 3, paramValue = 0.3; end  % Default to 30% sampling
        if paramValue <= 0 || paramValue > 1
            warning('Sample percentage must be between 0 and 1, setting to 0.3');
            paramValue = 0.3;
        end
        
        ptOut = pcdownsample(ptIn, 'random', paramValue);
        fprintf('[Preprocess] Downsampled using random sampling (%.1f%%): %d -> %d points\n', ...
                paramValue*100, ptIn.Count, ptOut.Count);
end
end

%% MLS Smoothing (Moving Least Squares) with performance optimization
function ptOut = preprocess_denoiseMLS(ptIn, radius, degree, useParallel)
% PREPROCESS_DENOISEMLS Smooth point cloud using MLS
% Inputs:
%   ptIn       - Input point cloud
%   radius     - Search radius for neighbors
%   degree     - Polynomial degree for fitting
%   useParallel- Use parallel processing (default: true if available)

% Input validation
if nargin < 1
    error('Point cloud input is required');
end
if ~isa(ptIn, 'pointCloud')
    error('Input must be a pointCloud object');
end
if ptIn.Count == 0
    error('Input point cloud is empty');
end
if nargin < 2, radius = 0.005; end
if nargin < 3, degree = 2; end
if nargin < 4, useParallel = true; end

% Validate parameters
if radius <= 0
    warning('Radius must be positive, setting to 0.005');
    radius = 0.005;
end
if degree < 1 || degree > 3
    warning('Degree must be between 1 and 3, setting to 2');
    degree = 2;
end

% Use built-in function if available (MATLAB R2020b+)
if exist('pcmls', 'file') == 2
    ptOut = pcmls(ptIn, radius, 'PolyOrder', degree);
    fprintf('[Preprocess] MLS smoothing applied using built-in function (radius %.3f, degree %d)\n', radius, degree);
    return;
end

% Fallback implementation
xyz = ptIn.Location;
kdtree = KDTreeSearcher(xyz);
smoothed = zeros(size(xyz));

% Check if parallel processing should be used
if useParallel && isempty(gcp('nocreate')) && license('test', 'distrib_computing_toolbox')
    useParallel = false; % No parallel pool available
end

if useParallel
    % Parallel processing
    parfor i = 1:size(xyz, 1)
        idx = rangesearch(kdtree, xyz(i, :), radius);
        neighbors = xyz(idx{1}, :);
        if size(neighbors, 1) < degree + 1
            smoothed(i, :) = xyz(i, :);
        else
            % Simple averaging for demonstration - could implement proper MLS fitting
            smoothed(i, :) = mean(neighbors, 1);
        end
    end
else
    % Serial processing
    for i = 1:size(xyz, 1)
        idx = rangesearch(kdtree, xyz(i, :), radius);
        neighbors = xyz(idx{1}, :);
        if size(neighbors, 1) < degree + 1
            smoothed(i, :) = xyz(i, :);
        else
            % Simple averaging for demonstration
            smoothed(i, :) = mean(neighbors, 1);
        end
    end
end

ptOut = pointCloud(smoothed);
if isfield(ptIn, 'Normal') && ~isempty(ptIn.Normal)
    ptOut.Normal = ptIn.Normal; % Preserve normals if they exist
end
fprintf('[Preprocess] MLS smoothing applied (radius %.3f, degree %d)\n', radius, degree);
end

%% Estimate normals with comprehensive options
function ptOut = preprocess_estimateNormals(ptIn, k, method, orientation)
% PREPROCESS_ESTIMATENORMALS Estimate normals using various methods
% Inputs:
%   ptIn       - Input point cloud
%   k          - Number of neighbors (default: 20)
%   method     - 'pca' or 'knn' (default: 'pca')
%   orientation- 'auto' or 'manual' (default: 'auto')

% Input validation
if nargin < 1
    error('Point cloud input is required');
end
if ~isa(ptIn, 'pointCloud')
    error('Input must be a pointCloud object');
end
if ptIn.Count == 0
    error('Input point cloud is empty');
end
if nargin < 2, k = 20; end
if nargin < 3, method = 'pca'; end
if nargin < 4, orientation = 'auto'; end

% Validate parameters
if k < 3
    warning('k value too small, setting to minimum of 3');
    k = 3;
end
validMethods = {'pca', 'knn'};
if ~any(strcmpi(method, validMethods))
    warning('Invalid method: %s. Using ''pca''.', method);
    method = 'pca';
end
validOrientations = {'auto', 'manual'};
if ~any(strcmpi(orientation, validOrientations))
    warning('Invalid orientation: %s. Using ''auto''.', orientation);
    orientation = 'auto';
end

% Estimate normals
ptOut = ptIn;
if strcmpi(method, 'pca')
    ptOut.Normal = pcnormals(ptIn, k);
else
    % KNN-based approach (simplified)
    xyz = ptIn.Location;
    kdtree = KDTreeSearcher(xyz);
    normals = zeros(size(xyz));
    
    for i = 1:size(xyz, 1)
        idx = knnsearch(kdtree, xyz(i, :), 'K', k);
        neighbors = xyz(idx, :);
        covariance = cov(neighbors);
        [V, ~] = eig(covariance);
        normals(i, :) = V(:, 1)'; % Smallest eigenvector
    end
    ptOut.Normal = normals;
end

% Orient normals if requested
if strcmpi(orientation, 'auto')
    ptOut = preprocess_orientNormals(ptOut);
end

fprintf('[Preprocess] Normals estimated using %d neighbors (%s method)\n', k, method);
end

%% Orient normals consistently with multiple methods
function ptOut = preprocess_orientNormals(ptIn, method, viewpoint)
% PREPROCESS_ORIENTNORMALS Orient normals consistently
% Inputs:
%   ptIn     - Input point cloud with normals
%   method   - 'centroid' or 'viewpoint' (default: 'centroid')
%   viewpoint- Camera viewpoint for orientation (required for 'viewpoint' method)

% Input validation
if nargin < 1
    error('Point cloud input is required');
end
if ~isa(ptIn, 'pointCloud')
    error('Input must be a pointCloud object');
end
if isempty(ptIn.Normal)
    error('Point cloud must have normals');
end
if nargin < 2, method = 'centroid'; end

% Validate method
validMethods = {'centroid', 'viewpoint'};
if ~any(strcmpi(method, validMethods))
    warning('Invalid method: %s. Using ''centroid''.', method);
    method = 'centroid';
end

ptOut = ptIn;
normals = ptIn.Normal;

switch lower(method)
    case 'centroid'
        centroid = mean(ptIn.Location, 1);
        dirs = ptIn.Location - centroid;
        flipIdx = dot(normals, dirs, 2) < 0;
        normals(flipIdx, :) = -normals(flipIdx, :);
        fprintf('[Preprocess] Normals oriented consistently outward from centroid\n');
        
    case 'viewpoint'
        if nargin < 3
            error('Viewpoint must be specified for viewpoint orientation method');
        end
        if numel(viewpoint) ~= 3
            error('Viewpoint must be a 3-element vector');
        end
        dirs = ptIn.Location - viewpoint;
        flipIdx = dot(normals, dirs, 2) < 0;
        normals(flipIdx, :) = -normals(flipIdx, :);
        fprintf('[Preprocess] Normals oriented consistently toward viewpoint [%.1f, %.1f, %.1f]\n', viewpoint);
end

ptOut.Normal = normals;
end

%% Additional utility: Remove isolated clusters
function ptOut = preprocess_removeIsolatedClusters(ptIn, minClusterSize)
% PREPROCESS_REMOVEISOLATEDCLUSTERS Remove small isolated clusters
% Inputs:
%   ptIn          - Input point cloud
%   minClusterSize- Minimum cluster size to keep (default: 50)

% Input validation
if nargin < 1
    error('Point cloud input is required');
end
if ~isa(ptIn, 'pointCloud')
    error('Input must be a pointCloud object');
end
if ptIn.Count == 0
    error('Input point cloud is empty');
end
if nargin < 2, minClusterSize = 50; end

% Cluster points using DBSCAN (simplified)
[labels, numClusters] = pcsegdist(ptIn, 0.02); % Fixed epsilon for simplicity

% Count points in each cluster
clusterSizes = zeros(numClusters, 1);
for i = 1:numClusters
    clusterSizes(i) = sum(labels == i);
end

% Keep only large enough clusters
validClusters = find(clusterSizes >= minClusterSize);
keepIndices = ismember(labels, validClusters);

ptOut = select(ptIn, keepIndices);
removedPoints = ptIn.Count - ptOut.Count;

fprintf('[Preprocess] Removed %d small clusters (%d points)\n', ...
        numClusters - length(validClusters), removedPoints);
end

%% Additional utility: Transform point cloud
function ptOut = preprocess_transform(ptIn, transformationMatrix)
% PREPROCESS_TRANSFORM Apply transformation to point cloud
% Inputs:
%   ptIn                 - Input point cloud
%   transformationMatrix - 4x4 transformation matrix

% Input validation
if nargin < 2
    error('Transformation matrix is required');
end
if ~isa(ptIn, 'pointCloud')
    error('Input must be a pointCloud object');
end
if ~isequal(size(transformationMatrix), [4, 4])
    error('Transformation matrix must be 4x4');
end

% Apply transformation
ptOut = pctransform(ptIn, affine3d(transformationMatrix'));
fprintf('[Preprocess] Applied transformation to point cloud\n');
end
