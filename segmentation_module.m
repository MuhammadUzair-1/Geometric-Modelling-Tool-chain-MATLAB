%% segmentation_module.m - Enhanced Segmentation Module
% Robust segmentation of hub and blades with improved algorithms and validation

%% ------------------- Main hub & blade segmentation -------------------
function segments = segmentation_segmentHubAndBlades(pt, params)
% SEGMENTHUBANDBLADES Segment point cloud into hub and blade regions
% Input: 
%   pt - pointCloud object
%   params - optional parameters structure
% Output: 
%   segments - struct array with fields: type, indices, pointCloud

% Input validation
if nargin < 1
    error('Point cloud input is required');
end
if ~isa(pt, 'pointCloud')
    error('Input must be a pointCloud object');
end
if pt.Count < 50
    error('Point cloud has insufficient points (%d < 50)', pt.Count);
end

% Default parameters
if nargin < 2, params = struct(); end
if ~isfield(params, 'hubQuantileLow'), params.hubQuantileLow = 0.1; end
if ~isfield(params, 'hubQuantileHigh'), params.hubQuantileHigh = 0.9; end
if ~isfield(params, 'hubRadiusPercentile'), params.hubRadiusPercentile = 25; end
if ~isfield(params, 'minHubPoints'), params.minHubPoints = 100; end

% Align propeller roughly using PCA
xyz = pt.Location;
[coeff, ~, latent] = pca(xyz);
fprintf('[Segmentation] PCA variances: %.4f, %.4f, %.4f\n', latent(1), latent(2), latent(3));
xyz_aligned = xyz * coeff;  % aligned points

% Hub region estimation - use the axis with smallest variance as rotation axis
[~, minVarIdx] = min(latent);
z = xyz_aligned(:, minVarIdx); % Use the axis with smallest variance

% Find hub region along the rotation axis
zHub = z > quantile(z, params.hubQuantileLow) & z < quantile(z, params.hubQuantileHigh);

% Calculate radial distance in the plane perpendicular to rotation axis
otherDims = setdiff(1:3, minVarIdx);
r = sqrt(sum(xyz_aligned(:, otherDims).^2, 2));

% Determine hub radius threshold
rHubThreshold = prctile(r(zHub), params.hubRadiusPercentile);
hubIdx = find(r < rHubThreshold & zHub);

% Ensure we have enough hub points
if numel(hubIdx) < params.minHubPoints
    warning('Too few hub points (%d), adjusting threshold', numel(hubIdx));
    % Use a more generous threshold
    rHubThreshold = prctile(r(zHub), params.hubRadiusPercentile + 20);
    hubIdx = find(r < rHubThreshold & zHub);
end

% Blade points
allIdx = (1:size(xyz, 1))';
bladeIdx = setdiff(allIdx, hubIdx);

% Create pointCloud objects
hubCloud = select(pt, hubIdx);
bladeCloud = select(pt, bladeIdx);

% Output struct array
segments = struct('type', {}, 'indices', {}, 'pointCloud', {});
segments(1).type = 'hub';
segments(1).indices = hubIdx;
segments(1).pointCloud = hubCloud;

segments(2).type = 'blade';
segments(2).indices = bladeIdx;
segments(2).pointCloud = bladeCloud;

fprintf('[Segmentation] Hub: %d pts (radius: %.3f), Blade: %d pts\n', ...
        hubCloud.Count, rHubThreshold, bladeCloud.Count);
end

%% ------------------- Blade mask from normals / curvature -------------------
function [mask, curvatureValues] = segmentation_bladeMaskFromNormals(pt, varargin)
% BLADEMASKFROMNORMALS Return boolean mask of blade points using curvature analysis
% Inputs:
%   pt - pointCloud object
%   Optional: normals (Nx3 array) or parameter structure
%   curvatureThreshold - scalar threshold (default: 0.02)
% Outputs:
%   mask - boolean mask of blade points
%   curvatureValues - computed curvature values for each point

% Parse inputs
p = inputParser;
addRequired(p, 'pt', @(x) isa(x, 'pointCloud'));
addOptional(p, 'normals', [], @(x) isempty(x) || (isnumeric(x) && size(x, 2) == 3));
addOptional(p, 'curvatureThreshold', 0.02, @isscalar);
addParameter(p, 'kNeighbors', 10, @isscalar);
parse(p, pt, varargin{:});

normals = p.Results.normals;
curvatureThreshold = p.Results.curvatureThreshold;
kNeighbors = p.Results.kNeighbors;

numPoints = pt.Count;
if numPoints < kNeighbors + 1
    error('Not enough points for curvature estimation (%d < %d)', numPoints, kNeighbors + 1);
end

% Compute normals if not provided
if isempty(normals)
    fprintf('[Segmentation] Computing normals for curvature estimation...\n');
    normals = pcnormals(pt, min(20, numPoints-1));
end

mask = false(numPoints, 1);
curvatureValues = zeros(numPoints, 1);

% Use efficient nearest neighbor search
if license('test', 'statistics_toolbox') && exist('createns', 'file')
    kd = createns(pt.Location, 'NSMethod', 'kdtree');
    useKDTree = true;
else
    useKDTree = false;
    fprintf('[Segmentation] Using exhaustive search (no Statistics Toolbox)\n');
end

% Adjust k to not exceed available points
k = min(kNeighbors, numPoints - 1);

fprintf('[Segmentation] Computing curvature for %d points (k=%d)...\n', numPoints, k);

% Process in batches for large point clouds
batchSize = min(1000, numPoints);
numBatches = ceil(numPoints / batchSize);

for batch = 1:numBatches
    batchStart = (batch-1)*batchSize + 1;
    batchEnd = min(batch*batchSize, numPoints);
    batchIndices = batchStart:batchEnd;
    
    if useKDTree
        idx = knnsearch(kd, pt.Location(batchIndices, :), 'K', k+1); % +1 to exclude self
        idx = idx(:, 2:end); % Remove self point
    else
        % Exhaustive search fallback (slow for large point clouds)
        idx = zeros(length(batchIndices), k);
        for i = 1:length(batchIndices)
            pointIdx = batchIndices(i);
            dists = vecnorm(pt.Location - pt.Location(pointIdx, :), 2, 2);
            [~, sortedIdx] = sort(dists);
            idx(i, :) = sortedIdx(2:k+1); % Exclude self
        end
    end
    
    for i = 1:length(batchIndices)
        pointIdx = batchIndices(i);
        localNormals = normals(idx(i, :), :);
        
        % Calculate covariance matrix of normals
        covMat = cov(localNormals);
        eigenvalues = eig(covMat);
        
        % Curvature estimation using eigenvalues
        if all(eigenvalues >= 0)  % Ensure positive eigenvalues
            curvature = min(eigenvalues) / sum(eigenvalues);
            curvatureValues(pointIdx) = curvature;
            
            if curvature > curvatureThreshold
                mask(pointIdx) = true;
            end
        end
    end
    
    if mod(batch, 10) == 0 || batch == numBatches
        fprintf('[Segmentation] Processed %d/%d points\n', batchEnd, numPoints);
    end
end

fprintf('[Segmentation] Blade mask computed. %.1f%% points above curvature threshold %.3f\n', ...
        nnz(mask)/numPoints*100, curvatureThreshold);
end

%% ------------------- Cluster blades with improved DBSCAN -------------------
function bladeSegments = segmentation_clusterBlades(pt, params)
% CLUSTERBLADES Cluster blade points using optimized clustering
% Inputs:
%   pt - pointCloud object
%   params - optional parameters structure
% Outputs:
%   bladeSegments - struct array of blade clusters

% Input validation
if ~isa(pt, 'pointCloud')
    error('Input must be a pointCloud object');
end
if pt.Count < 10
    error('Point cloud has insufficient points (%d < 10)', pt.Count);
end

% Default parameters
if nargin < 2, params = struct(); end
if ~isfield(params, 'epsilon'), params.epsilon = 0.01; end
if ~isfield(params, 'minPoints'), params.minPoints = 50; end
if ~isfield(params, 'maxClusters'), params.maxClusters = 10; end

xyz = pt.Location;

% Check for DBSCAN availability
if license('test', 'statistics_toolbox') && exist('dbscan', 'file')
    fprintf('[Segmentation] Using DBSCAN clustering (epsilon=%.3f, minPoints=%d)\n', ...
            params.epsilon, params.minPoints);
    labels = dbscan(xyz, params.epsilon, params.minPoints);
else
    % Use Euclidean clustering as fallback
    fprintf('[Segmentation] Using Euclidean clustering (no DBSCAN available)\n');
    
    % For large point clouds, use subsampling to speed up clustering
    if size(xyz, 1) > 10000
        fprintf('[Segmentation] Subsampling for efficient clustering...\n');
        sampleIdx = randperm(size(xyz, 1), min(10000, size(xyz, 1)));
        xyzSample = xyz(sampleIdx, :);
        
        % Cluster subsampled points
        try
            T = clusterdata(xyzSample, 'Linkage', 'single', 'Cutoff', params.epsilon, ...
                           'MaxClust', params.maxClusters);
            
            % Assign labels to original points based on nearest neighbor
            kd = createns(xyzSample, 'NSMethod', 'kdtree');
            labels = knnsearch(kd, xyz);
            labels = T(labels);
        catch
            % Fallback to simple distance-based clustering
            labels = ones(size(xyz, 1), 1);
            warning('Clustering failed, treating all points as one cluster');
        end
    else
        try
            labels = clusterdata(xyz, 'Linkage', 'single', 'Cutoff', params.epsilon, ...
                               'MaxClust', params.maxClusters);
        catch
            labels = ones(size(xyz, 1), 1);
            warning('Clustering failed, treating all points as one cluster');
        end
    end
    
    % Convert to DBSCAN-like format (negative for noise)
    uniqueLabels = unique(labels);
    for i = 1:length(uniqueLabels)
        clusterSize = sum(labels == uniqueLabels(i));
        if clusterSize < params.minPoints
            labels(labels == uniqueLabels(i)) = -1;  % Mark as noise
        end
    end
end

% Process clusters
uniqueLabels = unique(labels);
% Remove noise label (-1) from cluster count
nClusters = numel(uniqueLabels) - any(uniqueLabels == -1);

if nClusters == 0
    warning('No valid clusters found. Adjust parameters or check input.');
    bladeSegments = struct('type', {}, 'indices', {}, 'pointCloud', {});
    return;
end

bladeSegments = struct('type', {}, 'indices', {}, 'pointCloud', {}, 'centroid', {}, 'size', {});
clusterCount = 0;

for i = 1:numel(uniqueLabels)
    if uniqueLabels(i) == -1
        continue;  % Skip noise
    end
    
    clusterCount = clusterCount + 1;
    idx = find(labels == uniqueLabels(i));
    clusterCloud = select(pt, idx);
    centroid = mean(clusterCloud.Location, 1);
    
    bladeSegments(clusterCount).type = sprintf('blade_%d', clusterCount);
    bladeSegments(clusterCount).indices = idx;
    bladeSegments(clusterCount).pointCloud = clusterCloud;
    bladeSegments(clusterCount).centroid = centroid;
    bladeSegments(clusterCount).size = numel(idx);
end

% Sort clusters by size (largest first)
[~, sortIdx] = sort([bladeSegments.size], 'descend');
bladeSegments = bladeSegments(sortIdx);

fprintf('[Segmentation] %d blade clusters detected\n', clusterCount);
for i = 1:clusterCount
    fprintf('  Blade %d: %d points, centroid [%.3f, %.3f, %.3f]\n', ...
            i, bladeSegments(i).size, bladeSegments(i).centroid);
end
end

%% ------------------- Enhanced Visualization -------------------
function segmentation_plotSegments(pt, segments, varargin)
% PLOTSEGMENTS Visualize segmented point cloud with various options
% Inputs:
%   pt - original point cloud (for context)
%   segments - struct array of segments
%   Optional: 'style' - 'combined' or 'separate'
%             'showOriginal' - true/false to show original points

% Parse optional parameters
p = inputParser;
addRequired(p, 'pt', @(x) isa(x, 'pointCloud'));
addRequired(p, 'segments', @isstruct);
addParameter(p, 'style', 'combined', @ischar);
addParameter(p, 'showOriginal', false, @islogical);
addParameter(p, 'titleStr', 'Segmented Propeller', @ischar);
parse(p, pt, segments, varargin{:});

style = p.Results.style;
showOriginal = p.Results.showOriginal;
titleStr = p.Results.titleStr;

if strcmpi(style, 'separate')
    % Create separate subplots for each segment
    numSegments = numel(segments);
    fig = figure;
    rows = ceil(sqrt(numSegments + showOriginal));
    cols = ceil((numSegments + showOriginal) / rows);
    
    if showOriginal
        subplot(rows, cols, 1);
        pcshow(pt);
        title('Original Point Cloud');
    end
    
    for i = 1:numSegments
        subplot(rows, cols, i + showOriginal);
        if segments(i).pointCloud.Count > 0
            pcshow(segments(i).pointCloud);
            title(segments(i).type);
        else
            text(0.5, 0.5, 'No points', 'HorizontalAlignment', 'center');
            axis off;
        end
    end
    
else
    % Combined plot
    figure; hold on; grid on; axis equal;
    
    if showOriginal
        % Plot original points in light gray
        scatter3(pt.Location(:,1), pt.Location(:,2), pt.Location(:,3), ...
                5, [0.8, 0.8, 0.8], 'filled', 'MarkerEdgeAlpha', 0.3, 'MarkerFaceAlpha', 0.3);
    end
    
    % Plot segments with distinct colors
    colors = lines(numel(segments));
    legendEntries = cell(numel(segments), 1);
    
    for i = 1:numel(segments)
        pc = segments(i).pointCloud;
        if pc.Count > 0
            scatter3(pc.Location(:,1), pc.Location(:,2), pc.Location(:,3), ...
                    10, repmat(colors(i,:), pc.Count, 1), 'filled');
            legendEntries{i} = sprintf('%s (%d points)', segments(i).type, pc.Count);
        else
            legendEntries{i} = sprintf('%s (0 points)', segments(i).type);
        end
    end
    
    title(titleStr);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    legend(legendEntries, 'Location', 'best');
    hold off;
end
end

%% ------------------- Utility: Enhanced point cloud validation -------------------
function [isEmpty, message] = segmentation_isPointCloudEmpty(pc, minPoints)
% ISPOINTCLOUDEMPTY Check if point cloud is empty or has insufficient points
% Inputs:
%   pc - pointCloud object or point array
%   minPoints - minimum required points (default: 10)
% Outputs:
%   isEmpty - boolean indicating if point cloud is empty
%   message - descriptive message

if nargin < 2, minPoints = 10; end

if isempty(pc)
    isEmpty = true;
    message = 'Point cloud is empty';
    return;
end

if isa(pc, 'pointCloud')
    count = pc.Count;
elseif isnumeric(pc) && size(pc, 2) == 3
    count = size(pc, 1);
else
    isEmpty = true;
    message = 'Invalid point cloud format';
    return;
end

isEmpty = count < minPoints;
if isEmpty
    message = sprintf('Insufficient points (%d < %d)', count, minPoints);
else
    message = sprintf('Valid point cloud with %d points', count);
end
end

%% ------------------- Utility: Extract specific segment type -------------------
function segment = segmentation_extractSegment(segments, segmentType)
% EXTRACTSEGMENT Extract specific segment type from segments struct
% Inputs:
%   segments - segments struct array
%   segmentType - type to extract ('hub', 'blade', or specific blade name)
% Output:
%   segment - matching segment or empty if not found

segment = [];
for i = 1:numel(segments)
    if strcmpi(segments(i).type, segmentType) || ...
       (strcmpi(segmentType, 'blade') && startsWith(segments(i).type, 'blade'))
        segment = segments(i);
        break;
    end
end

if isempty(segment)
    warning('Segment type "%s" not found', segmentType);
end
end
