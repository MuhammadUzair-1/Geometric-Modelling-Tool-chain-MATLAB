%% registration_module.m - Enhanced Registration Functions for Propeller Toolchain
% Robust alignment functions for propeller and blade point clouds

%% Align propeller to global Z-axis using PCA with proper axis selection
function [T, ptOut] = registration_alignToAxis(pt, axisToAlign)
% REGISTRATION_ALIGNTOAXIS Align point cloud to specified axis using PCA
% Inputs:
%   pt         - Input point cloud
%   axisToAlign- Axis to align to: 'z' (default), 'x', or 'y'
% Outputs:
%   T          - Transformation structure with R (rotation) and t (translation)
%   ptOut      - Transformed point cloud

% Input validation
if nargin < 1
    error('Point cloud input is required');
end
if ~isa(pt, 'pointCloud')
    error('Input must be a pointCloud object');
end
if pt.Count == 0
    error('Input point cloud is empty');
end
if nargin < 2, axisToAlign = 'z'; end

% Validate axis selection
validAxes = {'x', 'y', 'z'};
if ~any(strcmpi(axisToAlign, validAxes))
    warning('Invalid axis: %s. Using ''z''.', axisToAlign);
    axisToAlign = 'z';
end

xyz = pt.Location;

% PCA to find principal axes
[coeff, ~, latent] = pca(xyz);
fprintf('[Registration] PCA variances: %.4f, %.4f, %.4f\n', latent(1), latent(2), latent(3));

% Determine which axis corresponds to propeller rotation axis
% The rotation axis should have the SMALLEST variance (points are symmetric around it)
[~, minVarIdx] = min(latent);
rotationAxis = coeff(:, minVarIdx);

% Determine target axis based on input
switch lower(axisToAlign)
    case 'x'
        targetAxis = [1; 0; 0];
    case 'y'
        targetAxis = [0; 1; 0];
    case 'z'
        targetAxis = [0; 0; 1];
end

% Compute rotation to align rotationAxis to target axis
v = cross(rotationAxis, targetAxis);
s = norm(v);
c = dot(rotationAxis, targetAxis);

if s < 1e-10
    % Vectors are parallel or anti-parallel
    if c < 0
        % Anti-parallel, need 180° rotation around perpendicular axis
        R = diag([-1, -1, 1]); % Simple 180° rotation around Z
    else
        % Parallel, no rotation needed
        R = eye(3);
    end
else
    % Rodrigues' rotation formula
    vx = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
    R = eye(3) + vx + vx^2 * (1 - c) / (s^2);
end

% Rotate and center at origin
centroid = mean(xyz, 1);
xyzAligned = (R * (xyz - centroid)')' + centroid;

ptOut = pointCloud(xyzAligned);
if isfield(pt, 'Normal') && ~isempty(pt.Normal)
    ptOut.Normal = (R * pt.Normal')'; % Rotate normals too
end

% Store transformation
T.R = R;
T.t = -centroid * R'; % Translation component
T.centroid = centroid;
T.axisAligned = axisToAlign;

fprintf('[Registration] PCA alignment to %s-axis completed. Centroid: [%.3f, %.3f, %.3f]\n', ...
        upper(axisToAlign), centroid(1), centroid(2), centroid(3));
end

%% Refine alignment using ICP with comprehensive options
function [ptOut, T, rmse] = registration_refineICP(pt, template, params)
% REGISTRATION_REFINEICP Refine point cloud alignment to template using ICP
% Inputs:
%   pt       - Moving point cloud
%   template - Fixed point cloud (template)
%   params   - Structure with ICP parameters
% Outputs:
%   ptOut    - Transformed point cloud
%   T        - Transformation structure
%   rmse     - Final RMSE value

% Input validation
if nargin < 2
    error('Both point cloud and template are required');
end
if ~isa(pt, 'pointCloud') || ~isa(template, 'pointCloud')
    error('Inputs must be pointCloud objects');
end
if pt.Count == 0 || template.Count == 0
    error('Point clouds cannot be empty');
end

% Default parameters
if nargin < 3, params = struct(); end
if ~isfield(params, 'maxIter'), params.maxIter = 50; end
if ~isfield(params, 'tolerance'), params.tolerance = 1e-6; end
if ~isfield(params, 'inlierRatio'), params.inlierRatio = 1.0; end
if ~isfield(params, 'metric'), params.metric = 'pointToPoint'; end
if ~isfield(params, 'extrapolate'), params.extrapolate = false; end
if ~isfield(params, 'verbose'), params.verbose = true; end

% Validate parameters
if params.maxIter < 1
    warning('MaxIter must be at least 1, setting to 50');
    params.maxIter = 50;
end
if params.tolerance <= 0
    warning('Tolerance must be positive, setting to 1e-6');
    params.tolerance = 1e-6;
end
if params.inlierRatio <= 0 || params.inlierRatio > 1
    warning('InlierRatio must be between 0 and 1, setting to 1.0');
    params.inlierRatio = 1.0;
end

% Perform ICP with specified parameters
tform = pcregistericp(pt, template, ...
    'MaxIterations', params.maxIter, ...
    'Tolerance', params.tolerance, ...
    'InlierRatio', params.inlierRatio, ...
    'Metric', params.metric, ...
    'Extrapolate', params.extrapolate, ...
    'Verbose', params.verbose);

ptOut = pctransform(pt, tform);

% Calculate final RMSE (more accurate than the iterative value)
movingTransformed = ptOut.Location;
fixed = template.Location;

if strcmpi(params.metric, 'pointToPoint')
    [indices, dists] = knnsearch(fixed, movingTransformed);
    rmse = sqrt(mean(dists.^2));
else
    % For pointToPlane, we'd need normals
    rmse = NaN;
    warning('RMSE calculation not implemented for pointToPlane metric');
end

% Store transformation info
T.R = tform.T(1:3, 1:3);
T.t = tform.T(4, 1:3);
T.tform = tform; % Store the complete transformation object

if params.verbose
    fprintf('[Registration] ICP refinement completed. RMSE=%.6f, Iterations=%d\n', ...
            rmse, tform.NumIterations);
end
end

%% Align multiple templates with proper transformation accumulation
function [ptAligned, transformations, rmses] = registration_alignMultipleTemplates(pt, templates, params)
% REGISTRATION_ALIGNMULTIPLETEMPLATES Align to multiple templates sequentially
% Inputs:
%   pt        - Input point cloud
%   templates - Cell array of template point clouds
%   params    - ICP parameters (optional)
% Outputs:
%   ptAligned - Final transformed point cloud
%   transformations - Cell array of transformation structures
%   rmses     - Array of RMSE values for each alignment

% Input validation
if nargin < 2
    error('Point cloud and templates are required');
end
if ~isa(pt, 'pointCloud')
    error('Input must be a pointCloud object');
end
if ~iscell(templates)
    error('Templates must be a cell array');
end
if isempty(templates)
    error('Templates cell array cannot be empty');
end

nTemplates = numel(templates);
transformations = cell(nTemplates, 1);
rmses = zeros(nTemplates, 1);
ptAligned = pt;

% Default parameters
if nargin < 3, params = struct(); end

for i = 1:nTemplates
    if ~isa(templates{i}, 'pointCloud')
        warning('Template %d is not a pointCloud object, skipping', i);
        continue;
    end
    
    fprintf('[Registration] Aligning to template %d/%d...\n', i, nTemplates);
    
    [ptAligned, T, rmse] = registration_refineICP(ptAligned, templates{i}, params);
    transformations{i} = T;
    rmses(i) = rmse;
    
    fprintf('[Registration] Template %d alignment completed. RMSE=%.6f\n', i, rmse);
end

fprintf('[Registration] Multiple template alignment completed. Average RMSE=%.6f\n', mean(rmses));
end

%% Additional utility: Coarse alignment using feature matching
function [ptOut, T, features] = registration_coarseAlign(pt, template, params)
% REGISTRATION_COARSEALIGN Coarse alignment using feature detection and matching
% Inputs:
%   pt       - Moving point cloud
%   template - Fixed point cloud
%   params   - Parameters for feature extraction and matching
% Outputs:
%   ptOut    - Coarsely aligned point cloud
%   T        - Estimated transformation
%   features - Detected features and matches

% Input validation
if nargin < 2
    error('Both point cloud and template are required');
end
if ~isa(pt, 'pointCloud') || ~isa(template, 'pointCloud')
    error('Inputs must be pointCloud objects');
end

% Default parameters
if nargin < 3, params = struct(); end
if ~isfield(params, 'minQuality'), params.minQuality = 0.01; end
if ~isfield(params, 'minContrast'), params.minContrast = 0.01; end
if ~isfield(params, 'maxNumKeypoints'), params.maxNumKeypoints = 1000; end

% Extract features (simplified example - would need actual feature detection)
% In practice, you might use:
% features1 = extractFeatures(pt, detectISSFeatures(pt));
% features2 = extractFeatures(template, detectISSFeatures(template));
% indexPairs = matchFeatures(features1, features2);

% For this example, we'll fall back to ICP but with different parameters
fprintf('[Registration] Performing coarse alignment...\n');

% Use ICP with relaxed parameters for coarse alignment
coarseParams = struct();
coarseParams.maxIter = 20;
coarseParams.tolerance = 1e-4;
coarseParams.inlierRatio = 0.8;
coarseParams.verbose = false;

[ptOut, T, rmse] = registration_refineICP(pt, template, coarseParams);
features = struct(); % Placeholder

fprintf('[Registration] Coarse alignment completed. RMSE=%.6f\n', rmse);
end

%% Additional utility: Evaluate registration quality
function [metrics, overlap] = registration_evaluateAlignment(pt, template, params)
% REGISTRATION_EVALUATEALIGNMENT Evaluate quality of registration
% Inputs:
%   pt       - Transformed point cloud
%   template - Reference point cloud
%   params   - Evaluation parameters
% Outputs:
%   metrics  - Structure with various quality metrics
%   overlap  - Estimated overlap percentage

% Input validation
if nargin < 2
    error('Both point cloud and template are required');
end
if ~isa(pt, 'pointCloud') || ~isa(template, 'pointCloud')
    error('Inputs must be pointCloud objects');
end

% Default parameters
if nargin < 3, params = struct(); end
if ~isfield(params, 'maxDistance'), params.maxDistance = 0.01; end

% Calculate point-to-point distances
[indices, dists] = knnsearch(template.Location, pt.Location);
validMatches = dists < params.maxDistance;

% Calculate metrics
metrics.rmse = sqrt(mean(dists(validMatches).^2));
metrics.mae = mean(dists(validMatches));
metrics.maxError = max(dists(validMatches));
metrics.medianError = median(dists(validMatches));
metrics.inlierRatio = nnz(validMatches) / pt.Count;

% Estimate overlap
overlap = metrics.inlierRatio * 100;

fprintf('[Registration] Alignment evaluation:\n');
fprintf('  RMSE: %.6f\n', metrics.rmse);
fprintf('  MAE: %.6f\n', metrics.mae);
fprintf('  Max Error: %.6f\n', metrics.maxError);
fprintf('  Median Error: %.6f\n', metrics.medianError);
fprintf('  Inlier Ratio: %.2f%%\n', metrics.inlierRatio * 100);
fprintf('  Estimated Overlap: %.1f%%\n', overlap);
end
