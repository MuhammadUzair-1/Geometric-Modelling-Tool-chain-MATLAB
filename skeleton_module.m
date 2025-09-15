%% skeleton_module.m - Improved blade skeleton extraction
function [centerline, sections, curvature] = skeleton_extractBlade(bladePt, nSections, crossSectionRadius)
% Extract blade centerline, cross-sections, and curvature

%% Parameter validation
if nargin < 2
    nSections = 20;
end
if nargin < 3
    crossSectionRadius = 0.05; % 5cm default
end

xyz = bladePt.Location;
nPoints = size(xyz,1);

%% Extract blade centerline with robust PCA alignment
centerline = extractCenterline(xyz);

%% Sample properly oriented cross-sections
sections = sampleCrossSections(xyz, centerline, nSections, crossSectionRadius);

%% Compute smoothed curvature
curvature = computeSmoothedCurvature(centerline);

fprintf('[Skeleton] Extraction complete: %d centerline points, %d sections\n', ...
    size(centerline,1), nSections);
end

%% Improved centerline extraction
function centerline = extractCenterline(xyz)
% Use PCA to find the actual longest dimension
[coeff, score, latent] = pca(xyz);

% Find which principal component has the largest variance
[~, mainAxisIdx] = max(latent);
mainAxis = coeff(:,mainAxisIdx);

% Project points along the actual longest axis
projLength = xyz * mainAxis;

% Use adaptive binning based on point density
nBins = min(50, round(size(xyz,1)/100)); % Adaptive bin count
[counts, edges, binIndices] = histcounts(projLength, nBins);

centerline = zeros(nBins, 3);
validBins = false(nBins, 1);

for i = 1:nBins
    if counts(i) > 0
        binPoints = xyz(binIndices == i, :);
        centerline(i, :) = median(binPoints, 1); % More robust than mean
        validBins(i) = true;
    else
        centerline(i, :) = NaN;
    end
end

% Remove NaN entries and smooth
centerline = centerline(validBins, :);
centerline = smoothCenterline(centerline);
end

%% Improved cross-section sampling with proper orientation
function sections = sampleCrossSections(xyz, centerline, nSections, radius)
nSections = min(nSections, size(centerline,1));
sections = struct('station', [], 'origin', [], 'normal', [], 'points', []);

% Compute tangent vectors along centerline for orientation
tangents = computeTangents(centerline);

for i = 1:nSections
    % Use linear interpolation for smoother sampling
    t = (i-1)/(nSections-1);
    idx = round(1 + t*(size(centerline,1)-1));
    idx = max(1, min(idx, size(centerline,1)));
    
    origin = centerline(idx, :);
    normal = tangents(idx, :); % Use actual tangent as normal
    
    % Extract points within the cross-section cylinder
    distances = sqrt(sum((xyz - origin).^2, 2));
    mask = distances < radius;
    
    sections(i).station = t;
    sections(i).origin = origin;
    sections(i).normal = normal;
    sections(i).points = xyz(mask, :);
end
end

%% Helper functions
function tangents = computeTangents(centerline)
n = size(centerline,1);
tangents = zeros(n,3);

% Central difference for interior points
for i = 2:n-1
    tangents(i,:) = centerline(i+1,:) - centerline(i-1,:);
    tangents(i,:) = tangents(i,:) / norm(tangents(i,:));
end

% Endpoint handling
tangents(1,:) = centerline(2,:) - centerline(1,:);
tangents(1,:) = tangents(1,:) / norm(tangents(1,:));
tangents(n,:) = centerline(n,:) - centerline(n-1,:);
tangents(n,:) = tangents(n,:) / norm(tangents(n,:));
end

function smoothed = smoothCenterline(centerline)
% Simple moving average smoothing
windowSize = 3;
smoothed = movmean(centerline, windowSize, 1);
end

function curvature = computeSmoothedCurvature(centerline)
n = size(centerline,1);
curvature = zeros(n,1);

for i = 2:n-1
    v1 = centerline(i,:) - centerline(i-1,:);
    v2 = centerline(i+1,:) - centerline(i,:);
    
    if norm(v1) > eps && norm(v2) > eps
        v1 = v1 / norm(v1);
        v2 = v2 / norm(v2);
        curvature(i) = acos(dot(v1, v2)) / (0.5*(norm(v1) + norm(v2)));
    end
end

% Smooth curvature and handle endpoints
curvature = movmean(curvature, 3);
curvature(1) = curvature(2);
curvature(n) = curvature(n-1);
end
