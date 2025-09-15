%% Fit B-spline to section points - FIXED
function splines = fitting_fitSectionSplines(sections, order, tol)
if nargin<2, order = 3; end
if nargin<3, tol = 1e-3; end
nSections = numel(sections);
splines = struct('form', {}, 'coefs', {}, 'knots', {}, 'order', {}, 'bbox', {});

for i = 1:nSections
    pts = sections(i).points;
    if size(pts, 1) < order + 1
        warning('Not enough points for B-spline in section %d (%d < %d)', i, size(pts, 1), order + 1);
        splines(i).form = 'invalid';
        continue;
    end
    
    % Project points to 2D plane (XY)
    pts2D = pts(:, 1:2);

    % Parameterize using chord length (FIXED)
    distances = sqrt(sum(diff(pts2D).^2, 2));
    t = [0; cumsum(distances)];
    if max(t) < eps
        t = linspace(0, 1, size(pts2D, 1))';
    else
        t = t / max(t);
    end

    % Fit B-spline (form 'B-')
    try
        spl = spap2(floor((order+1)/2), order, t', pts2D');  % knots = order + 1
        
        splines(i).form = 'B-';
        splines(i).coefs = spl.coefs;      % Control points (2 × nCtrl)
        splines(i).knots = spl.knots;      % Knot vector
        splines(i).order = order;
        splines(i).bbox = [min(pts2D); max(pts2D)];
        splines(i).splineObject = spl;     % Store the actual spline object
        
    catch ME
        warning('B-spline fitting failed for section %d: %s', i, ME.message);
        splines(i).form = 'failed';
    end
end

fprintf('[Fitting] B-spline fitting done for %d sections\n', nSections);
end

%% Loft NURBS surface along centerline - FIXED
function nurbs = fitting_fitSurfaceNURBS(splines, centerline)
nSections = numel(splines);
degreeU = 3;
degreeV = 3;

% Find valid splines
validSplines = find(arrayfun(@(s) isfield(s, 'form') && strcmp(s.form, 'B-'), splines));
if isempty(validSplines)
    error('No valid splines for NURBS lofting');
end

% Determine control points arrangement
nCtrlPoints = arrayfun(@(i) size(splines(i).coefs, 2), validSplines);
maxCtrl = max(nCtrlPoints);
minCtrl = min(nCtrlPoints);

if maxCtrl ~= minCtrl
    warning('Varying control point counts (%d-%d). Using max count.', minCtrl, maxCtrl);
end

% Initialize control points and weights
ctrlPts = zeros(3, maxCtrl, nSections);
weights = ones(maxCtrl, nSections);

for i = 1:nSections
    if ~strcmp(splines(i).form, 'B-')
        continue;  % Skip invalid splines
    end
    
    % Get control points from B-spline (2D XY)
    xyCtrl = splines(i).coefs;  % 2 × nCtrl
    
    % Add Z coordinate from centerline and create 3D control points
    nCtrl = size(xyCtrl, 2);
    ctrlPts(1:2, 1:nCtrl, i) = xyCtrl;
    ctrlPts(3, 1:nCtrl, i) = centerline(i, 3);
    
    % Pad if necessary
    if nCtrl < maxCtrl
        ctrlPts(1:2, nCtrl+1:maxCtrl, i) = repmat(xyCtrl(:, end), 1, maxCtrl - nCtrl);
        ctrlPts(3, nCtrl+1:maxCtrl, i) = centerline(i, 3);
        weights(nCtrl+1:maxCtrl, i) = 1;
    end
end

% Create NURBS structure (simplified - real implementation needs proper knot vectors)
nurbs = struct();
nurbs.form = 'B-';
nurbs.dim = 3;
nurbs.number = [maxCtrl, nSections];
nurbs.coefs = zeros(4, maxCtrl, nSections);
nurbs.coefs(1:3, :, :) = ctrlPts;
nurbs.coefs(4, :, :) = weights;
nurbs.order = [degreeU, degreeV];
nurbs.knots = {linspace(0, 1, maxCtrl + degreeU), linspace(0, 1, nSections + degreeV)};

fprintf('[Fitting] Lofted NURBS surface created with %d control points\n', maxCtrl);
end

%% Compute fitting error for sections - FIXED
function errors = fitting_computeSectionErrors(sections, splines)
nSections = numel(sections);
errors = nan(nSections, 1);  % Use NaN for invalid sections

for i = 1:nSections
    pts = sections(i).points;
    if ~strcmp(splines(i).form, 'B-') || isempty(pts)
        errors(i) = NaN;
        continue;
    end
    
    % Create evaluation parameters
    nEval = min(100, size(pts, 1));
    t_eval = linspace(0, 1, nEval);
    
    try
        % Evaluate B-spline
        splineEval = fnval(splines(i).splineObject, t_eval);
        
        % Find closest points for error calculation
        actualPts = pts(:, 1:2)';
        distances = zeros(1, nEval);
        
        for j = 1:nEval
            dists = vecnorm(actualPts - splineEval(:, j), 2, 1);
            distances(j) = min(dists);
        end
        
        errors(i) = sqrt(mean(distances.^2));
        
    catch ME
        warning('Error computation failed for section %d: %s', i, ME.message);
        errors(i) = NaN;
    end
end

validErrors = errors(~isnan(errors));
if ~isempty(validErrors)
    fprintf('[Fitting] RMS errors: mean=%.4f, max=%.4f\n', mean(validErrors), max(validErrors));
else
    fprintf('[Fitting] No valid error computations\n');
end
end
