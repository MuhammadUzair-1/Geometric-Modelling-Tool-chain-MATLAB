%% Compute chord, pitch, twist along blade - FIXED VERSION
function report = analysis_computeChordPitchTwist(splines, centerline)
nSections = numel(splines);

% Validate inputs
if nSections == 0
    error('No splines provided');
end
if size(centerline, 1) ~= nSections || size(centerline, 2) ~= 3
    error('Centerline must be Nx3 matrix matching number of splines');
end

chord = zeros(nSections,1);
pitch = zeros(nSections,1);
twist = zeros(nSections,1);

% Evaluate splines at multiple points for accurate measurements
nEvalPoints = 50;  % Number of points to evaluate per spline
evalParams = linspace(0, 1, nEvalPoints);

for i = 1:nSections
    if isempty(splines(i).coefs) || isempty(splines(i).breaks)
        chord(i) = NaN; pitch(i) = NaN; twist(i) = NaN;
        continue;
    end
    
    % Evaluate spline points
    try
        evalPoints = ppval(splines(i), evalParams)';
    catch
        chord(i) = NaN; pitch(i) = NaN; twist(i) = NaN;
        continue;
    end
    
    if size(evalPoints, 1) < 2
        chord(i) = NaN; pitch(i) = NaN; twist(i) = NaN;
        continue;
    end
    
    % Chord: max distance along local X axis (first principal component)
    chord(i) = max(evalPoints(:,1)) - min(evalPoints(:,1));
    
    % Pitch: axial distance between sections (Z-direction)
    if i > 1
        try
            prevPoints = ppval(splines(i-1), evalParams)';
            if ~isempty(prevPoints)
                pitch(i) = abs(centerline(i,3) - centerline(i-1,3));
            else
                pitch(i) = NaN;
            end
        catch
            pitch(i) = NaN;
        end
    else
        pitch(i) = NaN;
    end
    
    % Twist: angle of chord line relative to reference
    chordVector = evalPoints(end,:) - evalPoints(1,:);
    twist(i) = atan2d(chordVector(2), chordVector(1));  % Angle in degrees
end

report.chord = chord;
report.pitch = pitch;
report.twist = twist;
report.nSections = nSections;

fprintf('[Analysis] Chord, pitch, twist computed for %d sections\n', nSections);
end

%% Measure hub diameter - FIXED VERSION
function d = analysis_measureHubDiameter(meshOrPt)
if isa(meshOrPt,'pointCloud')
    xyz = meshOrPt.Location;
elseif isstruct(meshOrPt) && isfield(meshOrPt,'vertices')
    xyz = meshOrPt.vertices;
else
    error('Unsupported input type for hub diameter');
end

if isempty(xyz)
    d = NaN;
    fprintf('[Analysis] Warning: Empty input for hub diameter\n');
    return;
end

% Use adaptive threshold based on data spread
radii = sqrt(sum(xyz(:,1:2).^2, 2));
maxRadius = max(radii);
threshold = maxRadius * 0.3;  % Adaptive threshold

hubRadii = radii(radii < threshold);
if isempty(hubRadii)
    d = 2 * mean(radii);  % Fallback to overall mean
    fprintf('[Analysis] Warning: Using fallback hub diameter measurement\n');
else
    d = 2 * mean(hubRadii);
end

fprintf('[Analysis] Hub diameter measured: %.4f\n', d);
end
