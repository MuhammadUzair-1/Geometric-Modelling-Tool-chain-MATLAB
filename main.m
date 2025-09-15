%% main.m - Propeller Analysis Pipeline Coordinator
function [report, ptAligned, segments, centerline, sections, splines] = main(varargin)
% MAIN - Coordinates propeller analysis pipeline using existing modules
%
% Usage:
%   main() % Uses default parameters
%   main('inputFile', 'data/scan.ply', 'nSections', 25) % Custom parameters
%   [report, ptAligned, segments] = main(...); % Return results
%
% Outputs:
%   report - Analysis results structure
%   ptAligned - Registered point cloud
%   segments - Segmented hub and blades
%   centerline - Blade centerline points
%   sections - Cross-section data
%   splines - Fitted spline data

%% Initialize
clc; clear; close all;
utils_logger('INFO', 'Starting propeller analysis pipeline');

% Start main timer
tAll = utils_startTimer('Propeller toolchain started');

try
    %% Parse input parameters
    params = parseInputParameters(varargin{:});
    
    %% Check dependencies
    utils_logger('INFO', 'Checking dependencies...');
    deps = utils_checkDependencies();
    if ~deps.computerVision || ~deps.curveFitting || ~deps.statistics
        error('Missing required toolboxes. Please install Computer Vision, Curve Fitting, and Statistics toolboxes.');
    end

    %% Validate input file
    if ~exist(params.inputFile, 'file')
        error('Input file not found: %s', params.inputFile);
    end
    
    %% Create output directory if needed
    if ~exist('output', 'dir')
        mkdir('output');
        utils_logger('INFO', 'Created output directory');
    end

    %% Step 1: Read point cloud
    tLoad = utils_startTimer('Loading point cloud');
    pt = pcread(params.inputFile);
    utils_stopTimer(tLoad, sprintf('Loaded %d points', pt.Count));

    %% Step 2: Preprocess
    tPreprocess = utils_startTimer('Preprocessing');
    pt = pcdenoise(pt, 'NumNeighbors', 20, 'Threshold', 1.0);
    pt = pcdownsample(pt, 'gridAverage', params.voxelSize);
    utils_stopTimer(tPreprocess, sprintf('Downsampled to %d points', pt.Count));

    %% Step 3: Segmentation
    tSeg = utils_startTimer('Segmentation');
    segments = segmentation_segmentHubAndBlades(pt);
    utils_stopTimer(tSeg, sprintf('Found %d segments', numel(segments)));
    
    % Validate segmentation
    if numel(segments) < 2
        error('Segmentation failed: Expected at least hub and one blade, got %d segments', numel(segments));
    end
    
    % Identify blade (assume largest segment after hub is blade)
    segmentSizes = arrayfun(@(s) s.pointCloud.Count, segments);
    [~, hubIdx] = max(segmentSizes);
    bladeIndices = setdiff(1:numel(segments), hubIdx);
    
    if isempty(bladeIndices)
        error('No blades found in segmentation');
    end
    
    % Use first blade for analysis
    blade = segments(bladeIndices(1)).pointCloud;
    utils_logger('INFO', 'Using blade with %d points for analysis', blade.Count);

    %% Step 4: Registration
    tReg = utils_startTimer('Registration');
    [~, ptAligned] = registration_alignToAxis(pt);
    [ptAligned, T] = registration_refineICP(ptAligned, blade);
    utils_stopTimer(tReg, 'Registration completed');

    %% Step 5: Skeleton & Cross-sections
    tSkel = utils_startTimer('Skeleton extraction');
    centerline = skeleton_extractBladeCenterline(blade);
    sections = skeleton_sampleCrossSections(blade, centerline, params.nSections);
    utils_stopTimer(tSkel, sprintf('Extracted %d sections', numel(sections)));

    %% Step 6: Fitting
    tFit = utils_startTimer('Fitting');
    splines = fitting_fitSectionSplines(sections, 3, 1e-3);
    nurbs = fitting_fitSurfaceNURBS(splines, centerline);
    errors = fitting_computeSectionErrors(sections, splines);
    utils_stopTimer(tFit, 'Fitting completed');

    %% Step 7: Analysis
    tAnalysis = utils_startTimer('Analysis');
    report = analysis_computeChordPitchTwist(splines, centerline);
    hubDiameter = analysis_measureHubDiameter(segments(1).pointCloud);
    
    % Add additional metrics to report
    report.timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    report.inputFile = params.inputFile;
    report.totalPoints = pt.Count;
    report.processedPoints = blade.Count;
    report.hubDiameter = hubDiameter;
    report.fittingErrors = errors;
    utils_stopTimer(tAnalysis, 'Analysis completed');

    %% Step 8: Export
    tExport = utils_startTimer('Export');
    export_writeJSONMetadata(report, 'output/report.json');
    
    % Only export CAD formats if we have valid geometry
    if ~isempty(nurbs)
        export_writeIGES(nurbs, 'output/propeller.iges');
    end
    utils_stopTimer(tExport, 'Export completed');

    %% Step 9: Visualization
    tViz = utils_startTimer('Visualization');
    visualization_plotPointCloud(ptAligned);
    visualization_plotSegments(ptAligned, segments);
    visualization_plotSectionsAndSplines(sections, splines, centerline);
    utils_stopTimer(tViz, 'Visualization completed');

    %% Final report
    utils_logger('INFO', 'Pipeline completed successfully!');
    utils_logger('INFO', 'Blade characteristics:');
    utils_logger('INFO', '  - Chord: %.3f m (avg)', mean([report.chord]));
    utils_logger('INFO', '  - Pitch: %.3f m (avg)', mean([report.pitch]));
    utils_logger('INFO', '  - Twist: %.1f deg (range)', range([report.twist]));
    utils_logger('INFO', '  - Hub diameter: %.3f m', hubDiameter);
    
    utils_stopTimer(tAll, 'Propeller toolchain finished');

catch ME
    %% Error handling
    utils_logger('ERROR', 'Pipeline failed: %s', ME.message);
    utils_stopTimer(tAll, 'Propeller toolchain failed');
    
    % Re-throw error for debugging
    rethrow(ME);
end
end

%% Helper function for parameter parsing
function params = parseInputParameters(varargin)
    % Default parameters
    params.inputFile = 'data/scan_example.ply';
    params.voxelSize = 0.002;
    params.nSections = 30;
    
    % Parse name-value pairs
    if mod(nargin, 2) == 0
        for i = 1:2:nargin
            if isfield(params, varargin{i})
                params.(varargin{i}) = varargin{i+1};
            else
                warning('Unknown parameter: %s', varargin{i});
            end
        end
    elseif nargin == 1 && isstruct(varargin{1})
        % Allow passing a config structure
        config = varargin{1};
        fields = fieldnames(config);
        for i = 1:numel(fields)
            if isfield(params, fields{i})
                params.(fields{i}) = config.(fields{i});
            end
        end
    elseif nargin > 0
        error('Parameters must be name-value pairs or a structure');
    end
    
    utils_logger('INFO', 'Using parameters:');
    utils_logger('INFO', '  Input file: %s', params.inputFile);
    utils_logger('INFO', '  Voxel size: %.4f m', params.voxelSize);
    utils_logger('INFO', '  Sections: %d', params.nSections);
end
