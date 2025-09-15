%% visualization_module.m - Enhanced visualization functions for Propeller Toolchain

%% Configuration
persistent figCount defaultColors

if isempty(figCount)
    figCount = 1;
    defaultColors = lines(7); % Predefine colors
end

%% Plot point cloud with options
function [hFig, hPc] = visualization_plotPointCloud(pt, varargin)
    % Parse optional parameters
    p = inputParser;
    addOptional(p, 'Parent', []);
    addParameter(p, 'Title', 'Propeller Point Cloud');
    addParameter(p, 'Color', [0.5 0.5 0.5]);
    addParameter(p, 'MarkerSize', 5);
    addParameter(p, 'View', [30, 45]);
    addParameter(p, 'Axis', true);
    addParameter(p, 'Grid', true);
    parse(p, varargin{:});
    
    % Create or use existing figure
    if isempty(p.Results.Parent)
        hFig = createFigure(p.Results.Title);
    else
        hFig = p.Results.Parent;
        figure(hFig);
        hold on;
    end
    
    % Plot point cloud
    if isa(pt, 'pointCloud')
        hPc = pcshow(pt, p.Results.Color, 'MarkerSize', p.Results.MarkerSize);
    else
        hPc = scatter3(pt(:,1), pt(:,2), pt(:,3), p.Results.MarkerSize, ...
                      p.Results.Color, 'filled');
    end
    
    % Configure plot
    if p.Results.Axis
        axis equal;
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    end
    if p.Results.Grid, grid on; else, grid off; end
    view(p.Results.View);
    title(p.Results.Title);
    
    fprintf('[Visualization] Point cloud plotted (%d points)\n', size(pt,1));
end

%% Plot cross-sections, splines, and centerline with proper orientation
function [hFig, hSections, hCenterline, hSplines] = visualization_plotSectionsAndSplines(sections, splines, centerline, varargin)
    % Input validation
    if isempty(sections) || isempty(centerline)
        error('Sections and centerline must be provided');
    end
    
    % Parse optional parameters
    p = inputParser;
    addParameter(p, 'Parent', []);
    addParameter(p, 'Title', 'Propeller Sections, Splines, and Centerline');
    addParameter(p, 'ShowSections', true);
    addParameter(p, 'ShowCenterline', true);
    addParameter(p, 'ShowSplines', true);
    addParameter(p, 'View', [30, 45]);
    parse(p, varargin{:});
    
    % Create figure
    if isempty(p.Results.Parent)
        hFig = createFigure(p.Results.Title);
    else
        hFig = p.Results.Parent;
        figure(hFig);
        hold on;
    end
    
    hSections = [];
    hCenterline = [];
    hSplines = [];
    
    % Plot cross-sections
    if p.Results.ShowSections
        colors = defaultColors(mod(0:numel(sections)-1, size(defaultColors,1)) + 1, :);
        for i = 1:numel(sections)
            if ~isempty(sections(i).points)
                % Use more efficient coloring
                hSection = scatter3(sections(i).points(:,1), ...
                                   sections(i).points(:,2), ...
                                   sections(i).points(:,3), ...
                                   5, colors(i,:), 'filled');
                hSections = [hSections; hSection];
            end
        end
    end
    
    % Plot centerline
    if p.Results.ShowCenterline && ~isempty(centerline)
        hCenterline = plot3(centerline(:,1), centerline(:,2), centerline(:,3), ...
                           'k-', 'LineWidth', 3, 'Marker', 'o', 'MarkerSize', 4);
    end
    
    % Plot spline curves with proper 3D orientation
    if p.Results.ShowSplines && ~isempty(splines)
        hSplines = gobjects(numel(splines), 1);
        for i = 1:numel(splines)
            if ~isempty(splines(i)) && isfield(splines(i), 'points') && ~isempty(splines(i).points)
                % Plot spline in 3D space using actual section orientation
                splinePoints = splines(i).points;
                hSplines(i) = plot3(splinePoints(:,1), splinePoints(:,2), splinePoints(:,3), ...
                                   'r-', 'LineWidth', 2);
            end
        end
    end
    
    % Configure plot
    axis equal; grid on;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    view(p.Results.View);
    title(p.Results.Title);
    
    legend([hCenterline, hSplines(1)], {'Centerline', 'Airfoil Splines'});
    
    fprintf('[Visualization] Plotted %d sections, centerline, and %d splines\n', ...
            numel(sections), numel(splines));
end

%% Plot hub and blades separately with improved labeling
function [hFig, hSegments] = visualization_plotSegments(segments, varargin)
    % Input validation
    if isempty(segments)
        error('Segments must be provided');
    end
    
    % Parse optional parameters
    p = inputParser;
    addParameter(p, 'Parent', []);
    addParameter(p, 'Title', 'Hub and Blade Segments');
    addParameter(p, 'Labels', {'Hub', 'Blade 1', 'Blade 2', 'Blade 3', 'Blade 4'});
    addParameter(p, 'View', [30, 45]);
    parse(p, varargin{:});
    
    % Create figure
    if isempty(p.Results.Parent)
        hFig = createFigure(p.Results.Title);
    else
        hFig = p.Results.Parent;
        figure(hFig);
        hold on;
    end
    
    hSegments = gobjects(numel(segments), 1);
    legendEntries = {};
    
    % Plot each segment
    colors = defaultColors(mod(0:numel(segments)-1, size(defaultColors,1)) + 1, :);
    
    for i = 1:numel(segments)
        if isa(segments(i).pointCloud, 'pointCloud')
            pts = segments(i).pointCloud.Location;
        else
            pts = segments(i).pointCloud;
        end
        
        if ~isempty(pts)
            hSegments(i) = scatter3(pts(:,1), pts(:,2), pts(:,3), 5, ...
                                   colors(i,:), 'filled');
            
            % Create legend entry
            if i <= length(p.Results.Labels)
                legendEntries{end+1} = p.Results.Labels{i};
            else
                legendEntries{end+1} = sprintf('Segment %d', i);
            end
        end
    end
    
    % Configure plot
    axis equal; grid on;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    view(p.Results.View);
    title(p.Results.Title);
    legend(hSegments(ishandle(hSegments)), legendEntries, 'Location', 'best');
    
    fprintf('[Visualization] Plotted %d segments\n', numel(segments));
end

%% Additional utility visualization functions
function visualization_plotComparison(original, processed, varargin)
    % Side-by-side comparison of original and processed point clouds
    hFig = figure('Position', [100, 100, 1200, 500]);
    
    subplot(1,2,1);
    visualization_plotPointCloud(original, 'Parent', gca, 'Title', 'Original Point Cloud');
    
    subplot(1,2,2);
    visualization_plotPointCloud(processed, 'Parent', gca, 'Title', 'Processed Point Cloud');
    
    suptitle('Point Cloud Processing Comparison');
end

function visualization_plotCurvature(centerline, curvature, varargin)
    % Plot centerline colored by curvature
    hFig = createFigure('Centerline Curvature');
    
    % Color points by curvature
    scatter3(centerline(:,1), centerline(:,2), centerline(:,3), 40, curvature, 'filled');
    
    axis equal; grid on; colorbar;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Centerline Curvature');
end

%% Helper functions
function hFig = createFigure(titleStr)
    hFig = figure('NumberTitle', 'off', 'Name', titleStr, ...
                 'Position', [100, 100, 800, 600], ...
                 'Color', 'w');
    hold on; grid on; axis equal;
    figCount = figCount + 1;
end

function closeAllVisualizations()
    % Close all visualization figures
    figHandles = findall(0, 'Type', 'figure');
    for i = 1:length(figHandles)
        if contains(get(figHandles(i), 'Name'), 'Propeller') || ...
           contains(get(figHandles(i), 'Name'), 'Visualization')
            close(figHandles(i));
        end
    end
    fprintf('[Visualization] Closed all visualization figures\n');
end
