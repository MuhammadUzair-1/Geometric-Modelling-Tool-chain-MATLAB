%% io_module.m - Complete IO functions for Propeller Toolchain
% All IO-related functions with robust error handling

%% Read point cloud with multiple format support
function ptCloud = io_readPointCloud(filename)
% IO_READPOINTCLOUD Read point cloud from various formats
% Supports: PLY, PCD, XYZ, TXT, MAT

if ~exist(filename, 'file')
    error('File not found: %s', filename);
end

[~, ~, ext] = fileparts(filename);
try
    switch lower(ext)
        case {'.ply', '.pcd'}
            % Use MATLAB's built-in readers
            ptCloud = pcread(filename);
            
        case {'.xyz', '.txt'}
            % Read ASCII point cloud
            data = readmatrix(filename);
            if size(data, 2) < 3
                error('File must have at least 3 columns for X,Y,Z');
            end
            
            % Handle optional color/intensity
            if size(data, 2) >= 6
                % Assume XYZRGB format
                ptCloud = pointCloud(data(:, 1:3), 'Color', uint8(data(:, 4:6)));
            elseif size(data, 2) >= 4
                % Assume XYZI format (intensity)
                ptCloud = pointCloud(data(:, 1:3), 'Intensity', data(:, 4));
            else
                % XYZ only
                ptCloud = pointCloud(data(:, 1:3));
            end
            
        case '.mat'
            % MATLAB format - expect pointCloud object or vertices
            data = load(filename);
            if isfield(data, 'ptCloud')
                ptCloud = data.ptCloud;
            elseif isfield(data, 'vertices')
                ptCloud = pointCloud(data.vertices);
            else
                error('MAT file must contain ptCloud or vertices variable');
            end
            
        otherwise
            error('Unsupported point cloud format: %s', ext);
    end
    
    fprintf('[IO] Point cloud loaded: %s (%d points)\n', filename, ptCloud.Count);
    
catch ME
    error('Failed to read point cloud from %s: %s', filename, ME.message);
end
end

%% Write point cloud with format options
function io_writePointCloud(ptCloud, filename, format)
% IO_WRITEPOINTCLOUD Write point cloud to file
% format: 'binary' (default) or 'ascii'

if nargin < 3
    format = 'binary';
end

[filepath, ~, ext] = fileparts(filename);
if ~isempty(filepath) && ~exist(filepath, 'dir')
    mkdir(filepath);
end

try
    switch lower(ext)
        case {'.ply', '.pcd'}
            pcwrite(ptCloud, filename, 'PLYFormat', format);
            
        case {'.xyz', '.txt'}
            % Write ASCII format
            fid = fopen(filename, 'w');
            if fid == -1
                error('Cannot open file: %s', filename);
            end
            
            if ~isempty(ptCloud.Color)
                % Write XYZRGB
                for i = 1:ptCloud.Count
                    fprintf(fid, '%.6f %.6f %.6f %d %d %d\n', ...
                        ptCloud.Location(i, 1), ptCloud.Location(i, 2), ptCloud.Location(i, 3), ...
                        ptCloud.Color(i, 1), ptCloud.Color(i, 2), ptCloud.Color(i, 3));
                end
            elseif ~isempty(ptCloud.Intensity)
                % Write XYZI
                for i = 1:ptCloud.Count
                    fprintf(fid, '%.6f %.6f %.6f %.6f\n', ...
                        ptCloud.Location(i, 1), ptCloud.Location(i, 2), ptCloud.Location(i, 3), ...
                        ptCloud.Intensity(i));
                end
            else
                % Write XYZ only
                for i = 1:ptCloud.Count
                    fprintf(fid, '%.6f %.6f %.6f\n', ...
                        ptCloud.Location(i, 1), ptCloud.Location(i, 2), ptCloud.Location(i, 3));
                end
            end
            fclose(fid);
            
        case '.mat'
            % Save as MATLAB format
            save(filename, 'ptCloud', '-v7.3');
            
        otherwise
            error('Unsupported output format: %s', ext);
    end
    
    fprintf('[IO] Point cloud saved: %s (%d points)\n', filename, ptCloud.Count);
    
catch ME
    error('Failed to write point cloud to %s: %s', filename, ME.message);
end
end

%% Read mesh with robust format support
function mesh = io_readMesh(filename)
% IO_READMESH Read mesh from various formats
% Supports: STL, OBJ, MAT

if ~exist(filename, 'file')
    error('File not found: %s', filename);
end

[~, ~, ext] = fileparts(filename);
mesh = struct('vertices', [], 'faces', []);

try
    switch lower(ext)
        case '.stl'
            if license('test', 'cad_toolbox') && exist('stlread', 'file')
                [F, V] = stlread(filename);
            else
                [V, F] = io_readSTL_custom(filename);
            end
            
        case '.obj'
            [V, F] = io_readOBJ_robust(filename);
            
        case '.mat'
            data = load(filename);
            if isfield(data, 'mesh')
                mesh = data.mesh;
            elseif isfield(data, 'vertices') && isfield(data, 'faces')
                mesh.vertices = data.vertices;
                mesh.faces = data.faces;
            else
                error('MAT file must contain mesh or vertices/faces variables');
            end
            return;
            
        otherwise
            error('Unsupported mesh format: %s', ext);
    end
    
    mesh.vertices = V;
    mesh.faces = F;
    
    fprintf('[IO] Mesh loaded: %s (%d vertices, %d faces)\n', filename, size(V, 1), size(F, 1));
    
catch ME
    error('Failed to read mesh from %s: %s', filename, ME.message);
end
end

%% Custom STL reader (fallback)
function [vertices, faces] = io_readSTL_custom(filename)
% Custom STL reader for binary STL files
fid = fopen(filename, 'r');
if fid == -1
    error('Cannot open STL file: %s', filename);
end

% Read header
header = fread(fid, 80, 'char=>char')';
numFaces = fread(fid, 1, 'uint32');

vertices = zeros(numFaces * 3, 3);
faces = zeros(numFaces, 3);

for i = 1:numFaces
    % Read normal (skip)
    fread(fid, 3, 'float32');
    
    % Read vertices
    v1 = fread(fid, 3, 'float32')';
    v2 = fread(fid, 3, 'float32')';
    v3 = fread(fid, 3, 'float32')';
    
    % Skip attribute byte count
    fread(fid, 1, 'uint16');
    
    % Store vertices and faces
    startIdx = (i-1)*3 + 1;
    vertices(startIdx:startIdx+2, :) = [v1; v2; v3];
    faces(i, :) = [startIdx, startIdx+1, startIdx+2];
end

fclose(fid);
end

%% Robust OBJ reader
function [vertices, faces] = io_readOBJ_robust(filename)
% Robust OBJ reader handling multiple format variations
fid = fopen(filename, 'r');
if fid == -1
    error('Cannot open OBJ file: %s', filename);
end

% Pre-allocate using file size estimate
fileInfo = dir(filename);
approxLines = round(fileInfo.bytes / 50); % Rough estimate
vertexList = cell(approxLines, 1);
faceList = cell(approxLines, 1);
vCount = 0;
fCount = 0;

while ~feof(fid)
    line = fgetl(fid);
    if isempty(line) || line(1) == '#'
        continue;
    end
    
    tokens = strsplit(strtrim(line));
    if isempty(tokens)
        continue;
    end
    
    switch tokens{1}
        case 'v'
            % Vertex
            vCount = vCount + 1;
            if length(tokens) >= 4
                vertexList{vCount} = str2double(tokens(2:4));
            end
            
        case 'f'
            % Face - handle multiple formats
            fCount = fCount + 1;
            faceVerts = cell(length(tokens)-1, 1);
            
            for i = 2:length(tokens)
                % Handle v, v/vt, v/vt/vn, v//vn formats
                parts = strsplit(tokens{i}, '/');
                if ~isempty(parts{1})
                    faceVerts{i-1} = str2double(parts{1});
                end
            end
            
            % Remove empty and convert to numeric
            faceVerts = faceVerts(cellfun(@(x) ~isempty(x), faceVerts));
            faceList{fCount} = cell2mat(faceVerts);
    end
end

fclose(fid);

% Convert to arrays
vertices = cell2mat(vertexList(1:vCount));
faces = cell2mat(faceList(1:fCount));

% Ensure 1-based indexing
if min(faces(:)) == 0
    faces = faces + 1;
end
end

%% Write mesh with format options
function io_writeMesh(mesh, filename, format)
% IO_WRITEMESH Write mesh to various formats
% format: 'binary' or 'ascii' (for STL)

if nargin < 3
    format = 'binary';
end

if ~io_validateMesh(mesh)
    error('Invalid mesh structure');
end

[filepath, ~, ext] = fileparts(filename);
if ~isempty(filepath) && ~exist(filepath, 'dir')
    mkdir(filepath);
end

try
    switch lower(ext)
        case '.stl'
            if license('test', 'cad_toolbox') && exist('stlwrite', 'file')
                stlwrite(filename, mesh.faces, mesh.vertices);
            else
                io_writeSTL_custom(mesh, filename, format);
            end
            
        case '.obj'
            io_writeOBJ_robust(mesh, filename);
            
        case '.mat'
            save(filename, 'mesh', '-v7.3');
            
        otherwise
            error('Unsupported mesh format: %s', ext);
    end
    
    fprintf('[IO] Mesh saved: %s (%d faces)\n', filename, size(mesh.faces, 1));
    
catch ME
    error('Failed to write mesh to %s: %s', filename, ME.message);
end
end

%% Custom STL writer
function io_writeSTL_custom(mesh, filename, format)
% Custom STL writer supporting ASCII and binary
if strcmpi(format, 'ascii')
    fid = fopen(filename, 'w');
    fprintf(fid, 'solid MATLAB_Export\n');
    
    for i = 1:size(mesh.faces, 1)
        v1 = mesh.vertices(mesh.faces(i, 1), :);
        v2 = mesh.vertices(mesh.faces(i, 2), :);
        v3 = mesh.vertices(mesh.faces(i, 3), :);
        
        normal = cross(v2 - v1, v3 - v1);
        normal = normal / norm(normal);
        
        fprintf(fid, 'facet normal %f %f %f\n', normal);
        fprintf(fid, '  outer loop\n');
        fprintf(fid, '    vertex %f %f %f\n', v1);
        fprintf(fid, '    vertex %f %f %f\n', v2);
        fprintf(fid, '    vertex %f %f %f\n', v3);
        fprintf(fid, '  endloop\n');
        fprintf(fid, 'endfacet\n');
    end
    
    fprintf(fid, 'endsolid MATLAB_Export\n');
    fclose(fid);
    
else
    % Binary format
    fid = fopen(filename, 'wb');
    header = sprintf('MATLAB STL Export %s', datestr(now));
    header = [header repmat(' ', 1, 80 - length(header))];
    fwrite(fid, header, 'char');
    fwrite(fid, size(mesh.faces, 1), 'uint32');
    
    for i = 1:size(mesh.faces, 1)
        v1 = mesh.vertices(mesh.faces(i, 1), :);
        v2 = mesh.vertices(mesh.faces(i, 2), :);
        v3 = mesh.vertices(mesh.faces(i, 3), :);
        
        normal = cross(v2 - v1, v3 - v1);
        normal = normal / norm(normal);
        
        fwrite(fid, single(normal), 'float32');
        fwrite(fid, single(v1), 'float32');
        fwrite(fid, single(v2), 'float32');
        fwrite(fid, single(v3), 'float32');
        fwrite(fid, 0, 'uint16');
    end
    
    fclose(fid);
end
end

%% Robust OBJ writer
function io_writeOBJ_robust(mesh, filename)
fid = fopen(filename, 'w');
if fid == -1
    error('Cannot open OBJ file: %s', filename);
end

fprintf(fid, '# OBJ file generated by Propeller Toolchain\n');
fprintf(fid, '# Vertices: %d, Faces: %d\n', size(mesh.vertices, 1), size(mesh.faces, 1));

% Write vertices
for i = 1:size(mesh.vertices, 1)
    fprintf(fid, 'v %.6f %.6f %.6f\n', mesh.vertices(i, 1), mesh.vertices(i, 2), mesh.vertices(i, 3));
end

% Write faces (ensure 1-based indexing)
faces = mesh.faces;
if min(faces(:)) == 0
    faces = faces + 1;
end

for i = 1:size(faces, 1)
    fprintf(fid, 'f %d %d %d\n', faces(i, 1), faces(i, 2), faces(i, 3));
end

fclose(fid);
end

%% Validate mesh structure
function isValid = io_validateMesh(mesh)
isValid = true;
if ~isstruct(mesh) || ~isfield(mesh, 'vertices') || ~isfield(mesh, 'faces')
    isValid = false;
    return;
end

if isempty(mesh.vertices) || isempty(mesh.faces)
    isValid = false;
    return;
end

if any(mesh.faces(:) < 1) || any(mesh.faces(:) > size(mesh.vertices, 1))
    isValid = false;
    return;
end

if any(isnan(mesh.vertices(:))) || any(isinf(mesh.vertices(:)))
    isValid = false;
end
end
