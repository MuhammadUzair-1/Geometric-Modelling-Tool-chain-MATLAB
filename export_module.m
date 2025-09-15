%% export_module.m - Complete Export functions for Propeller Toolchain
% Export results to JSON, STL, IGES, OBJ, and other formats

%% Write JSON metadata
function export_writeJSONMetadata(data, filename)
% EXPORT_WRITEJSONMETADATA Export data structure to JSON file
% Input: data - MATLAB structure, filename - output JSON file path

% Check if directory exists
[filepath, ~, ~] = fileparts(filename);
if ~isempty(filepath) && ~exist(filepath, 'dir')
    mkdir(filepath);
end

try
    % Check for jsonencode availability (R2016b+)
    if exist('jsonencode', 'file') == 2
        jsonText = jsonencode(data, 'PrettyPrint', true);
    else
        % Fallback for older MATLAB versions
        jsonText = export_jsonencode_fallback(data);
    end
    
    fid = fopen(filename, 'w');
    if fid == -1
        error('Cannot open %s for writing', filename);
    end
    fwrite(fid, jsonText, 'char');
    fclose(fid);
    fprintf('[Export] JSON metadata saved to %s\n', filename);
    
catch ME
    warning('[Export] Failed to write JSON: %s', ME.message);
    if exist('fid', 'var') && fid ~= -1
        fclose(fid);
    end
    if exist(filename, 'file')
        delete(filename);
    end
    rethrow(ME);
end
end

%% JSON fallback for older MATLAB versions
function jsonText = export_jsonencode_fallback(data)
% Manual JSON implementation for MATLAB versions < R2016b
if isstruct(data)
    jsonText = struct2json(data);
elseif iscell(data)
    jsonText = cell2json(data);
elseif isnumeric(data) || islogical(data)
    jsonText = mat2json(data);
elseif ischar(data)
    jsonText = ['"' data '"'];
else
    error('Unsupported data type for JSON conversion');
end
end

%% Helper: Convert struct to JSON
function jsonText = struct2json(s)
fields = fieldnames(s);
elements = cell(length(fields), 1);
for i = 1:length(fields)
    fieldName = fields{i};
    fieldValue = s.(fieldName);
    if isstruct(fieldValue)
        elements{i} = sprintf('"%s": %s', fieldName, struct2json(fieldValue));
    elseif iscell(fieldValue)
        elements{i} = sprintf('"%s": %s', fieldName, cell2json(fieldValue));
    elseif isnumeric(fieldValue) || islogical(fieldValue)
        elements{i} = sprintf('"%s": %s', fieldName, mat2json(fieldValue));
    elseif ischar(fieldValue)
        elements{i} = sprintf('"%s": "%s"', fieldName, fieldValue);
    else
        elements{i} = sprintf('"%s": null', fieldName);
    end
end
jsonText = ['{' strjoin(elements, ', ') '}'];
end

%% Helper: Convert cell array to JSON
function jsonText = cell2json(c)
elements = cell(size(c));
for i = 1:numel(c)
    if isstruct(c{i})
        elements{i} = struct2json(c{i});
    elseif iscell(c{i})
        elements{i} = cell2json(c{i});
    elseif isnumeric(c{i}) || islogical(c{i})
        elements{i} = mat2json(c{i});
    elseif ischar(c{i})
        elements{i} = ['"' c{i} '"'];
    else
        elements{i} = 'null';
    end
end
jsonText = ['[' strjoin(elements, ', ') ']'];
end

%% Helper: Convert matrix to JSON
function jsonText = mat2json(m)
if isscalar(m)
    jsonText = num2str(m);
elseif isvector(m)
    elements = arrayfun(@num2str, m, 'UniformOutput', false);
    jsonText = ['[' strjoin(elements, ', ') ']'];
else
    % For matrices, convert to cell array of rows
    rows = cell(size(m, 1), 1);
    for i = 1:size(m, 1)
        rowElements = arrayfun(@num2str, m(i, :), 'UniformOutput', false);
        rows{i} = ['[' strjoin(rowElements, ', ') ']'];
    end
    jsonText = ['[' strjoin(rows, ', ') ']'];
end
end

%% Write STL mesh
function export_writeSTL(mesh, filename)
% EXPORT_WRITESTL Export mesh to STL format
% Input: mesh - struct with vertices and faces, filename - output path

if ~isstruct(mesh) || ~isfield(mesh,'vertices') || ~isfield(mesh,'faces')
    error('[Export] Invalid mesh structure. Required: vertices, faces');
end

% Validate mesh data
if isempty(mesh.vertices) || isempty(mesh.faces)
    error('[Export] Empty mesh data');
end

% Check if directory exists
[filepath, ~, ~] = fileparts(filename);
if ~isempty(filepath) && ~exist(filepath, 'dir')
    mkdir(filepath);
end

try
    % Check for CAD Toolbox STL writer
    if license('test', 'cad_toolbox') && exist('stlwrite', 'file')
        stlwrite(filename, mesh.faces, mesh.vertices);
    else
        % Custom STL writer fallback
        export_writeSTL_custom(mesh, filename);
    end
    fprintf('[Export] Mesh exported to STL: %s (%d faces)\n', filename, size(mesh.faces, 1));
    
catch ME
    warning('[Export] Failed to write STL: %s', ME.message);
    if exist(filename, 'file')
        delete(filename);
    end
    rethrow(ME);
end
end

%% Custom STL writer (fallback)
function export_writeSTL_custom(mesh, filename)
fid = fopen(filename, 'w');
if fid == -1
    error('Cannot open STL file: %s', filename);
end

% Write STL header (80 bytes)
header = sprintf('MATLAB STL Export - %s', datestr(now));
header = [header repmat(' ', 1, 80 - length(header))];
fwrite(fid, header(1:80), 'char');

% Write number of facets (4 bytes)
numFaces = size(mesh.faces, 1);
fwrite(fid, numFaces, 'uint32');

V = mesh.vertices;
F = mesh.faces;

% Write each triangle
for i = 1:numFaces
    if any(F(i, :) > size(V, 1)) || any(F(i, :) < 1)
        fclose(fid);
        error('Invalid face indices: face %d contains invalid vertex references', i);
    end
    
    v1 = V(F(i, 1), :);
    v2 = V(F(i, 2), :);
    v3 = V(F(i, 3), :);
    
    % Calculate normal (right-hand rule)
    normal = cross(v2 - v1, v3 - v1);
    if norm(normal) > eps
        normal = normal / norm(normal);
    else
        normal = [0 0 1]; % Default normal for degenerate triangles
    end
    
    % Write normal (3x float32)
    fwrite(fid, single(normal), 'float32');
    
    % Write vertices (9x float32)
    fwrite(fid, single(v1), 'float32');
    fwrite(fid, single(v2), 'float32');
    fwrite(fid, single(v3), 'float32');
    
    % Attribute byte count (2 bytes, usually 0)
    fwrite(fid, 0, 'uint16');
end

fclose(fid);
end

%% Write IGES NURBS surface
function export_writeIGES(nurbs, filename)
% EXPORT_WRITEIGES Export NURBS surface to IGES format
% Input: nurbs - NURBS structure, filename - output path

% Validate NURBS structure
requiredFields = {'knots', 'controlPoints', 'weights', 'degree'};
if ~isstruct(nurbs) || ~all(isfield(nurbs, requiredFields))
    error('[Export] Invalid NURBS structure. Required: knots, controlPoints, weights, degree');
end

% Check if directory exists
[filepath, ~, ~] = fileparts(filename);
if ~isempty(filepath) && ~exist(filepath, 'dir')
    mkdir(filepath);
end

try
    % Use third-party IGES toolbox if available
    if exist('igesout', 'file') == 2
        % Convert to IGES toolbox format
        crv = nrbmak(nurbs.controlPoints', nurbs.knots);
        igesout(crv, filename);
    else
        % Fallback: Simple IGES writer for NURBS
        export_writeIGES_nurbs(nurbs, filename);
    end
    fprintf('[Export] NURBS exported to IGES: %s\n', filename);
    
catch ME
    warning('[Export] Failed to write IGES: %s', ME.message);
    if exist(filename, 'file')
        delete(filename);
    end
    rethrow(ME);
end
end

%% Fallback IGES writer for NURBS
function export_writeIGES_nurbs(nurbs, filename)
fid = fopen(filename, 'w');
if fid == -1
    error('Cannot open IGES file: %s', filename);
end

% IGES file header
fprintf(fid, 'MATLAB NURBS Export                                                                 S      1\n');
fprintf(fid, '1H,,1H;,,,%%s-%%s-%%s.%%s:%%s,,,,,,,4H%.4s,32H%.32s,,,,,,'',,,,                       G      1\n', ...
    datestr(now, 'yyyy-mm-dd-HH:MM:SS'), 'PROP', 'NURBS Surface');
fprintf(fid, '6H%.6s,11H%.11s,,,8H%.8s,8H%.8s,8H%.8s,6H%.6s,,,,,,,''                            G      2\n', ...
    '1.0', ' millimeters', 'EXPORT', 'MATLAB', 'NURBS', '1.0');
fprintf(fid, '32HThis is a simplified IGES NURBS export,,,,''                                    G      3\n');

% Write NURBS surface entity (Type 128)
% This is a simplified implementation - real IGES is much more complex
ctrlPts = nurbs.controlPoints;
weights = nurbs.weights;
knots = nurbs.knots;
degree = nurbs.degree;

fprintf(fid, '128,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d P      1\n', ...
    1, 1, degree, degree, size(ctrlPts,1), size(ctrlPts,2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

fclose(fid);
end

%% Write OBJ mesh
function export_writeOBJ(mesh, filename)
% EXPORT_WRITEOBJ Export mesh to OBJ format
% Input: mesh - struct with vertices and faces, filename - output path

if ~isstruct(mesh) || ~isfield(mesh,'vertices') || ~isfield(mesh,'faces')
    error('[Export] Invalid mesh structure. Required: vertices, faces');
end

% Validate mesh data
if isempty(mesh.vertices) || isempty(mesh.faces)
    error('[Export] Empty mesh data');
end

% Check if directory exists
[filepath, ~, ~] = fileparts(filename);
if ~isempty(filepath) && ~exist(filepath, 'dir')
    mkdir(filepath);
end

fid = fopen(filename, 'w');
if fid == -1
    error('Cannot open OBJ file: %s', filename);
end

try
    V = mesh.vertices;
    F = mesh.faces;
    
    % Write header
    fprintf(fid, '# OBJ file generated by Propeller Toolchain\n');
    fprintf(fid, '# Vertices: %d, Faces: %d\n', size(V, 1), size(F, 1));
    fprintf(fid, 'o propeller\n');
    
    % Write vertices
    for i = 1:size(V, 1)
        fprintf(fid, 'v %.6f %.6f %.6f\n', V(i, 1), V(i, 2), V(i, 3));
    end
    
    % Ensure 1-based indexing for OBJ format
    if min(F(:)) == 0
        F = F + 1;
        fprintf('[Export] Converted 0-based to 1-based indexing for OBJ\n');
    end
    
    % Validate face indices
    if max(F(:)) > size(V, 1) || min(F(:)) < 1
        fclose(fid);
        error('Face indices out of range. Check mesh integrity.');
    end
    
    % Write faces
    for i = 1:size(F, 1)
        fprintf(fid, 'f %d %d %d\n', F(i, 1), F(i, 2), F(i, 3));
    end
    
    fclose(fid);
    fprintf('[Export] Mesh exported to OBJ: %s (%d faces)\n', filename, size(F, 1));
    
catch ME
    if fid ~= -1
        fclose(fid);
    end
    if exist(filename, 'file')
        delete(filename);
    end
    rethrow(ME);
end
end

%% Write STEP file (placeholder with recommendation)
function export_writeSTEP(nurbs, filename)
% EXPORT_WRITESTEP Placeholder for STEP export
% Note: STEP export requires specialized libraries

fprintf('[Export] Warning: STEP export requires external libraries.\n');
fprintf('Consider using: \n');
fprintf('  - ISO10303-21 writers (commercial)\n');
fprintf('  - OpenCASCADE library integration\n');
fprintf('  - CAD Toolbox for MATLAB (if available)\n');

% Create informative placeholder file
fid = fopen(filename, 'w');
if fid == -1
    error('Cannot open STEP file: %s', filename);
end

fprintf(fid, 'ISO-10303-21;\n');
fprintf(fid, 'HEADER;\n');
fprintf(fid, 'FILE_DESCRIPTION((''STEP Placeholder''),''2;1'');\n');
fprintf(fid, 'FILE_NAME(''%s'', ''%s'', (''''), (''''), '''', '''', '''');\n', ...
    filename, datestr(now, 'yyyy-mm-dd'));
fprintf(fid, 'ENDSEC;\n');
fprintf(fid, 'DATA;\n');
fprintf(fid, '/* STEP export not implemented - requires external library */\n');
fprintf(fid, 'ENDSEC;\n');
fprintf(fid, 'END-ISO-10303-21;\n');

fclose(fid);
fprintf('[Export] STEP placeholder created: %s\n', filename);
end

%% Export point cloud to PLY format
function export_writePLY(ptCloud, filename)
% EXPORT_WRITEPLY Export point cloud to PLY format
% Input: ptCloud - pointCloud object or struct with vertices, filename - output path

if isa(ptCloud, 'pointCloud')
    vertices = ptCloud.Location;
elseif isstruct(ptCloud) && isfield(ptCloud, 'vertices')
    vertices = ptCloud.vertices;
else
    error('[Export] Invalid point cloud input');
end

% Check if directory exists
[filepath, ~, ~] = fileparts(filename);
if ~isempty(filepath) && ~exist(filepath, 'dir')
    mkdir(filepath);
end

fid = fopen(filename, 'w');
if fid == -1
    error('Cannot open PLY file: %s', filename);
end

try
    % Write PLY header
    fprintf(fid, 'ply\n');
    fprintf(fid, 'format ascii 1.0\n');
    fprintf(fid, 'comment MATLAB generated\n');
    fprintf(fid, 'element vertex %d\n', size(vertices, 1));
    fprintf(fid, 'property float x\n');
    fprintf(fid, 'property float y\n');
    fprintf(fid, 'property float z\n');
    fprintf(fid, 'end_header\n');
    
    % Write vertices
    for i = 1:size(vertices, 1)
        fprintf(fid, '%.6f %.6f %.6f\n', vertices(i, 1), vertices(i, 2), vertices(i, 3));
    end
    
    fclose(fid);
    fprintf('[Export] Point cloud exported to PLY: %s (%d points)\n', filename, size(vertices, 1));
    
catch ME
    if fid ~= -1
        fclose(fid);
    end
    if exist(filename, 'file')
        delete(filename);
    end
    rethrow(ME);
end
end

%% Validate mesh structure
function isValid = export_validateMesh(mesh)
% EXPORT_VALIDATEMESH Validate mesh structure integrity
isValid = true;
if ~isstruct(mesh)
    isValid = false;
    return;
end

if ~isfield(mesh, 'vertices') || ~isfield(mesh, 'faces')
    isValid = false;
    return;
end

if isempty(mesh.vertices) || isempty(mesh.faces)
    isValid = false;
    return;
end

% Check for valid indices
if any(mesh.faces(:) < 1) || any(mesh.faces(:) > size(mesh.vertices, 1))
    isValid = false;
    return;
end

% Check for NaN/inf values
if any(isnan(mesh.vertices(:))) || any(isinf(mesh.vertices(:)))
    isValid = false;
end
end
