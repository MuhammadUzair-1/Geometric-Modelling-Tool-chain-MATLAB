%% utils_module.m - Enhanced utility functions for Propeller Toolchain

%% Global configuration
persistent logLevel logFile timerStack

if isempty(logLevel)
    logLevel = 'INFO'; % Default log level: DEBUG, INFO, WARN, ERROR
    logFile = ''; % No log file by default
    timerStack = struct('name', {}, 'startTime', {});
end

%% Enhanced logging helper with levels and file support
function utils_logger(level, msg, varargin)
    persistent logLevelMap
    if isempty(logLevelMap)
        logLevelMap = containers.Map({'DEBUG', 'INFO', 'WARN', 'ERROR'}, [1, 2, 3, 4]);
    end
    
    % Check if this message should be logged
    if logLevelMap(level) < logLevelMap(logLevel)
        return;
    end
    
    % Format message with optional arguments
    if ~isempty(varargin)
        msg = sprintf(msg, varargin{:});
    end
    
    % Add timestamp
    timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS.FFF');
    logMsg = sprintf('[%s] [%s] %s\n', timestamp, upper(level), msg);
    
    % Output to console
    fprintf('%s', logMsg);
    
    % Output to file if configured
    if ~isempty(logFile)
        fid = fopen(logFile, 'a');
        if fid ~= -1
            fprintf(fid, '%s', logMsg);
            fclose(fid);
        end
    end
end

%% Configure logging
function utils_setLogLevel(level)
    validLevels = {'DEBUG', 'INFO', 'WARN', 'ERROR'};
    if any(strcmpi(level, validLevels))
        logLevel = upper(level);
        utils_logger('INFO', 'Log level set to %s', logLevel);
    else
        utils_logger('ERROR', 'Invalid log level: %s. Use: DEBUG, INFO, WARN, ERROR', level);
    end
end

function utils_setLogFile(filename)
    if isempty(filename)
        logFile = '';
        utils_logger('INFO', 'Log file disabled');
    else
        try
            % Test if we can write to the file
            fid = fopen(filename, 'a');
            if fid == -1
                error('Cannot open log file: %s', filename);
            end
            fclose(fid);
            logFile = filename;
            utils_logger('INFO', 'Log file set to: %s', filename);
        catch ME
            utils_logger('ERROR', 'Failed to set log file: %s', ME.message);
        end
    end
end

%% Robust dependency checking with error handling
function [deps, missingDeps] = utils_checkDependencies(requiredDeps)
    if nargin < 1
        requiredDeps = {'computerVision', 'curveFitting', 'statistics'};
    end
    
    deps = struct();
    missingDeps = {};
    
    % Correct toolbox license names and validation functions
    toolboxInfo = {
        'computerVision', 'video_and_image_blockset', 'pointCloud', @() exist('pointCloud', 'class') == 8;
        'curveFitting',   'curve_fitting_toolbox',    'spap2',     @() exist('spap2', 'file') == 2;
        'statistics',     'statistics_toolbox',       'pca',       @() exist('pca', 'file') == 2;
        'parallel',       'distrib_computing_toolbox','parfor',    @() exist('parfor', 'builtin') == 5;
    };
    
    for i = 1:size(toolboxInfo, 1)
        name = toolboxInfo{i,1};
        licenseName = toolboxInfo{i,2};
        testFunction = toolboxInfo{i,4};
        
        % Check both license and function existence
        hasLicense = license('test', licenseName);
        hasFunction = testFunction();
        
        deps.(name) = hasLicense && hasFunction;
        
        if ~deps.(name)
            missingDeps{end+1} = name;
            utils_logger('WARN', 'Missing dependency: %s', name);
        end
    end
    
    % Check if required dependencies are missing
    if ~isempty(missingDeps) && nargin > 0
        missingRequired = intersect(missingDeps, requiredDeps);
        if ~isempty(missingRequired)
            error('Missing required dependencies: %s', strjoin(missingRequired, ', '));
        end
    end
    
    utils_logger('INFO', 'Dependency check completed');
end

%% Enhanced timer utility with stack support
function tHandle = utils_startTimer(msg)
    if nargin < 1
        msg = 'Unnamed timer';
    end
    
    % Push timer to stack
    newTimer.name = msg;
    newTimer.startTime = tic;
    timerStack(end+1) = newTimer;
    
    tHandle = length(timerStack); % Return stack index as handle
    utils_logger('DEBUG', 'Timer started: %s', msg);
end

function elapsed = utils_stopTimer(tHandle, msg)
    if nargin < 2
        if ~isempty(timerStack) && tHandle <= length(timerStack)
            msg = timerStack(tHandle).name;
        else
            msg = 'Timer';
        end
    end
    
    if isempty(timerStack) || tHandle > length(timerStack)
        utils_logger('WARN', 'Invalid timer handle: %d', tHandle);
        elapsed = NaN;
        return;
    end
    
    elapsed = toc(timerStack(tHandle).startTime);
    
    % Pop timer from stack
    timerStack(tHandle) = [];
    
    utils_logger('INFO', '%s: %.3f seconds', msg, elapsed);
    
    if nargout == 0
        clear elapsed; % Don't output if not requested
    end
end

%% Additional utility functions
function success = utils_createFolder(folderPath)
    try
        if ~exist(folderPath, 'dir')
            mkdir(folderPath);
            utils_logger('INFO', 'Created folder: %s', folderPath);
        end
        success = true;
    catch ME
        utils_logger('ERROR', 'Failed to create folder %s: %s', folderPath, ME.message);
        success = false;
    end
end

function result = utils_safeFunctionCall(funcHandle, varargin)
    try
        result = funcHandle(varargin{:});
        utils_logger('DEBUG', 'Function call succeeded');
    catch ME
        utils_logger('ERROR', 'Function call failed: %s', ME.message);
        rethrow(ME);
    end
end
