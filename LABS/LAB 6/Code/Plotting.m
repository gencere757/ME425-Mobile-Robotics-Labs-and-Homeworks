% Script: Plotting.m
% Loads all .mat files, generates readable figures, and saves them as PNG.
% Uses experiment-aware titles based on filename and variable names.

clear; clc;

% Directory to search (current folder). Change '.' if needed.
rootDir = 'C:\Users\Arda Gencer\Desktop\ME 425\LABS\LAB 6\Filtered Data';

% Get all .mat files recursively under rootDir
mat_files = dir(fullfile(rootDir, '**', '*.mat'));

if isempty(mat_files)
    warning('No .mat files found under %s', rootDir);
end

for i = 1:numel(mat_files)
    % Build full path safely
    filename = fullfile(mat_files(i).folder, mat_files(i).name);

    % Extra safety: skip if not a regular file
    if mat_files(i).isdir
        continue;
    end

    % Load the current .mat file
    fprintf('Loading: %s\n', filename);
    data = load(filename);

    % Check if t_values exists
    if ~isfield(data, 't_values')
        warning('File %s does not contain t_values. Skipping...', filename);
        continue;
    end

    t_values = data.t_values;

    % Get all variable names except t_values
    var_names = fieldnames(data);
    var_names(strcmp(var_names, 't_values')) = [];

    % Check if both x_values and z_values exist
    has_x = ismember('x_values', var_names);
    has_z = ismember('z_values', var_names);

    % Prepare figure title and output file base name
    [~, shortName, ~] = fileparts(filename);
    figTitle = buildFigureTitle(shortName);   % helper function below (filename removed)
    pngName  = [shortName '_plots.png'];

    % Create a new figure for this file
    figure('Name', figTitle, 'NumberTitle', 'off', 'Position', [100, 100, 1000, 700]);

    if has_x && has_z
        % Base variables (paired x & z)
        baseVars  = {'x_values','z_values'};
        otherVars = setdiff(var_names, baseVars);

        % Total number of subplots: 1 for x/z + others
        n_total = 1 + numel(otherVars);
        n_cols  = ceil(sqrt(n_total));
        n_rows  = ceil(n_total / n_cols);

        % --- Subplot 1: measurement vs estimate of y ---
        subplot(n_rows, n_cols, 1);
        plot(t_values, data.x_values, 'b-', 'DisplayName', 'Estimate x̂');
        hold on;
        plot(t_values, data.z_values, 'r-', 'DisplayName', 'Measurement z');
        hold off;

        xlabel('Time t (s)');
        ylabel('Lateral distance y (m)');
        title('Measurement vs estimate of y');
        legend('Location', 'best');
        grid on;

        % --- Remaining subplots for other variables ---
        for j = 1:numel(otherVars)
            subplot(n_rows, n_cols, j+1);
            vname = otherVars{j};
            y = data.(vname);
            plot(t_values, y);

            xlabel('Time t (s)');
            [ylab, ttl] = prettyLabelForVar(vname);  % helper below
            ylabel(ylab);
            title(ttl);
            grid on;
        end

        sgtitle(figTitle, 'FontSize', 14, 'FontWeight', 'bold');

    else
        % No paired x/z: every variable gets its own subplot
        n_vars = numel(var_names);

        if n_vars == 0
            warning('File %s only contains t_values. No data to plot.', filename);
            close(gcf);
            continue;
        end

        n_cols = ceil(sqrt(n_vars));
        n_rows = ceil(n_vars / n_cols);

        for j = 1:n_vars
            subplot(n_rows, n_cols, j);
            vname = var_names{j};
            y = data.(vname);
            plot(t_values, y);

            xlabel('Time t (s)');
            [ylab, ttl] = prettyLabelForVar(vname);
            ylabel(ylab);
            title(ttl);
            grid on;
        end

        sgtitle(figTitle, 'FontSize', 14, 'FontWeight', 'bold');
    end

    % Save the figure as PNG
    saveas(gcf, pngName);
    fprintf('Saved: %s\n', pngName);
end

fprintf('\nAll plots saved successfully with clean titles!\n');

%% -------- Helper Functions -------------------------------------------

function figTitle = buildFigureTitle(shortName)
% Build a human-readable figure title from the .mat file name (no filename included)

    lowerName = lower(shortName);

    % --- Task type ---
    if contains(lowerName, 'task1')
        taskStr = 'Task 1 - 1D Lateral Estimation';
    elseif contains(lowerName, 'task2')
        taskStr = 'Task 2 - 2D Pose Estimation';
    else
        taskStr = 'Kalman Filter Experiment';
    end

    % --- Initial heading (angled vs straight/parallel) ---
    if contains(lowerName, 'angled')
        headingStr = 'Robot starts angled to wall';
    else
        headingStr = 'Robot starts parallel to wall';
    end

    % --- Trust scenario (model vs sensor vs equal) ---
    if contains(lowerName, 'sensorreliance') || ...
       (contains(lowerName, 'sensor') && ~contains(lowerName, 'model'))
        trustStr = 'Sensor-reliant (R << Q)';
    elseif contains(lowerName, 'modelreliance') || contains(lowerName, 'model')
        trustStr = 'Model-reliant (Q << R)';
    elseif contains(lowerName, 'equal')
        trustStr = 'Equal trust (Q = R)';
    else
        trustStr = '';
    end

    % Combine strings without the .mat filename
    if isempty(trustStr)
        figTitle = sprintf('%s - %s', taskStr, headingStr);
    else
        figTitle = sprintf('%s - %s - %s', taskStr, headingStr, trustStr);
    end
end

function [ylab, ttl] = prettyLabelForVar(vname)
% Map raw variable names to nice y-axis labels and subplot titles

    switch vname
        case 'x_values'
            ylab = 'Estimated lateral distance y (m)';
            ttl  = 'Estimated lateral distance y';

        case 'z_values'
            ylab = 'Measured lateral distance y (m)';
            ttl  = 'Measured lateral distance y';

        case 'K_values'
            ylab = 'Kalman gain K';
            ttl  = 'Kalman gain evolution';

        case 'P_values'
            ylab = 'Error covariance P';
            ttl  = 'Error covariance evolution';

        case 'P11_values'
            ylab = 'Variance of y: P_{11}';
            ttl  = 'Variance of lateral distance P_{11}';

        case 'P22_values'
            ylab = 'Variance of \theta: P_{22}';
            ttl  = 'Variance of heading P_{22}';

        case 'theta_est_values'
            ylab = 'Estimated heading theta (rad)';
            ttl  = 'Estimated heading theta';

        case 'y_est_values'
            ylab = 'Estimated lateral distance y (m)';
            ttl  = 'Estimated lateral distance y';

        otherwise
            % Fallback: just prettify the name
            ylab = strrep(vname, '_', '\_');
            ttl  = strrep(vname, '_', '\_');
    end
end