

% Folder containing your .mat files
folderPath = 'C:\Users\Arda Gencer\Desktop\ME 425\LABS\LAB 6\2D Data';   % <-- change this

% Variables you want to keep
varsToKeep = {'t_values', 'P11_values', 'P22_values', 'K_values', 'theta_est_values', 'y_est_values'};

% Get list of all .mat files in the folder
files = dir(fullfile(folderPath, '*.mat'));

for k = 1:numel(files)
    inFile = fullfile(folderPath, files(k).name);

    % Optional: skip files that are already filtered
    if startsWith(files(k).name, 'filtered_')
        continue
    end

    % Load into a struct so we don't pollute the workspace
    S = load(inFile);

    % Build filtered struct with only desired variables (if they exist)
    filtered = struct();
    for v = 1:numel(varsToKeep)
        name = varsToKeep{v};
        if isfield(S, name)
            filtered.(name) = S.(name);
        else
            warning('Variable "%s" not found in file "%s".', name, files(k).name);
        end
    end

    % Create output filename with "filtered_" prefix
    [~, baseName, ~] = fileparts(files(k).name);
    outFile = fullfile(folderPath, ['filtered_' baseName '.mat']);

    % Save filtered variables
    save(outFile, '-struct', 'filtered');
end