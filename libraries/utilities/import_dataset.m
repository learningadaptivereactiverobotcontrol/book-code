function import_dataset(datasetPath, nSamplePoint)
% Import data recorded on the real robot, and convert them to a format
% suitable for the practical.

if nargin < 2
    nSamplePoint = 200;
end

if nargin == 0
    datasetPath = 'bags';
else
    assert(exist(datasetPath, 'file') == 7, 'Could not find folder')
end

% Import Panda robot wrapper
filepath = fileparts(which('import_dataset.m'));
addpath(genpath(fullfile(filepath, '..', 'matlab_exercises', 'libraries', 'book-robot-simulation')));
pandaRobot = PandaWrapper();


filelist = dir(fullfile(datasetPath, '**', '*.db3'));
nTraj = length(filelist);

trajectories = nan(6, nSamplePoint, nTraj);
for iTraj=1:nTraj
    % Open new bag for new trajectory
    bagDirectory = filelist(iTraj).folder;
    bag = ros2bag(bagDirectory);
    msgs = readMessages(bag);

    msgsStruct = cell2mat(msgs);

    jointPosition = reshape(cell2mat({msgsStruct.position}), 7, []);
    position = pandaRobot.computeForwardKinematics(jointPosition);
    
    jointVelocity = reshape(cell2mat({msgsStruct.velocity}), 7, []);
    velocity = pandaRobot.computeForwardVelocity(jointPosition, jointVelocity);

    time = cellfun(@(x) double(x.header.stamp.sec) + 1e-9*double(x.header.stamp.nanosec) ...
        - 1e-9*double(bag.StartTime), msgs)';

    % Resample trajectories
    trajectories(:, :, iTraj) = [interp1(time, position', linspace(time(1), time(end), nSamplePoint)), ...
                                      interp1(time, velocity', linspace(time(1), time(end), nSamplePoint))]';
end

saveDirectory = fullfile(filepath, '..', 'matlab_exercises', 'practical_1', 'dataset', 'demonstration_dataset.mat');
save(saveDirectory, 'trajectories');
disp("Saved datset to " + saveDirectory);

end