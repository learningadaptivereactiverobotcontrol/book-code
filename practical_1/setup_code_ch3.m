function setup_code_ch3(compileFlag)
% For Windows user: you can run setup_code_ch3(0) (or launch the script as is) to just
% test that the libraries are correctly working on your computer

% If you get an error, or you are using linux or maccOS, you'll need to
% recompile the libraries on your computer by running setup_code_ch3(1) 

if nargin == 0
    compileFlag = 0;
end

% Get folder path
folder_path = fileparts(which('setup_code_ch3.m'));

%% Install libsvm 

% Install toolboxes in ML_toolbox
libsvm_path = fullfile(folder_path, '..', 'libraries', 'book-ml-toolbox', 'methods', 'toolboxes', 'libsvm', 'matlab');
cd(libsvm_path);
if exist("svmtrain") ~= 3 || compileFlag == true
    run('make.m') 
else
    disp('libsvm is correctly installed')
end
cd(folder_path)
disp('Press space to continue to next library'); pause()

%% Install lightspeed
% IF NOT WORKING FOR WINDOWS PC, MEX with a compiler needs to be configured = install compiler
lightspeed_path = fullfile(folder_path, '..', 'libraries', 'book-thirdparty', 'lightspeed');
cd(lightspeed_path);
addpath(genpath(lightspeed_path))

% Install lightspeed if required
if compileFlag == true
    % If error here, mex compiler has to be setup "mex -setup"
    install_lightspeed
end
test_lightspeed
close all;

cd(folder_path)
disp('Press space to continue to next library'); pause()

%% Install sedumi
sedumi_path = fullfile(folder_path, '..', 'libraries', 'book-thirdparty', 'sedumi');
cd(sedumi_path);
addpath(genpath(sedumi_path))

if compileFlag == true
    % If error here run "install_sedumi -build"
    install_sedumi
end
test_sedumi

cd(folder_path);
close all;

disp('All libraries are installed correctly')
end