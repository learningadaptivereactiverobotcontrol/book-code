function setup_code_ch3(compileFlag)
% For Windows user: you can run setup_code_ch3(0) (or launch the script as is) to just
% test that the libraries are correctly working on your computer. If you get an error, 
% you can recompile the libraries on your computer by running setup_code_ch3(1).
%
% For MacOs and Linux, you'll need to recompile the libraries on your computer by running setup_code_ch3(1). 
% Further isntructions for Mac users below
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                  WARNING FOR MAC USERS
% If you are using a Mac, you have some complements that you need to install.
%
% This should be done during the lecture4 exercises. 
% If you SKIPPED those, go to the 'lecture4-learning-control-laws' directory and 
% follow the READ_ME instructions in the 'mac_setup' directory.
%
% You will then need to run 'ch3_ex4_lpvDS.m' once to finalize the setup.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin == 0
    compileFlag = 0;
end

% Get folder path
folder_path = fileparts(which('setup_code_ch3.m'));
%% Give acces to Lasa developer if you are on mac
if ismac
    system("sudo spctl --master-disable")
end

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

%% Install sedumi
sedumi_path = fullfile(folder_path, '..', 'libraries', 'book-thirdparty', 'sedumi');
cd(sedumi_path);
addpath(genpath(sedumi_path))

if compileFlag == true
    % If error here run "install_sedumi -build"
    install_sedumi
end
test_sedumi

cd(folder_path)
disp('Press space to continue to next library'); pause()

%% Install lightspeed
% IF NOT WORKING FOR WINDOWS PC, MEX with a compiler needs to be configured = install compiler
lightspeed_path = fullfile(folder_path, '..', 'libraries', 'book-thirdparty', 'lightspeed');
cd(lightspeed_path);
addpath(genpath(lightspeed_path))

% Install lightspeed if required
if compileFlag == true || ismac
    % If error here, mex compiler has to be setup "mex -setup"
    install_lightspeed
end
test_lightspeed
close all;

cd(folder_path);
close all;

disp('All libraries are installed correctly')
%%
if ismac
    system("sudo spctl --master-enable")
end

end
