function setup_code_ch3(testFlag)
% For Windows user: you can run setup_code_ch3(1) to only test that the 
% libraries are correctly working on your computer. If you get an error, 
% you can recompile the libraries on your computer by running setup_code_ch3
%
% For MacOs and Linux, you'll need to recompile the libraries on your computer by running setup_code_ch3
% Further instructions for Mac users below. 
% These are also available in the READ_ME in the folder 'mac_setup'.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                  WARNING FOR MAC USERS
% If you are using a Mac, you have some complements that you need to install 
% 
% First, follow this url : https://developer.apple.com/download/all/?q=command%20line%20tools
% Then log in to Apple Developper (create an account if needed) and download :
% 'Command Line Tools for Xcode 14.2'
% 
% Once downloaded, open the package and follow the installation process.
% 
% Then, you have to open a terminal window at the folder matlab_exercices/lecture4-learning-control-laws. 
% For that you have to open your finder, go to the folder matlab_exercices/lecture4-learning-control-laws
% Then, right click and choose: "New terminal to folder"
% Here a terminal window will open. If you run pwd, you should see  ../matlab_exercices/lecture4-learning-control-laws. 
% 
% We use SUDO command line, that means you‘ll have to enter you admin password. 
% 
% Then run the following command in this terminal:
% bash Mac_config.sh
%
% You will then need to run 'ch3_ex4_lpvDS.m' once to finalize the setup.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin == 0
    compileFlag = 1;
    testFlag = 1;
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
if testFlag == true
    test_sedumi
end
close all;

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
if testFlag == true
    test_lightspeed
end

cd(folder_path);
close all;

disp('All libraries are installed correctly')
%%
if ismac
    system("sudo spctl --master-enable")
end

end
