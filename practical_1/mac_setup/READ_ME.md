# Instructions to set up lpvDS learning on Mac 

Follow these instructions to set up proper compilation of the required librairies on Mac. You will then need to run 'setup_code_ch3(1)' in Matlab.

1. First, download Xcode from the App Store. Then, open it once and accept the license

2. Once downloaded, open the package and follow the installation process.
 
3. Then, you have to open a terminal window at the folder matlab_exercices/lecture4-learning-control-laws. 
For that you have to open your finder, go to the folder matlab_exercices/lecture4-learning-control-laws
Then, right click and choose: "New terminal to folder"
Here a terminal window will open. If you run pwd, you should see  ../matlab_exercices/lecture4-learning-control-laws. 
 
We use SUDO command line, which means you‘ll have to enter you admin password. 

4. Then run the following command in this terminal:
bash Mac_config.sh

5. To finalize the setup, 'MPC_LPVDS.m' should be run once with 'est_options.type = 0'.


If you encounter any issue with this installation, please contact tristan.bonato@epfl.ch 
