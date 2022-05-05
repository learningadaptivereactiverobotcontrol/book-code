function varargout = gui_lpvDS(varargin)
%GUI_LPVDS MATLAB code file for gui_lpvDS.fig
%      GUI_LPVDS, by itself, creates a new GUI_LPVDS or raises the existing
%      singleton*.
%
%      H = GUI_LPVDS returns the handle to a new GUI_LPVDS or the handle to
%      the existing singleton*.
%
%      GUI_LPVDS('Property','Value',...) creates a new GUI_LPVDS using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to gui_lpvDS_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      GUI_LPVDS('CALLBACK') and GUI_LPVDS('CALLBACK',hObject,...) call the
%      local function named CALLBACK in GUI_LPVDS.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES
% Edit the above text to modify the response to help gui_lpvDS

% Last Modified by GUIDE v2.5 22-Oct-2019 09:34:37

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
clc;
% nargin=0;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_lpvDS_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_lpvDS_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
   gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before gui_lpvDS is made visible.
function gui_lpvDS_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for gui_lpvDS
handles.output = hObject;
handles.Damping=varargin{1};
% set up a simple robot and a figure that plots it
robot = create_simple_robot();
hObject = initialize_robot_figure(robot, hObject);
title('Feasible Robot Workspace','Interpreter','LaTex')
% Base Offset
base = [-1 1]';
% Axis limits
limits = [-2.5 0.5 -0.45 1.25];

% Default Values
handles.ds_type   = 'seds';
handles.opt_type  = '(O1)-QLF';
handles.obj_type  = 'MSE';
handles.gmm_type  = 'Manual';
handles.init_type = 'Manual';
handles.limits    = limits;
handles.robot     = robot;
handles.base      = base;
handles.num_gaussians=2;

% Update handles structure
guidata(hObject, handles);

% Draw Reference Trajectories
collect_data(hObject, handles);

% UIWAIT makes gui_lpvDS wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = gui_lpvDS_OutputFcn(~, ~, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function figure1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(~, ~, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1


% --- Executes on button press in learn_DS_lpv.
function learn_DS_lpv_Callback(hObject, eventdata, handles)
% hObject    handle to learn_DS_lpv (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

num_gauss=handles.num_gaussians;
est_type=1;
% Visualize Cluster Parameters on Manifold Data
if isfield(handles, 'h_gmm')
    h_gmm = handles.h_gmm;
    h_ctr = handles.h_ctr;
    delete(h_gmm)
    delete(h_ctr)
end

Data = handles.Data;
N  = size(Data,2);
if N < 500
    sample = 1;
else 
    sample = 2;
end
est_options = [];
est_options.type        = est_type;   % GMM Estimation Alorithm Type    
est_options.maxK        = 15;  % Maximum Gaussians for Type 1
est_options.fixed_K     = num_gauss;  % Fix K and estimate with EM for Type 1
est_options.samplerIter = 40;  % Maximum Sampler Iterations
                                    % For type 0: 20-50 iter is sufficient
                                    % For type 2: >100 iter are needed                                    
est_options.do_plots      = 0;   % Plot Estimation Statistics
est_options.sub_sample    = sample;   % Size of sub-sampling of trajectories
est_options.estimate_l    = 1;   % 0/1 Estimate the lengthscale, if set to 1
est_options.l_sensitivity = 5;  
est_options.length_scale  = [];  

[Priors, Mu, Sigma] =  fit_gmm(Data(1:2,:), Data(3:4,:), est_options);
est_K      = length(Priors) ;
[~, est_labels] =  my_gmm_cluster(Data(1:2,:), Priors, Mu, Sigma, 'hard', []);
fprintf ('****** Optimal number of Gaussians K=%d ******\n', est_K);

%%% Visualize GMM pdf from learnt parameters
clear ds_gmm; ds_gmm.Mu = Mu; ds_gmm.Sigma = Sigma; 
ds_gmm.Priors = Priors; 

% Adjust Covariance Matrices
tot_dilation_factor = 1; rel_dilation_fact = 0.2;
Sigma_ = adjust_Covariances(ds_gmm.Priors, ds_gmm.Sigma, tot_dilation_factor, rel_dilation_fact);
ds_gmm.Sigma = Sigma_;

[~, h_gmm, h_ctr] =  plotGMMParameters( Data(1:2,:), est_labels, ds_gmm.Mu, ds_gmm.Sigma, hObject);

handles.ds_type = 'lpv';
handles.h_gmm = h_gmm;
handles.h_ctr = h_ctr;
guidata(hObject, handles);
handles.ds_gmm=ds_gmm;





% Extract Data from Handles
limits   = handles.limits ;
att_g    = handles.att_g ;
M        = size(Data,1);
constr_type = 0;
fprintf ('Constraint from QLF:\n A_k^T + A_k < 0 \n');
init_cvx = 0;
P_opt = eye(M);
%%%%%%%%  LPV system sum_{k=1}^{K}\gamma_k(xi)(A_kxi + b_k) %%%%%%%%  
[A_k, b_k, ~] = optimize_lpv_ds_from_data(Data, att_g, constr_type, ds_gmm, P_opt, init_cvx);
ds_lpv = @(x) lpv_ds(x, ds_gmm, A_k, b_k);

%%%%%%%%%%%%%%    Plot Resulting DS  %%%%%%%%%%%%%%%%%%%
% Create DS function handle
if isfield(handles, 'h_ds')
    h_ds = handles.h_ds;
    delete(h_ds)
end

h_ds = plot_ds_model(hObject, ds_lpv, [0 0]', limits,'medium');
handles.h_ds    = h_ds;
handles.ds_fun  = ds_lpv;
handles.A_g     = A_k;
guidata(hObject, handles);


function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double

% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in checkbox3.
function checkbox3_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox3
% if get(hObject,'Value')
%     est_type = 1   
% end

% --- Executes on selection change in optimization_options.
function optimization_options_Callback(hObject, eventdata, handles)
% hObject    handle to optimization_options (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns optimization_options contents as cell array
%        contents{get(hObject,'Value')} returns selected item from optimization_options
contents = cellstr(get(hObject,'String'));
handles.opt_type = contents{get(hObject,'Value')};
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function optimization_options_CreateFcn(hObject, eventdata, handles)
% hObject    handle to optimization_options (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Run_SEDS.
function Run_SEDS_Callback(hObject, eventdata, handles)
% hObject    handle to Run_SEDS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

Data_sh = handles.Data_sh;
ds_gmm  = handles.ds_gmm;
att_g   = handles.att_g;
limits  = handles.limits;
num_K_seds = handles.num_K_seds;

clear options;
switch handles.obj_type
    case 'MSE'
        options.objective = 'mse';         
    case 'Likelihood'
        options.objective = 'likelihood' ; 
end  
options.tol_mat_bias = 10^-6; % A very small positive scalar to avoid
                              % instabilities in Gaussian kernel [default: 10^-1]                             
options.display = 1;          % An option to control whether the algorithm
                              % displays the output of each iterations [default: true]                            
options.tol_stopping=10^-9;   % A small positive scalar defining the stoppping
                              % tolerance for the optimization solver [default: 10^-10]
options.max_iter = 500;       % Maximum number of iteration forthe solver [default: i_max=1000]


ds_gmm.Priors
ds_gmm.Mu
ds_gmm.Sigma

[Priors, Mu, Sigma]= SEDS_Solver(ds_gmm.Priors, ds_gmm.Mu, ds_gmm.Sigma, Data_sh,options); %running SEDS optimization solver
clear ds_seds
ds_seds = @(x) GMR_SEDS(Priors,Mu,Sigma,x,1:2,3:4);

%%%%%%%%%%%%%%    Plot Resulting DS  %%%%%%%%%%%%%%%%%%%
% Create DS function handle
if isfield(handles, 'h_ds')
    h_ds = handles.h_ds;
    delete(h_ds)
end

h_ds = plot_ds_model(hObject, ds_seds, att_g, limits,'medium');
handles.h_ds    = h_ds;
handles.ds_fun  = ds_seds;
handles.ds_gmm  = ds_gmm; 
handles.num_K_seds = num_K_seds;
guidata(hObject, handles);


function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1


% --- Executes on selection change in listbox2.
function listbox2_Callback(hObject, eventdata, handles)
% hObject    handle to listbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox2
contents = cellstr(get(hObject,'String'));
handles.obj_type = contents{get(hObject,'Value')}
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function listbox2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes during object creation, after setting all properties.
function num_gauss_CreateFcn(hObject, eventdata, handles)
% hObject    handle to num_gauss (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox4.
function checkbox4_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox4
% if get(hObject,'Value')
%     est_type = 0   
% end

% --- Executes on button press in checkbox6.
function checkbox6_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox6
% if get(hObject,'Value')
%     est_type = 1
%     num_gmms = []
% end


% --- Executes on button press in pushbutton7.
% --- GMM initialization for SEDS --- %
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

Data_sh = handles.Data_sh;
if isfield(handles,'num_K_seds')
    num_K_seds = handles.num_K_seds;
end
sample = 1;
switch handles.init_type
    case 'Manual'        
        %finding an initial guess for GMM's parameter
        if (num_K_seds > 15)
            warning ('!!!!!! Maximum Number of Gaussians K=15, setting to K=15 !!!!!!');
            num_K_seds = 15;
        end
            
        [Priors_0, Mu_0, Sigma_0] = initialize_SEDS(Data_sh(:,1:sample:end),num_K_seds);
                      
    case 'Model Selection'
        est_options = [];
        est_options.type        = 1;   % GMM Estimation Alorithm Type
        est_options.maxK        = 15;  % Maximum Gaussians for Type 1/2
        est_options.do_plots    = 0;   % Plot Estimation Statistics
        est_options.fixed_K     = [];   % Fix K and estimate with EM
        est_options.sub_sample  = 1;   % Size of sub-sampling of trajectories
        
        [Priors0, ~, ~] = fit_gmm(Data_sh(:,1:sample:end), [], est_options);
        nb_gaussians = length(Priors0);
        fprintf ('Optimal K=%d with EM-Model Selection',nb_gaussians);
        %finding an initial guess for GMM's parameter
        [Priors_0, Mu_0, Sigma_0] = initialize_SEDS(Data_sh(:,1:sample:end),nb_gaussians);
        num_K_seds = nb_gaussians;
end

if isfield(handles, 'h_gmm')
    h_gmm = handles.h_gmm;
    h_ctr = handles.h_ctr;
    delete(h_gmm)
    delete(h_ctr)
end

[~, est_labels]   =  my_gmm_cluster(Data_sh(1:2,:), Priors_0, Mu_0(1:2,:), Sigma_0(1:2,1:2,:), 'hard', []);
[~, h_gmm, h_ctr] =  plotGMMParameters( Data_sh(1:2,:), est_labels, Mu_0(1:2,:) + repmat(handles.att_g,[1 size(Mu_0,2)]), Sigma_0(1:2,1:2,:), hObject);

clear ds_gmm
ds_gmm.Priors = Priors_0;
ds_gmm.Mu = Mu_0;
ds_gmm.Sigma = Sigma_0;

handles.ds_type = 'seds';
handles.num_K_seds = num_K_seds;
handles.ds_gmm = ds_gmm;
handles.h_gmm = h_gmm;
handles.h_ctr = h_ctr;
guidata(hObject, handles);


% --- Executes on button press in checkbox5.
function checkbox5_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox5


% --- Executes on selection change in gmm_fit.
function gmm_fit_Callback(hObject, eventdata, handles)
% hObject    handle to gmm_fit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns gmm_fit contents as cell array
%        contents{get(hObject,'Value')} returns selected item from gmm_fit
contents = cellstr(get(hObject,'String'));
handles.gmm_type = contents{get(hObject,'Value')};
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function gmm_fit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gmm_fit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double
handles.num_K_seds = str2double(get(hObject,'String'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in SEDS_init.
function SEDS_init_Callback(hObject, eventdata, handles)
% hObject    handle to SEDS_init (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns SEDS_init contents as cell array
%        contents{get(hObject,'Value')} returns selected item from SEDS_init
contents = cellstr(get(hObject,'String'));
handles.init_type = contents{get(hObject,'Value')};
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function SEDS_init_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SEDS_init (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in simulate.
function simulate_Callback(hObject, eventdata, handles)
% hObject    handle to simulate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% extract data
ds_gmm = handles.ds_gmm;
robot  = handles.robot;
base   = handles.base;
att_g  = handles.att_g;

% run simulation
struct_stiff = [];

switch handles.ds_type
    case 'seds'
        A_g = [];
        struct_stiff.DS_type = 'seds'; % Options: SEDS, Global-LPV, LAGS, LMDS ?        
        ds_seds = handles.ds_fun;
        ds_fun = @(x)(ds_seds(x-handles.att_g));
        
    case 'lpv'        
        ds_fun = handles.ds_fun;
        A_g    = handles.A_g;
        struct_stiff.DS_type = 'global'; % Options: SEDS, Global-LPV, LAGS, LMDS ?
end
struct_stiff.gmm = ds_gmm;
struct_stiff.A_g = A_g;
struct_stiff.basis = 'D'; % Options: D (using the same basis as D) or I (using the world basis)

dt = 0.01;

% simulate_passiveDS_GUI(handles.figure1, robot, base, ds_fun, att_g, dt, varargin{1});
% To visualize damping/stiffness matrices
simulate_passiveDS_GUI(handles.figure1, robot, base, ds_fun, att_g, dt, handles.Damping, struct_stiff );


% --- Executes on button press in Clear_All.
function Clear_All_Callback(hObject, eventdata, handles)
% hObject    handle to Clear_All (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
contents = cellstr(get(hObject,'String'));
handles.clear_all = contents{get(hObject,'Value')};
if isfield(handles, 'h_ds')
    h_ds = handles.h_ds;
    delete(h_ds)
end
if isfield(handles, 'h_gmm')
    h_gmm = handles.h_gmm;
    delete(h_gmm)
end
if isfield(handles, 'h_ctr')
    h_ctr = handles.h_ctr;
    delete(h_ctr)
end
if isfield(handles, 'h_data')
    h_data = handles.h_data;
    delete(h_data)
end
if isfield(handles, 'h_att')
    h_att = handles.h_att;
    delete(h_att)
end
if isfield(handles, 'h_vel')
    h_vel = handles.h_vel;
    delete(h_vel)
end
guidata(hObject, handles);

% Draw Reference Trajectories
collect_data(hObject, handles);

% -- Internal function to collect demonstrations -- %
function collect_data(hObject, handles)
[data, hp] = draw_mouse_data_on_DS(handles.figure1, handles.limits);
Data = []; Data_sh = []; x0_all = []; x0_end = [];
for l=1:length(data)    
    % Check where demos end and shift
    data_ = data{l};
    x0_end = [x0_end data_(1:2,end)];
    Data = [Data data_];
    x0_all = [x0_all data_(1:2,1)];    
    
    
    % Shift data to origin
    data_(1:2,:) = data_(1:2,:) - repmat(data_(1:2,end), [1 length(data_)]);
    data_(3:4,end) = zeros(2,1);

    Data_sh = [Data_sh data_];
end

% Position/Velocity Trajectories
Xi_ref     = Data(1:2,:);
Xi_dot_ref = Data(3:end,:);

% Global Attractor of DS
att_g = mean(x0_end,2);

% Position/Velocity Trajectories
delete(hp)
[h_data, h_att, h_vel] = plot_reference_trajectories_DS_GUI(Data, att_g, 10, 0.5);
grid on;
box on;
title('Demonstrated Trajectories','Interpreter','LaTex','FontSize',20);
xlabel('$x_1$','Interpreter','LaTex','FontSize',20);
ylabel('$x_2$','Interpreter','LaTex','FontSize',20);

handles.Data      = Data;
handles.Data_sh   = Data_sh;
handles.att_g     = att_g;
handles.h_data    = h_data;
handles.h_att     = h_att;
handles.h_vel     = h_vel;

% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function uipanel6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to uipanel6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.num_gaussians=floor(get(hObject,'Value'));

% Hard set minimum K to 2, must be a better way to do this
if handles.num_gaussians < 2
    handles.num_gaussians=2;
end
guidata(hObject, handles);
fprintf("Number of Gaussians is %i\n", floor(handles.num_gaussians));


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
%set(hObject, 'Min', 2);

% --- Executes when entered data in editable cell(s) in uitable2.
function uitable2_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to uitable2 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
