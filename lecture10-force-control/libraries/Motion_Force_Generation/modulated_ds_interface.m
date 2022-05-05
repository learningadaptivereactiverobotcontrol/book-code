function varargout = modulated_ds_interface(varargin)
% MODULATED_DS_INTERFACE MATLAB code for modulated_ds_interface.fig
%      MODULATED_DS_INTERFACE, by itself, creates a new MODULATED_DS_INTERFACE or raises the existing
%      singleton*.
%
%      H = MODULATED_DS_INTERFACE returns the handle to a new MODULATED_DS_INTERFACE or the handle to
%      the existing singleton*.
%
%      MODULATED_DS_INTERFACE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MODULATED_DS_INTERFACE.M with the given input arguments.
%
%      MODULATED_DS_INTERFACE('Property','Value',...) creates a new MODULATED_DS_INTERFACE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before modulated_ds_interface_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to modulated_ds_interface_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help modulated_ds_interface

% Last Modified by GUIDE v2.5 29-May-2018 17:53:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @modulated_ds_interface_OpeningFcn, ...
                   'gui_OutputFcn',  @modulated_ds_interface_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
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


% --- Executes just before modulated_ds_interface is made visible.
function modulated_ds_interface_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to modulated_ds_interface (see VARARGIN)

% Choose default command line output for modulated_ds_interface\if
    handles.output = hObject;
    if(length(varargin)==0)
        handles.perturbation = true;
        handles.makeAMovie = false;
        handles.loadModel = false;
    elseif(length(varargin)==1)
        handles.perturbation = varargin{1};
        handles.makeAMovie = false;
        handles.loadModel = false;
    elseif(length(varargin)==2)
        handles.perturbation = varargin{1};
        handles.makeAMovie = varargin{2};
        handles.loadModel = false;
    elseif(length(varargin)==3)
        handles.perturbation = varargin{1};
        handles.makeAMovie = varargin{2};
        handles.loadModel = varargin{3};
    end
    handles.color =  [217 83 25
                      119 172 48
                      0 114 189
                      126 47 142]/255;
    handles.modulation = true;
    handles.map = gray;
    handles.map2 = handles.map(30:end,:);
    handles.map2(1:11,:) = repmat(handles.map(30,:),11,1);
    handles.axisLimits = [-1 1 0 1];
    handles.stop = false;
    handles.updateSurface = true;
    handles.startSimulation = false;
%     set(gcf,'CloseRequestFcn',@exit_Callback);
    iptPointerManager(gcf, 'enable');
    iptSetPointerBehavior(gca, @(gcf, currentPoint)set(gcf, 'Pointer', 'crosshair'));

    if(handles.loadModel)
        model = load('model.mat','svmgrad','Xm','Ym','Tau','Xs');
        handles.svmgrad = model.svmgrad;
        handles.Xm = model.Xm;
        handles.Ym = model.Ym;
        handles.Tau = model.Tau;
        handles.Xs = model.Xs;
        handles.startSimulation = true;
        handles.updateSurface = false;
        guidata(hObject, handles);
        set(handles.pushbutton2,'enable','off');
        startSimulation(hObject,handles);
        handles = guidata(hObject);
        handles.startSimulation = false;
        guidata(hObject,handles); 
        if(handles.updateSurface)
            updateSurface(hObject,handles);
            handles = guidata(hObject);
        end
    else
        updateSurface(hObject,handles);
        handles = guidata(hObject);
        handles.updateSurface = false;
    end
    % Update handles structure
    guidata(hObject, handles);
     
  
%     if(handles.stop)
%         close all;
%     end


% UIWAIT makes modulated_ds_interface wait for user response (see UIRESUME)
% uiwait(handles.figure1);
% function exit_Callback(hObject,varargin)
%     handles = guidata(hObject);
%     handles.stop = true;
%     guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = modulated_ds_interface_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
handles = guidata(hObject);
varargout{1} = handles.output;
if(handles.stop)
    close all;
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles = guidata(gcbo);
    handles.updateSurface = true;
    guidata(hObject,handles);
%     handles.startSimulation = false;
    if(handles.startSimulation == false)
        guidata(hObject, handles);
        updateSurface(hObject,handles);
        handles = guidata(hObject);
        guidata(gcbo,handles); 
    end
    
function updateSurface(hObject,handles)
    guidata(hObject, handles);
    set(handles.pushbutton1,'enable','off');
    set(handles.pushbutton2,'enable','off');
    cleanPlot(hObject,handles);
    mouseData = get_mouse_curve(gcf,hObject,handles);
    handles = guidata(hObject);
    handles.mouseData = mouseData;
    guidata(hObject,handles);
    if(~handles.stop)
        set(handles.pushbutton3,'enable','off');
        set(handles.axes1.Title,'String','Compute surface model ...','Interpreter','latex','FontSize',15);
        axes(handles.axes1);
        smoothMouseData(hObject,handles);
        handles = guidata(hObject);
        buildDataset(hObject,handles);
        handles = guidata(hObject);
        computeModel(hObject,handles);
        handles = guidata(hObject);
        evaluateModel(hObject,handles);
        handles = guidata(hObject);
        plotLearnedSurface(hObject,handles);
        handles = guidata(hObject);
        set(handles.pushbutton3,'enable','on'); 
    end
    set(handles.pushbutton1,'enable','on');
    set(handles.pushbutton2,'enable','on');
    handles.updateSurface = false;
    guidata(hObject, handles);   
    
function cleanPlot(hObject,handles)

    legend(gca,'off');
    colorbar('off');  
    cla(gcf);
    plot([-1 1],[0.4 0.4],'LineWidth',4,'LineStyle','--','Color','k');
    grid on;
    axis equal;
    axis(handles.axisLimits);
    xlabel('$x$ [$m$]','Interpreter','latex','FontSize',15);
    ylabel('$y$ [$m$]','Interpreter','latex','FontSize',15);
    xticks(handles.axisLimits(1):0.2:handles.axisLimits(2));
    yticks(handles.axisLimits(3):0.2:handles.axisLimits(4));
    title('Draw the surface below the dashed line using the mouse','Interpreter','latex','FontSize',15);
    guidata(hObject, handles);
    
function smoothMouseData(hObject,handles)
   
    x = handles.mouseData(:,1);
    y = handles.mouseData(:,2);
    t=(0:size(x,1)-1)';
    tq = linspace(0,size(x,1)-1,1000)';
    xq = interp1(t,x,tq,'PCHIP');
    yq = interp1(t,y,tq,'PCHIP');
    yf =  smooth(xq,yq);
    handles.Xs = [xq yf];
    guidata(hObject, handles);

function buildDataset(hObject,handles)

    nbData = 4000;
    handles.X = zeros(nbData,2);
    offset = 1;
    
    handles.X(:,1) = min(handles.Xs(:,1))+(max(handles.Xs(:,1))-min(handles.Xs(:,1)))*rand(nbData,1);
    handles.X(:,2) = (offset+max(handles.Xs(:,2)))*rand(nbData,1);
    handles.tau = zeros(length(handles.X),1);
    for k = 1:length(handles.X)
        [val,id] = min(vecnorm(handles.X(k,:)'-handles.Xs')');
        if(handles.X(k,2)-handles.Xs(id,2)<0)
            handles.tau(k) = -val;
        else
            handles.tau(k) = val;
        end
    end
    handles.XFull = [handles.X;handles.Xs];
    handles.tauFull = [handles.tau;zeros(length(handles.Xs),1)];
    guidata(hObject, handles);
    
function computeModel(hObject,handles)

    svr_options.svr_type    = 0;    % 0: epsilon-SVR, 1: nu-SVR
    svr_options.C           = 100;   % set the parameter C of C-SVC, epsilon-SVR, and nu-SVR 
    svr_options.epsilon     = 0.01;  % set the epsilon in loss function of epsilon-SVR 
    % Kernel OPTIONS
    svr_options.kernel_type = 2;    % 0: linear: u'*v, 1: polynomial: (gamma*u'*v + coef0)^degree, 2: radial basis function: exp(-gamma*|u-v|^2)
    svr_options.sigma       = 0.2;  %  radial basis function: exp(-gamma*|u-v|^2), gamma = 1/(2*sigma^2)
    [~, model] = svm_regressor(handles.XFull,handles.tauFull,svr_options,[]);
    handles.svmgrad = [];
    handles.svmgrad.D       = size(handles.X,2);
    handles.svmgrad.nSV     = model.totalSV;
    handles.svmgrad.b       = -model.rho;
    handles.svmgrad.sigma   = svr_options.sigma;
    handles.svmgrad.yalphas = model.sv_coef';
    handles.svmgrad.SVs     = full(model.SVs)';
    guidata(hObject, handles);

function evaluateModel(hObject,handles)
        
    [handles.Xm,handles.Ym] = meshgrid(handles.axisLimits(1):0.01:handles.axisLimits(2),...
                   handles.axisLimits(3):0.01:handles.axisLimits(4));
    Xmt = handles.Xm(:);
    Ymt = handles.Ym(:);
    Taut = zeros(size(Xmt));
    for k = 1:length(Xmt)
        Taut(k) = calculateGamma(handles.svmgrad,[Xmt(k);Ymt(k)]);
    end
    handles.Tau = reshape(Taut,size(handles.Xm));
    guidata(hObject, handles);

function plotLearnedSurface(hObject,handles)
        
    cla(handles.axes1);
    levels = 0:0.1:1;
    levels = [-0.4 levels];
    contourf(handles.Xm,handles.Ym,handles.Tau,levels);
    hold on;
    scatter(handles.Xs(:,1),handles.Xs(:,2));
    colormap(handles.map(30:end,:));
    c = colorbar('Fontsize',15,'Xtick',0:0.2:1);
    c.Label.String = '$\Gamma$(\boldmath$x$) [\unboldmath$m$]';
    c.Label.Interpreter = 'latex';
    cr = contour(handles.Xm,handles.Ym,handles.Tau,[0 0],'LineWidth',2,'LineColor','k');
    cl = clabel(cr,'Interpreter','latex','FontSize',15);
    set(cl(1),'Marker','none');
    set(cl(2),'String','$\Gamma$(\boldmath$x$) = 0');
    [~,id] = min(handles.Xs(:,1));
    set(cl(2),'Position',[-0.95,handles.Xs(id,2)-0.05,0]);
    hold off;
    axis equal;
    axis(handles.axisLimits);
    xticks(handles.axisLimits(1):0.2:handles.axisLimits(2));
    yticks(handles.axisLimits(3):0.2:handles.axisLimits(4));
    xlabel('$x$ [$m$]','Interpreter','latex','FontSize',15);
    ylabel('$y$ [$m$]','Interpreter','latex','FontSize',15);
    title('Learned surface model','Interpreter','latex','FontSize',15);
    grid on;
    legend({'Surface levels','User data'},'Location', 'NorthEast','location',...
            'NE','interpreter','latex','FontSize',15);
    guidata(hObject, handles);


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles = guidata(gcbo);
    handles.startSimulation = true;
    handles.updateSurface = false;
    guidata(hObject, handles);
    set(handles.pushbutton2,'enable','off');
    startSimulation(hObject,handles);
    handles = guidata(hObject);
    handles.startSimulation = false;
    guidata(gcbo,handles); 
    if(handles.stop)
        close all;
    elseif(handles.updateSurface)
        updateSurface(hObject,handles);
    end
    
function startSimulation(hObject,handles)
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Initialize static plot %
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    cla(handles.axes1);
    levels = 0:0.1:1;
    levels = [-0.4 levels];
    [~,h(1)] = contourf(handles.Xm,handles.Ym,handles.Tau,levels);
    hold on;
    text(0.1,0.95,'Robot','Interpreter','latex','FontSize',15);
    colormap(handles.map2);
    c = colorbar('Fontsize',15,'Xtick',0:0.2:1);
    c.Label.String = '$\Gamma$(\boldmath$x$) [\unboldmath$m$]';
    c.Label.Interpreter = 'latex';
    cr = contour(handles.Xm,handles.Ym,handles.Tau,[0 0],'LineWidth',2,'LineColor','k');
    cl = clabel(cr,'Interpreter','latex','FontSize',15);
    set(cl(1),'Marker','none');
    set(cl(2),'String','$\Gamma$(\boldmath$x$) = 0');
    [~,id] = min(handles.Xs(:,1));
    set(cl(2),'Position',[-0.95,handles.Xs(id,2)-0.05,0]);
    axis equal;
    axis(handles.axisLimits);
    xticks(handles.axisLimits(1):0.2:handles.axisLimits(2));
    yticks(handles.axisLimits(3):0.2:handles.axisLimits(4));
    xlabel('$x$ [$m$]','Interpreter','latex','FontSize',15);
    ylabel('$y$ [$m$]','Interpreter','latex','FontSize',15);
    grid on;
    basePosition = [0;1];
    robot = create_simple_robot_force(basePosition);
    robot.plot([0,-pi/2]);
    view([0,90]);
    axis(handles.axisLimits);
    xticks(handles.axisLimits(1):0.2:handles.axisLimits(2));
    yticks(handles.axisLimits(3):0.2:handles.axisLimits(4));
    xlabel('$x$ [$m$]','Interpreter','latex','FontSize',15);
    ylabel('$y$ [$m$]','Interpreter','latex','FontSize',15);
    rotate3d off;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Initialize simulation variables %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    damp = [100;100];
    v0 = 0.3;
    scale = 0.2/v0;
    targetForce = 20;
    reachingThreshold = 0.03;
    pointThreshold = 0.02;
    pointThresholdTarget = 1.0;
    vidObj = [];
    if(handles.makeAMovie)
        vidObj = VideoWriter('movie.avi');
        vidObj.Quality = 100;
        vidObj.FrameRate = 10;
        open(vidObj);
    end
    idLoop = -1;
    while(~handles.stop && ~handles.updateSurface)
            idLoop = idLoop+1;
%         try
            %%%%%%%%%%%%%%%%%%%%%%
            % Get starting point %
            %%%%%%%%%%%%%%%%%%%%%%
            xs = getStartingPoint(hObject,handles,pointThreshold);
            handles = guidata(hObject);
            if(handles.stop || handles.updateSurface)
                break;
            end
            q = simple_robot_ikin(robot,xs-basePosition);
            robot.animate(q);
            legend(gca,'off');
            if(exist('p','var'))
                delete(p);
            end
            %%%%%%%%%%%%%%%%%%%%
            % Get target point %
            %%%%%%%%%%%%%%%%%%%%
%             if(idLoop == 0) 
                target = getTargetPoint(hObject,handles,pointThresholdTarget);
                handles = guidata(hObject);
%             end
            if (idLoop == 1)
                handles.modulation = true;
                guidata(hObject, handles);
            end
            if(handles.stop || handles.updateSurface)
                break;
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Initialize dynamic plot %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            [xev,yev] = meshgrid(handles.axisLimits(1):0.04:handles.axisLimits(2),...
                         handles.axisLimits(3):0.04:handles.axisLimits(4));
            xev = xev(:);
            yev = yev(:);
%             id = 1:5:length(handles.Xs);
%             xev = [xev;handles.Xs(id,1)];
%             yev = [yev;handles.Xs(id,2)];
            vev = zeros(length(xev),2);
            for k = 1:length(xev)
                normalDistance = calculateGamma(handles.svmgrad,[xev(k);yev(k)]);
                if(abs(normalDistance)<=0.02)
                    e1 = -calculateGammaDerivative(handles.svmgrad,[xev(k);yev(k)]);
                    e1 = e1/norm(e1);
                    temp = [xev(k);yev(k)];
                    temp = temp+normalDistance*e1;
                    xev(k) = temp(1);
                    yev(k) = temp(2);
                    normalDistance = calculateGamma(handles.svmgrad,[xev(k);yev(k)]);
                end
                if(normalDistance>=-0.02)
                    if(normalDistance < 0)
                        normalDistance = 0;
                    end
                    e1 = -calculateGammaDerivative(handles.svmgrad,[xev(k);yev(k)]);
                    e1 = e1/norm(e1);
                    vn = computeNominalDynamics([xev(k);yev(k)],target,normalDistance,e1,v0);    
                    Fd = (1-tanh(20*normalDistance))*targetForce;
                    if(handles.modulation)
                        vev(k,:) = computeModulatedDynamics(vn,Fd,normalDistance,e1,damp)';
                    else
                        vev(k,:) = vn;
                    end     
                end
            end
            
            if(~handles.makeAMovie)
                title('Simulating ...');
            end
            x = xs;
            hold on;
            p(1) = quiver(xev,yev,vev(:,1),vev(:,2),'k','FaceAlpha','AutoScaleFactor',0.5);
            p(2) = plot(x(1),x(2),'color',handles.color(4,:),'lineWidth',6);
            p(3) = quiver(x(1),x(2),0,0,'k','LineWidth',2,'MaxHeadSize',5);
            p(4) = quiver(x(1),x(2),0,0,'color',handles.color(2,:),'LineWidth',2,'MaxHeadSize',5);
            if(handles.modulation) 
                p(5) = quiver(x(1),x(2),0,0,'color',handles.color(1,:),'LineWidth',2,'MaxHeadSize',5);
                p(6) = quiver(x(1),x(2),0,0,'color',handles.color(3,:),'LineWidth',2,'MaxHeadSize',5);
                p(7) = scatter(xs(1),xs(2),300,handles.color(3,:),'filled','^');
                p(8) = scatter(target(1),target(2),300,handles.color(1,:),'filled','^');
            else
                p(5) = scatter(xs(1),xs(2),300,handles.color(3,:),'filled','^');
                p(6) = scatter(target(1),target(2),300,handles.color(1,:),'filled','^');
                
            end
            hold off;
            if(handles.makeAMovie)
                if (handles.modulation)
                    legend([h(1),p],{'Surface levels','Vector field of \boldmath$\dot{x}_d$','\boldmath$x$','\boldmath$n(x)$', ...
                      '\boldmath$f(x)$' ,'\boldmath$\dot{x}_d$',...
                      '\boldmath$f_n(x)$',...
                      '\boldmath$x_0$','target'},'Location', 'NorthEast','location',...
                      'NE','interpreter','latex','FontSize',14);
                else
                    legend([h(1),p],{'Surface levels','Vector field of \boldmath$f(x)$','\boldmath$x$','\boldmath$n(x)$', ...
                      '\boldmath$f(x)$' ,'\boldmath$x_0$','target'},'Location', 'NorthEast','location',...
                      'NE','interpreter','latex','FontSize',14);
                end
            end
            set(gcf,'Pointer','arrow');
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Reset simulation variables %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            t = 0;
            qd = [0,0];
            dt = 0.005;
            count = 0;
            Xsim = [];
            E1sim = [];
            Vdsim = [];
            Vnsim = [];
            Fdsim = [];
            

            %%%%%%%%%%%%%
            % Main loop %
            %%%%%%%%%%%%%
            while(~handles.stop && ~handles.updateSurface)
                % compute state of end-effector
                x = robot.fkine(q);
                x = x(1:2,4);
                v = robot.jacob0(q)*qd';
                v = v(1:2);
                % Compute distance and normal vector to surface
                normalDistance = calculateGamma(handles.svmgrad,x);
                if(normalDistance<0)
                    normalDistance = 0;
                end
                e1 = -calculateGammaDerivative(handles.svmgrad,x);
                e1 = e1/norm(e1);
                % Compute nominal dynamics
%                 normalDistance
                vn = computeNominalDynamics(x,target,normalDistance,e1,v0);
                % Compute modulated dynamics
                if handles.modulation == false
                    vd = vn;
%                     Fd = (1-tanh(20*normalDistance))*targetForce;
                    Fd = 0;
                else
                    Fd = (1-tanh(20*normalDistance))*targetForce;
                    vd = computeModulatedDynamics(vn,Fd,normalDistance,e1,damp);
                end
                % compute cartesian wrench command
                B = findDampingBasis(vd);
                D = B*[damp(1) 0;0 damp(2)]*B';
%                 B = findDampingBasis2(vn,e1);
%                 [U,D,V] = svd(B);
%                 if(min(diag(D))<1e-3)
%                     disp('bou');
%                     B = [vn/norm(vn) vn/norm(vn)];
%                 end
%                 Dinv = zeros(2);
%                 lambda = 1e-3;
%                 for k = 1:2
%                     Dinv(k,k) = D(k,k)/(D(k,k)^2+lambda^2);
%                 end
%                 if(D(1,1)<1e-6)
%                     Dinv(1,1) = 0;
%                 else
%                     Dinv(1,1) = 1/D(1,1);
%                 end
%                 if(D(2,2)<1e-6)
%                     Dinv(2,2) = 0;
%                 else
%                     Dinv(2,2) = 1/D(2,2);
%                 end
%                 D = B*[damp(1) 0;0 damp(2)]*(V*Dinv*U');
%                 D = B*[damp(1) 0;0 damp(2)]*pinv(B);
                wrench = D*(vd-v);
%                 if(norm(wrench)>30)
%                     wrench = wrench*30/norm(wrench);
%                 end
                % Kill the force generated to make the robot staying on the
                % surface
                if(handles.modulation)
                    wrench = wrench-Fd*e1;
                    % Add perturbation force if desired
                    if(t>3 && t <5 && handles.perturbation)
                        if(idLoop==2)
                            wrench = wrench-50*e1;
                        end
                    end         
                end
                % Compute joint torques
                torques = robot.jacob0(q)'*[wrench;zeros(4,1)];
                % Apply control to the robot            
                qdd = robot.accel(q,[0,0],torques')';
                % Integrate one time step
                qd = qd+dt*qdd;
                q = q+qd*dt+qdd/2*dt^2;
                t = t+dt;
                robot.delay = dt;
                 
                Xsim = [Xsim x];
                E1sim = [E1sim e1];
                Vdsim = [Vdsim vd];
                Vnsim = [Vnsim vn];
                Fdsim = [Fdsim wrench'*e1];

                % Check if target reached
                if (norm(x-target)<reachingThreshold && normalDistance < reachingThreshold)
                    if(~handles.modulation)
                        set(p(2),'XData',Xsim(1,:),'YData',Xsim(2,:));
                        set(p(3),'XData',x(1),'YData',x(2),'UData',0.25*e1(1),'VData',0.25*e1(2));
                        temp = vn*scale;
                        set(p(4),'XData',x(1),'YData',x(2),'UData',temp(1),'VData',temp(2));
                        if(handles.modulation)
                            temp = vd*scale;
                            set(p(5),'XData',x(1),'YData',x(2),'UData',temp(1),'VData',temp(2));
                            temp = (Fd/damp(1))*e1*scale;
                            set(p(6),'XData',x(1),'YData',x(2),'UData',temp(1),'VData',temp(2));
                        end
                        robot.animate(q);
                        count = count+1;
                        if(handles.makeAMovie)       
                            writeVideo(vidObj, getframe(gcf));
                        end
                        hold on;
                        temp = (Fd/damp(1))*e1*scale;
                        bou = quiver(x(1),x(2),temp(1),temp(2),'color',handles.color(3,:),'LineWidth',2,'MaxHeadSize',5);
                        hold off;
                        legend([h(1),p,bou],{'Surface levels','Vector field of \boldmath$f(x)$','\boldmath$x$','\boldmath$n(x)$', ...
                        '\boldmath$f(x)$' ,'\boldmath$x_0$','target','Desired contact force'},'Location', 'NorthEast','location',...
                        'NE','interpreter','latex','FontSize',14);
                        legend('-DynamicLegend');
                        end
                    break;
                end
                if(t> 0.08*count)
                    set(p(2),'XData',Xsim(1,:),'YData',Xsim(2,:));
                    set(p(3),'XData',x(1),'YData',x(2),'UData',0.25*e1(1),'VData',0.25*e1(2));
                    temp = vn*scale;
                    set(p(4),'XData',x(1),'YData',x(2),'UData',temp(1),'VData',temp(2));
                    if(handles.modulation)
                        temp = vd*scale;
                        set(p(5),'XData',x(1),'YData',x(2),'UData',temp(1),'VData',temp(2));
                        temp = (Fd/damp(1))*e1*scale;
                        set(p(6),'XData',x(1),'YData',x(2),'UData',temp(1),'VData',temp(2));
                    end
                    robot.animate(q);
                    count = count+1;
                    if(handles.makeAMovie)       
                        writeVideo(vidObj, getframe(gcf));
                    end
                end
                handles = guidata(hObject);
            end
            if(~handles.makeAMovie)
                if (handles.modulation)
                    legend([h(1),p],{'Surface levels','Vector field of \boldmath$\dot{x}_d$','\boldmath$x$','\boldmath$n(x)$', ...
                      '\boldmath$f(x)$' ,'\boldmath$\dot{x}_d$',...
                      '\boldmath$f_n(x)$',...
                      '\boldmath$x_0$','target'},'Location', 'NorthEast','location',...
                      'NE','interpreter','latex','FontSize',14);
                else
                    legend([h(1),p],{'Surface levels','Vector field of \boldmath$f(x)$','\boldmath$x$','\boldmath$n(x)$', ...
                      '\boldmath$f(x)$' ,'\boldmath$x_0$','target'},'Location', 'NorthEast','location',...
                      'NE','interpreter','latex','FontSize',14);
                end
            end
            if(handles.makeAMovie)       
                writeVideo(vidObj, getframe(gcf));
            end
            iptSetPointerBehavior(gca, @(gcf, currentPoint)set(gcf, 'Pointer', 'crosshair'));
            guidata(hObject, handles);
%         catch
%             disp('Could not find joint space configuration. Please choose another point in the workspace.')
%         end
    end
    if(handles.makeAMovie)
        close(vidObj);
        winopen('movie.avi');
    end


function xs = getStartingPoint(hObject,handles,pointThreshold)
    if(~handles.makeAMovie)
        title('Select a starting point above the surface','Interpreter','latex','FontSize',15);
    end
    xs = get_point(gcf,hObject,handles);
    handles = guidata(hObject);
    if(~handles.stop && ~handles.updateSurface)
        normalDistance = calculateGamma(handles.svmgrad,xs);
        while(normalDistance < pointThreshold)
            if(~handles.makeAMovie)
                title('Starting point is not above the surface ... Retry', 'Interpreter','latex','FontSize',15);
            end
            xs = get_point(gcf,hObject,handles);
            handles = guidata(hObject);
            if(~handles.stop && ~handles.updateSurface)
                normalDistance = calculateGamma(handles.svmgrad,xs);
            else
                break;
            end   
        end
    end
    
function B = findDampingBasis(xd)
     y1 = 1;
     y2 = -xd(1)/xd(2);
     y = [y1;y2];
     B = [xd./norm(xd), y./norm(y)];
     
function B = findDampingBasis2(vn,e1)
%     if(norm(cross([vn/norm(vn);0],[e1;0]))<1e-3)
%          B = [vn./norm(vn), vn./norm(vn)];
%     else
        B = [vn/norm(vn), e1];
%     end
    
function target = getTargetPoint(hObject,handles,pointThreshold)
    if(~handles.makeAMovie)
        title('Select a target point on the surface');
    end
    target = get_point(gcf,hObject,handles);
    handles = guidata(hObject);
    if(~handles.stop && ~handles.updateSurface)
        normalDistance = calculateGamma(handles.svmgrad,target);
        while(abs(normalDistance)> pointThreshold)
            if(~handles.makeAMovie)
                title('Target point is not on the surface ... Retry','Interpreter','latex','FontSize',15);
            end
            target = get_point(gcf,hObject,handles);
            handles = guidata(hObject);
            if(~handles.stop && ~handles.updateSurface)
                normalDistance = calculateGamma(handles.svmgrad,target);
            else
                break;
            end      
        end
    end  
  
function vn = computeNominalDynamics(x,target,normalDistance,e1,v0)
    e2 = zeros(2,1);
    e2(1) = 1;
    e2(2) = -e1(1)/e1(2);
    e2 = e2/norm(e2);
    vc = dot(target-x,e2)*e2;
    vc = vc/norm(vc);
    angle = sign(e2'*vc)*acos(e1'*vc);
    beta = (1-tanh(20*normalDistance));
    theta = beta*angle;
    R = [cos(theta) -sin(theta);
         sin(theta) cos(theta)];
    vn = v0*R*e1;

function vd = computeModulatedDynamics(vn,Fd,normalDistance,e1,damp)
%     temp = dot(e1,vn)*Fd/damp(2);
%     la = (-temp+sqrt(temp^2+norm(vn)^4))/norm(vn)^2;
%     vd = la*vn+(Fd/damp(2))*e1;
    vd = vn+(Fd/damp(1))*e1;
%     fprintf('Distance: %.2f Fd: %.2f',normalDistance,Fd);
    
% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    % Update handles structure
    handles = guidata(hObject);
    handles.stop = true;
    guidata(hObject, handles);
    if(handles.startSimulation == false && handles.updateSurface == false)
        close all;
    end
    save('workspace.mat')
    save('model.mat','-STRUCT','handles');
    