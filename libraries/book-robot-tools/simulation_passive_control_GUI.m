function [hd, ht] = simulation_passive_control_GUI(fig, robot, base, reshaped_ds, target, q, dt, varargin)
handle(fig)
t = 0;
qd = [0,0];
hd = [];
ht = [];

global pertForce;
pertForce = [0;0];
maxPert = [0;0];

% Variable for Plotting Damping Matrix
i = 1;
mod_step = 40;
color = [ 0.6 0.4 0.6; 1 0.4 0.6];

if nargin == 8
    struct_stiff = varargin{1};
    gmm = struct_stiff.gmm;
    A_g = struct_stiff.A_g;
    K = length(gmm.Priors);
    basis_type = struct_stiff.basis;
    DS_type = struct_stiff.DS_type;
    if strcmp(DS_type,'lags')        
        A_l =      struct_stiff.A_l;
        A_d = struct_stiff.A_d  ;
        att_k  = struct_stiff.att_k;
        alpha_fun = struct_stiff.alpha_fun;
        grad_alpha_fun = struct_stiff.grad_alpha_fun;
        h_functor = struct_stiff.h_functor;
        grad_h_functor = struct_stiff.grad_h_functor;
        lambda_functor = struct_stiff.lambda_functor;
        grad_lambda_functor = struct_stiff.grad_lambda_functor;
    end
end

% Stop recording
button_pos = [400 10 80 22];
stop_btn = uicontrol('style','pushbutton','String', 'Stop Sim','Callback',@stop_sim, ...
          'position', button_pos, 'UserData', 1); 
      
while ( (get(stop_btn, 'UserData') == 1))
    % Set perturbations with the mouse
    set(gcf,'WindowButtonDownFcn',@startPerturbation);
    
    % compute state of end-effector
    x = robot.fkine(q);
    x = x(1:2,4); 
    xd = robot.jacob0(q)*qd';    
    xd = xd(1:2) ;  
    
    %reference_vel(t);
%     xd_ref = reshaped_ds(x-target);
    xd_ref = reshaped_ds(x);
    
    % put lower bound on speed, just to speed up simulation
    th = 1.0;
    if(norm(xd_ref)<th)
        xd_ref = xd_ref/norm(xd_ref)*th;
    end
    xdd_ref = -(xd - xd_ref)/dt*0.5;
    
    % Compute Damping Matrix
    Q = findDampingBasis(xd_ref);
    L = [1 0;0 2];
    D = Q*L*Q';
    
    % Plot Damping/Stiffness Matrix    
    if (i == 1) || (mod(i,mod_step) == 0)
        if nargin == 8
            D_scaled = Q*(0.01*real(L) + eps)*Q';
            hd = [hd, ml_plot_gmm_contour(gca,1,x,D_scaled,color(1,:),1)];
            Jacobian_DS = zeros(2,2,K);
            
            switch DS_type
                case 'global'
                    for k=1:K
                        gamma_k_x    = gamma_prob_fun(x, gmm, k);
                        gamma_k_grad = gamma_prob_grad_fun(x, gmm, k);
                        Jacobian_DS(:,:,k) = (gamma_k_grad*x' + gamma_k_x*eye(2,2))*A_g(:,:,k)';
                    end
                    
                case 'lags'
                    for k=1:K
                        % Computation of each k-th DS Jacobian
                        % gamma and gradient
                        gamma_k_x    = gamma_prob_fun(x, gmm, k);
                        gamma_k_grad = gamma_prob_grad_fun(x, gmm, k);
                        
                        % computing first term of weighted jacobian of k-th DS
                        A_l_k = h_functor{k}(x)*A_l(:,:,k) + (1-h_functor{k}(x))*A_d(:,:,k);
                        f_k_t = alpha_fun(x)*x'*A_g(:,:,k)' + (1-alpha_fun(x))*( (x - att_k(:,k))'*A_l_k' - lambda_functor{k}(x)*grad_h_functor{k}(x)');
                        first_term = gamma_k_grad*f_k_t;
                        
                        % computing second term of weighted jacobian of k-th DS
                        global_term = (grad_alpha_fun(x)*x' + alpha_fun(x)*eye(2,2)) *A_g(:,:,k)';
                        local_term = (-grad_alpha_fun(x)*(x - att_k(:,k))' + (1-alpha_fun(x))*eye(2,2))*A_l_k';
                        modulation_term = (grad_alpha_fun(x)*lambda_functor{k}(x) - (1-alpha_fun(x))*grad_lambda_functor{k}(x))*grad_h_functor{k}(x)';
                        local_inter_term = (1-alpha_fun(x))* grad_h_functor{k}(x) * (x - att_k(:,k))' * (A_l(:,:,k) - A_d(:,:,k))';
                        second_term = gamma_k_x * (global_term + local_term + modulation_term + local_inter_term);
                        Jacobian_DS(:,:,k) = first_term + second_term;
                    end
            end
            Jacobian_DS_sym = sum(Jacobian_DS,3);
            eig(Jacobian_DS_sym)
            
            % Apparent Stiffness projected on the defined Basis
            switch basis_type
                case 'D'
                    E = Q;
                case 'I'
                    E = eye(2);
            end
            
            K_stiff_1   = - L(1,1) * (E(:,1)'  * Jacobian_DS_sym * E(:,1));
            K_stiff_2   = - L(1,1) * (E(:,2)'  * Jacobian_DS_sym * E(:,2));
            K_stiff_eig = abs([K_stiff_1 0; 0 K_stiff_2]);
            K_scaled = E *(0.01*K_stiff_eig + eps)*E'  ;
            hd = [hd, ml_plot_gmm_contour(gca,1,x,K_scaled,color(2,:),1)];
            ht = [ht, plot(x(1), x(2), 'm+', 'markersize', 30)];
        end        
    end
    
    % Compute Cartesian Control    
    u_cart = - D*(xd-xd_ref);
    
    % feedforward term
    u_cart = u_cart + simple_robot_cart_inertia(robot,q)*xdd_ref;       
    
    % external perturbations with the mouse
    u_cart = u_cart + pertForce;
    
    % compute joint space control
    u_joint = robot.jacob0(q)'*[u_cart;zeros(4,1)];
    
    % apply control to the robot
    qdd = robot.accel(q,qd,u_joint')';
    
    % integrate one time step
    qd = qd+dt*qdd;
    q = q+qd*dt+qdd/2*dt^2;
    t = t+dt;
    if (norm(x - target)<0.01)
        break
    end
    robot.delay = dt;
    robot.animate(q);
  
    i = i + 1;
end
delete(stop_btn)


%% Perturbations with the mouse and simulation stopping
    function stop_sim(ObjectS, ~)
        set(ObjectS, 'UserData', 0);
    end
    function startPerturbation(~,~)
        motionData = [];
        set(gcf,'WindowButtonMotionFcn',@perturbationFromMouse);
        x_p = get(gca,'Currentpoint');
        x_p = x_p(1,1:2)';
        hand = plot(x_p(1),x_p(2),'r.','markersize',20);
        hand2 = plot(x_p(1),x_p(2),'r.','markersize',20);
        set(gcf,'WindowButtonUpFcn',@(h,e)stopPerturbation(h,e));
        
        function stopPerturbation(~,~)
            delete(hand)
            delete(hand2)
            set(gcf,'WindowButtonMotionFcn',[]);
            set(gcf,'WindowButtonUpFcn',[]);
            pertForce = [0;0];
        end
        
        
        function ret = perturbationFromMouse(~,~)
            x_p = get(gca,'Currentpoint');
            x_p = x_p(1,1:2)';
            motionData = [motionData, x_p];
            pertForce = 20*(motionData(:,end)-motionData(:,1));
            norm_pertForce = norm(pertForce);
            if norm_pertForce > norm(maxPert)
                maxPert = pertForce;
            end
            ret=1;
            delete(hand2)
            hand2 = plot([motionData(1,1),motionData(1,end)],[motionData(2,1),motionData(2,end)],'-r');
        end
    end


end
