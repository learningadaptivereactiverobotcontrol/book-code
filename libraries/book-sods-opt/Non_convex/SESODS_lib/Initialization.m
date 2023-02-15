function [Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V, Data_P_A, Data_V_A,Data_A,demos_P,demos_V,demos,Time]=Initialization(demos,demos_P,demos_V,options)

if (strcmp(options.Velocity,'True')~=1)
    if (strcmp(options.smoothing,'True')==1)
        [demos_P,demos_V,demos]=preprocess_demos_For_smoothing(demos,options.dt);
        Time=max(size(demos_P{1,1}))*options.dt;
        options.Velocity='True';
    else
        demos_P{1}=zeros(options.d,1);
        demos_V{1}=zeros(options.d,1);
        [demos_P,demos_V,demos]=preprocess_demos_For_generating_Velocity(demos,demos_P,demos_V,options.dt,options.tol_cutting,options);    
        Time=max(size(demos_P{1,1}))*options.dt;
    end
else
    [demos_P,demos_V,demos]=preprocess_demos_For_generating_Velocity(demos,demos_P,demos_V,options.dt,options.tol_cutting,options);
    Time=max(size(demos_P{1,1}))*options.dt;
end
[~ , ~,~ , ~, Data_P_A, Data_V_A,Data_A, ~,demos_P,demos_V,demos] = preprocess_demos_Second_Order(demos_P,demos_V,options); 
[Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V,~]=Initilizing_Second_Order(demos, Data_P_A, Data_V_A,options);   
     
