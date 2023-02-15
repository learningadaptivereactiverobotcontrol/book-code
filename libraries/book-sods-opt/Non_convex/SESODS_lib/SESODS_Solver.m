function [Priors_P, Mu_P, Sigma_P,Priors_V, Mu_V, Sigma_V]=SESODS_Solver(Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V, Data_P_A, Data_V_A,Data_A,demos_P,demos_V,Time,options)


K=options.K;
d=options.d;
%% S-SD is constructed here
C=Initialize_Epsilon(Sigma_0_P,Sigma_0_V,K,d);
EPSILON=abs(max(C))/10;   

if (strcmp(options.method,'SYM')==1)||(strcmp(options.Method_For_APA,'SYM')==1)
    [Priors_0_P_SYM, Mu_0_P_SYM, Sigma_0_P_SYM ,Priors_0_V_SYM ,Mu_0_V_SYM ,Sigma_0_V_SYM]=Solver_MSE_Sym(Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V,Data_A,EPSILON,options);
    [Priors_P, Mu_P, Sigma_P,Priors_V, Mu_V, Sigma_V]=Solver_MSE_Sym_G(Priors_0_P_SYM, Mu_0_P_SYM, Sigma_0_P_SYM ,Priors_0_V_SYM ,Mu_0_V_SYM ,Sigma_0_V_SYM,Data_A,EPSILON,options);
     Simulation_SYM(Priors_P, Mu_P, Sigma_P,Priors_V, Mu_V, Sigma_V,Data_P_A,Data_V_A,demos_P,demos_V,2*Time);
end
%% J-SD is constructed here
if (strcmp(options.Method_For_APA,'SYM')==1)
    [P, F,Sigma_0_P_SYM,Sigma_0_V_SYM]=P_Calculator(Sigma_0_P_SYM,Sigma_0_V_SYM);
    if F==0
       [P, ~,Sigma_0_P,Sigma_0_V]=P_Calculator(Sigma_0_P,Sigma_0_V);
    else
        Priors_0_P=Priors_0_P_SYM;
        Mu_0_P=Mu_0_P_SYM;
        Sigma_0_P=Sigma_0_P_SYM;
        Mu_0_V=Mu_0_V_SYM;
        Sigma_0_V=Sigma_0_V_SYM;
    end
end

if (strcmp(options.method,'APA')==1)
    if (strcmp(options.Method_For_APA,'SYM')~=1)
        [P, ~,Sigma_0_P,Sigma_0_V]=P_Calculator(Sigma_0_P,Sigma_0_V);
    end
    p0 = GMM_2_Parameters(Priors_0_P,Mu_0_P,Sigma_0_P, Mu_0_V, Sigma_0_V,P,d,K);
    [C, ~, ~, ~]=ctr_eigenvalue(p0,d,K,0,Sigma_0_P,Sigma_0_V); 
    EPSILON=abs(max(C)); 
    [Priors_P, Mu_P, Sigma_P, Mu_V, Sigma_V,~,~]=Solver_MSE_APA(Priors_0_P,Mu_0_P,Sigma_0_P, Mu_0_V, Sigma_0_V,P,Data_A,EPSILON,options);  
    Simulation_APA(Priors_P, Mu_P, Sigma_P, Mu_V, Sigma_V,Data_P_A,demos_P,demos_V,10*Time);
    Priors_V=[];
end