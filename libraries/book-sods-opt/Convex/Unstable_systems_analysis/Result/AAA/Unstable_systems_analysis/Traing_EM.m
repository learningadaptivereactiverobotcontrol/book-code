function [prior,Mu,Sigma,A,b,Priors_0, Mu_0, Sigma_0,time]=Traing_EM(Data,options)
Method=1;
Matlab=false;
if Method==1
    if (Matlab==true)
        Options = statset('Display','final','MaxIter',1);
        obj = gmdistribution.fit(transpose(Data),options.K,'Options',Options,'Regularize',10^(-12),'Replicates',1);
        Priors_0=obj.ComponentProportion;
        Mu_0=transpose(obj.mu);
        Sigma_0=obj.Sigma;
        Options = statset('Display','off','MaxIter',5000);
        tic
        obj = gmdistribution.fit(transpose(Data),options.K,'Options',Options,'Regularize',10^(-12),'Replicates',10);
        time=toc/10;
        prior=obj.ComponentProportion;
        Mu=transpose(obj.mu);
        Sigma=obj.Sigma;
    else
        tic
        [Priors_0, Mu_0, Sigma_0] = EM_init_kmeans(Data, options.K);
        [prior, Mu, Sigma] = EM(Data, Priors_0, Mu_0, Sigma_0);
        time=toc;
    end
else
    tic
    [Priors_0, Mu_0, Sigma_0] = EM_init_kmeans(Data, options.K);
    [prior, Mu, Sigma] = EM(Data, Priors_0, Mu_0, Sigma_0);
    time=toc;
    if (Matlab==true)
        [prior,Mu,Sigma]=Solver_MSE(prior,Mu,Sigma,Data);
    else
        %         [Priors Mu Sigma]=MSE_Solver(prior,Mu,Sigma,Data,options);
        for i=1:10
            [prior,Mu,Sigma]=Solver_MSE_NoSigma(prior,Mu,Sigma,Data);
            [prior,Mu,Sigma]=Solver_MSE_Yalmip(prior,Mu,Sigma,Data);
        end
    end
end
d = size(Sigma,1)/2;
for i=1:options.K
    Sigmainput=Sigma(1:d,1:d,i);
    SigmaInput_Output=Sigma((d+1):2*d,1:d,i);
    A(:,:,i)=SigmaInput_Output/(Sigmainput);
    b(:,i)=Mu(d+1:2*d,i)-A(:,:,i)*Mu(1:d,i);
end