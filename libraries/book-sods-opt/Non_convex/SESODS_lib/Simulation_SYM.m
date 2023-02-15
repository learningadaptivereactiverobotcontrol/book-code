function Simulation_SYM(Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V,Data_P_A,Data_V_A,demos_P,demos_V,Time)
figure1 = figure;
% 
% Create axes
axes1 = axes('Parent',figure1,'FontSize',14);
box(axes1,'on');
grid(axes1,'on');
hold(axes1,'on');

% Create ylabel
ylabel('\xi(1,2)','FontSize',20);

% Create xlabel
xlabel('\xi(1,1)','FontSize',20);
for j=1:length(demos_P) 
A=plot(demos_P{j}(1,:),demos_P{j}(2,:),'DisplayName','Demonstration','LineWidth',3, 'Color',[0 0.443137258291245 0.737254917621613]);
hold on
% end 

C=plot(demos_P{j}(1,1),demos_P{j}(2,1),'DisplayName','The Initial Point','MarkerFaceColor',[0 0.447058826684952 0.74117648601532],...
    'MarkerSize',15,...
    'Marker','hexagram',...
    'LineStyle','none',...
    'Color',[0 0.447058826684952 0.74117648601532]);
D=plot(0,0,'DisplayName','The Target','MarkerFaceColor',[1 0 0],'MarkerSize',15,'Marker','pentagram',...
    'LineStyle','none',...
    'Color',[1 0 0]); 
hold on 
end
step_Time=0.01;
d=size(Mu_0_P,1)/2;
options = odeset('RelTol',1e-4,'AbsTol',1e-4);
for j=1:length(demos_P)
    Handle_X=demos_P{j}(1:d,1);
    Handle_DX=demos_V{j}(1:d,1);
%  Handle_X=Data_P_A(1:d,1);
%   Handle_DX=Data_V_A(1:d,1);
    Xrobot_object_initial=transpose([Handle_X ;Handle_DX]);
% %     
%     for t=0:step_Time:100*Time
%     X  = Handle_X; 
%     DX =Handle_DX;  
%     [DDX,Denominator_P,Denominator_V,Numerator_P,Numerator_V,A_P,A_V]= SE_DS(Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V,DX,X);
%     Handle_DX=DX+step_Time*DDX
%     Handle_X=X+step_Time*Handle_DX;
%     plot(Handle_X(1,1),Handle_X(2,1),'.','Color',[1 0 1-0.5]);
%     end
[T1,x] = ode45(@(t,xi_r) SE_Second(t,xi_r,Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V),[0:0.01:Time],Xrobot_object_initial,options);
X(:,:,j)=x(:,1:d);
end
for j=1:length(demos_P) 
B=plot(X(1,1,j),X(1,2,j),'DisplayName','Generated trajectory','LineWidth',4,'LineStyle','-',...
    'Color',[1 0 0.5]);
end

 legend1 = legend([A B C D],{'The Demonstration','The Generated Trajectory','The Initial Point','The Target'});
 set(legend1,'Visible','off','FontSize',9);
for j=1:length(demos_P) 
plot(X(:,1,j),X(:,2,j),'LineWidth',4,'LineStyle','-',...
    'Color',[1 0 0.5]);
end
% for i=1:size(X,1)
%     if rem(i,10)==0
%         for j=1:length(demos_P)
%             plot(X(i,1,j),X(i,2,j),'.','Color',[(j/2)^2 0 1-j/2]);
%             hold on
%         end
%     end
%     pause(0.01)
% end



function [Dx_main,Denominator_P,Denominator_V,Numerator_P,Numerator_V,A_P,A_V]= SE_DS(Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V,DX,X)

K=size(Sigma_0_P);
if max(size(K))==2
    K(1,3)=1;
end
d=size(X);
Input=zeros(d(1,1),1);
Numerator=zeros(1,K(1,3));
A=zeros(d(1,1),d(1,1),K(1,3));
%%% Position
for i=1:d(1,1)
    Input(i,1)=X(i,1);
end

Output=0;
Denominator=0; 
for j=1:K(1,3)
    Sigmainput=Sigma_0_P(1:d(1,1),1:d(1,1),j);
    Muinput=Mu_0_P(1:d(1,1),j);
    PI=Priors_0_P(1,j);
    Numerator(1,j)=(1/sqrt(((2*pi)^(d(1,1)))*abs(det(Sigmainput))))*exp(-0.5*abs((transpose(Input-Muinput)/(Sigmainput))*(Input-Muinput)));
    Denominator=Denominator+PI*Numerator(1,j);
end
if Denominator==0
for j=1:K(1,3)
    Sigmainput=Sigma_0_P(1:d(1,1),1:d(1,1),j);
    Muinput=Mu_0_P(1:d(1,1),j);
    PI=Priors_0_P(1,j);
    Handle_Num=-0.5*abs((transpose(Input-Muinput)/(Sigmainput))*(Input-Muinput));
    Numerator(1,j)=0;
    for ii=0:10
            Numerator(1,j)=Numerator(1,j)+Handle_Num^(ii)/factorial(ii);
    end
    Numerator(1,j)=(1/sqrt(((2*pi)^(d(1,1)))*abs(det(Sigmainput))))*Numerator(1,j);
    Denominator=Denominator+PI*Numerator(1,j);
end
end
Denominator_P=Denominator;

for o=1:K(1,3) 
    Sigmainput=Sigma_0_P(1:d(1,1),1:d(1,1),o);
    SigmaInput_Output=Sigma_0_P(d(1,1)+1:2*d(1,1),1:d(1,1),o);
    Muinput=Mu_0_P(1:d(1,1),o); 
    PI=Priors_0_P(1,o);
    A(:,:,o)=SigmaInput_Output/(Sigmainput);
    if norm((PI*Numerator(1,o)/Denominator)*(A(:,:,o)*(Input)))<1000
    Output=Output+(PI*Numerator(1,o)/Denominator)*(A(:,:,o)*(Input));
    end
end
Numerator_P=Numerator;
A_P=A;


%% Velocity
for i=1:d(1,1)
    Input(i,1)=DX(i,1);
end
DOutput=0;
Denominator=0;
for j=1:K(1,3)
    Sigmainput=Sigma_0_V(1:d(1,1),1:d(1,1),j);
    Muinput=Mu_0_V(1:d(1,1),j);
    PI=Priors_0_V(1,j);
    Numerator(1,j)=(1/sqrt(((2*pi)^(d(1,1)))*abs(det(Sigmainput))))*exp(-0.5*abs((transpose(Input-Muinput)/(Sigmainput))*(Input-Muinput)));
    Denominator=Denominator+PI*Numerator(1,j);
end
if Denominator==0
for j=1:K(1,3)
    Sigmainput=Sigma_0_V(1:d(1,1),1:d(1,1),j);
    Muinput=Mu_0_V(1:d(1,1),j);
    PI=Priors_0_V(1,j);
    Handle_Num=-0.5*abs((transpose(Input-Muinput)/(Sigmainput))*(Input-Muinput));
    Numerator(1,j)=0;
    for ii=0:10
            Numerator(1,j)=Numerator(1,j)+Handle_Num^(ii)/factorial(ii);
    end
    Numerator(1,j)=(1/sqrt(((2*pi)^(d(1,1)))*abs(det(Sigmainput))))*Numerator(1,j);
    Denominator=Denominator+PI*Numerator(1,j);
end
end
Denominator_V=Denominator;

for o=1:K(1,3)
    Sigmainput=Sigma_0_V(1:d(1,1),1:d(1,1),o);
    SigmaInput_Output=Sigma_0_V(d(1,1)+1:2*d(1,1),1:d(1,1),o);
    Muinput=Mu_0_V(1:2,o);

    PI=Priors_0_V(1,o);
    A(:,:,o)=SigmaInput_Output/(Sigmainput);
    if norm((PI*Numerator(1,o)/Denominator)*(A(:,:,o)*(Input)))<1000
    DOutput=DOutput+(PI*Numerator(1,o)/Denominator)*(A(:,:,o)*(Input));
    end
end
A_V=A;
Numerator_V=Numerator;
Dx_main=Output+DOutput;


function dy = SE_Second(t,xi_r,Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V)
K=size(Sigma_0_P);
if max(size(K))==2
    K(1,3)=1;
end
d=size(xi_r);
d(1,1)=d(1,1)/2;
Input=zeros(d(1,1),1);
DInput=zeros(d(1,1),1);
Numerator=zeros(1,K(1,3));
A=zeros(d(1,1),d(1,1),K(1,3));
%%% Position
for i=1:d(1,1)
    Input(i,1)=xi_r(i,1);
end
for i=1:d(1,1)
    DInput(i,1)=xi_r(i+d(1,1),1);
end

Output=0;
Denominator=0; 
for j=1:K(1,3)
    Sigmainput=Sigma_0_P(1:d(1,1),1:d(1,1),j);
    Muinput=Mu_0_P(1:d(1,1),j);
    PI=Priors_0_P(1,j);
    Numerator(1,j)=(1/sqrt(((2*pi)^(d(1,1)))*abs(det(Sigmainput))))*exp(-0.5*abs((transpose(Input-Muinput)/(Sigmainput))*(Input-Muinput)));
    Denominator=Denominator+PI*Numerator(1,j);
end
if Denominator==0
for j=1:K(1,3)
    Sigmainput=Sigma_0_P(1:d(1,1),1:d(1,1),j);
    Muinput=Mu_0_P(1:d(1,1),j);
    PI=Priors_0_P(1,j);
    Handle_Num=-0.5*abs((transpose(Input-Muinput)/(Sigmainput))*(Input-Muinput));
    Numerator(1,j)=0;
    for ii=0:10
            Numerator(1,j)=Numerator(1,j)+Handle_Num^(ii)/factorial(ii);
    end
    Numerator(1,j)=(1/sqrt(((2*pi)^(d(1,1)))*abs(det(Sigmainput))))*Numerator(1,j);
    Denominator=Denominator+PI*Numerator(1,j);
end
end

for o=1:K(1,3) 
    Sigmainput=Sigma_0_P(1:d(1,1),1:d(1,1),o);
    SigmaInput_Output=Sigma_0_P(d(1,1)+1:2*d(1,1),1:d(1,1),o);
    PI=Priors_0_P(1,o);
    A(:,:,o)=SigmaInput_Output/(Sigmainput);
    Output=Output+(PI*Numerator(1,o)/Denominator)*(A(:,:,o)*(Input));
end
%% Velocity
Input=DInput;
DOutput=0;
Denominator=0;
for j=1:K(1,3)
    Sigmainput=Sigma_0_V(1:d(1,1),1:d(1,1),j);
    Muinput=Mu_0_V(1:d(1,1),j);
    PI=Priors_0_V(1,j);
    Numerator(1,j)=(1/sqrt(((2*pi)^(d(1,1)))*abs(det(Sigmainput))))*exp(-0.5*abs((transpose(Input-Muinput)/(Sigmainput))*(Input-Muinput)));
    Denominator=Denominator+PI*Numerator(1,j);
end
if Denominator==0
for j=1:K(1,3)
    Sigmainput=Sigma_0_V(1:d(1,1),1:d(1,1),j);
    Muinput=Mu_0_V(1:d(1,1),j);
    PI=Priors_0_V(1,j);
    Handle_Num=-0.5*abs((transpose(Input-Muinput)/(Sigmainput))*(Input-Muinput));
    Numerator(1,j)=0;
    for ii=0:10
            Numerator(1,j)=Numerator(1,j)+Handle_Num^(ii)/factorial(ii);
    end
    Numerator(1,j)=(1/sqrt(((2*pi)^(d(1,1)))*abs(det(Sigmainput))))*Numerator(1,j);
    Denominator=Denominator+PI*Numerator(1,j);
end
end

for o=1:K(1,3)
    Sigmainput=Sigma_0_V(1:d(1,1),1:d(1,1),o);
    SigmaInput_Output=Sigma_0_V(d(1,1)+1:2*d(1,1),1:d(1,1),o);
    PI=Priors_0_V(1,o);
    A(:,:,o)=SigmaInput_Output/(Sigmainput);
    DOutput=DOutput+(PI*Numerator(1,o)/Denominator)*(A(:,:,o)*(Input));
end
Dx_main=Output+DOutput;
dy=([DInput;Dx_main]);