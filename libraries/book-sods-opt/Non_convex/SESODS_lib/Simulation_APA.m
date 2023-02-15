function Simulation_APA(Priors_0_P,Mu_0_P,Sigma_0_P, Mu_0_V, Sigma_0_V,Data_P_A,demos_P,demos_V,Time)
figure1 = figure;

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
A=plot(demos_P{j}(1,:),demos_P{j}(2,:),'DisplayName','Demonstration','LineWidth',3,'Color',[0 0.443137258291245 0.737254917621613]);
hold on

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
    Handle_X=demos_P{1,j}(1:d,1);
    Handle_DX=demos_V{1,j}(1:d,1);
    Xrobot_object_initial=transpose([Handle_X ;Handle_DX]);
[T1,x] = ode45(@(t,xi_r) SE_Second(t,xi_r,Priors_0_P,Mu_0_P,Sigma_0_P, Mu_0_V, Sigma_0_V),[0:0.001:Time],Xrobot_object_initial,options);
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
%     if rem(i,5)==0
%         for j=1:length(demos_P)
%             plot(X(i,1,j),X(i,2,j),'LineWidth',4,'LineStyle','-',...
%     'Color',[1 0 0.5]);
%             hold on
%         end
%         
%     pause(0.01)
%     end
% end






 function dy = SE_Second(t,xi_r,Priors_0_P,Mu_0_P,Sigma_0_P, Mu_0_V, Sigma_0_V)
K=size(Sigma_0_P);
if max(size(K))==2
    K(1,3)=1;
end
d=size(xi_r);
d(1,1)=d(1,1)/2;
Input=zeros(d(1,1),1);
DInput=zeros(d(1,1),1);
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
    Sigmainput_P=Sigma_0_P(1:d(1,1),1:d(1,1),j);
    Muinput_P=Mu_0_P(1:d(1,1),j);
    Sigmainput_V=Sigma_0_V(1:d(1,1),1:d(1,1),j);
    Muinput_V=Mu_0_V(1:d(1,1),j);
    PI=Priors_0_P(1,j);
    Numerator_P(1,j)=(1/sqrt(((2*pi)^(d(1,1)))*abs(det(Sigmainput_P))))*exp(-0.5*abs((transpose(Input-Muinput_P)/(Sigmainput_P))*(Input-Muinput_P)));
    Numerator_V(1,j)=(1/sqrt(((2*pi)^(d(1,1)))*abs(det(Sigmainput_V))))*exp(-0.5*abs((transpose(DInput-Muinput_V)/(Sigmainput_V))*(DInput-Muinput_V)));
    Denominator=Denominator+PI*Numerator_P(1,j)*Numerator_V(1,j);
end
if Denominator==0
% for j=1:K(1,3)
%     Sigmainput_P=Sigma_0_P(1:d(1,1),1:d(1,1),j);
%     Muinput_P=Mu_0_P(1:d(1,1),j);
%     Sigmainput_V=Sigma_0_V(1:d(1,1),1:d(1,1),j);
%     Muinput_V=Mu_0_V(1:d(1,1),j);
%     PI=Priors_0_P(1,j);
%     Numerator_P(1,j)=(1/sqrt(((2*pi)^(d(1,1)))*abs(det(Sigmainput_P))))*1/(0.5*abs((transpose(Input-Muinput_P)/(Sigmainput_P))*(Input-Muinput_P)));
%     Numerator_V(1,j)=(1/sqrt(((2*pi)^(d(1,1)))*abs(det(Sigmainput_V))))*1/(0.5*abs((transpose(DInput-Muinput_V)/(Sigmainput_V))*(DInput-Muinput_V)));
%     Denominator=Denominator+PI*Numerator_P(1,j)*Numerator_V(1,j);
% end
   Denominator=realmin;
end

for o=1:K(1,3) 
    Sigmainput_P=Sigma_0_P(1:d(1,1),1:d(1,1),o);
    SigmaInput_Output_P=Sigma_0_P(d(1,1)+1:2*d(1,1),1:d(1,1),o);
%     Muinput_P=Mu_0_P(1:d(1,1),o);
    Sigmainput_V=Sigma_0_V(1:d(1,1),1:d(1,1),o);
    SigmaInput_Output_V=Sigma_0_V(d(1,1)+1:2*d(1,1),1:d(1,1),o);
%     Muinput_V=Mu_0_V(1:d(1,1),o); 
    PI=Priors_0_P(1,o);
%     Numerator_P(1,o)=(1/sqrt(((2*pi)^(K(1,3)+d(1,1)))*abs(det(Sigmainput_P))))*exp(-0.5*abs((transpose(Input-Muinput_P)/(Sigmainput_P))*(Input-Muinput_P)));
%     Numerator_V(1,o)=(1/sqrt(((2*pi)^(K(1,3)+d(1,1)))*abs(det(Sigmainput_V))))*exp(-0.5*abs((transpose(DInput-Muinput_V)/(Sigmainput_V))*(DInput-Muinput_V)));
    A_P(:,:,o)=SigmaInput_Output_P/(Sigmainput_P);
    A_V(:,:,o)=SigmaInput_Output_V/(Sigmainput_V);
    Output=Output+(PI*Numerator_P(1,o)*Numerator_V(1,o)/Denominator)*(A_P(:,:,o)*(Input)+A_V(:,:,o)*(DInput));
end
Dx_main=Output;
dy=([DInput;Dx_main]);