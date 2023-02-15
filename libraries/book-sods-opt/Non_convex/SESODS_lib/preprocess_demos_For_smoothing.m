function [demos_P,demos_V,demos_O]=preprocess_demos_For_smoothing(demos1,dt)
norm=0;
for i=1:length(demos1)
clear Time tmp tmp_d tmp_dd tmp_DD
    for ii=1:length(demos1{i})
        Time(ii)=dt*ii;
    end
      d = size(demos1{1},1);
    for j=1:d
        tmp(j,:) = smooth(demos1{i}(j,:),25); 
    end
    %saving the final point (target) of each demo
    xT(:,i) = demos1{i}(:,end); 
    
    % shifting demos to the origin
    tmp = tmp - repmat(xT(:,i),1,size(tmp,2));
    if norm==0
        norm=max(max(abs((tmp))));
%         norm=1;
    end
    tmp=tmp/norm;
    if d==2
     plot(tmp(1,:),tmp(2,:))
     hold on
    end
    x0=tmp(:,1);
    % computing the first time derivative
    tmp_d = diff(tmp,1,2)/dt;
    for j=1:d
        tmp_d(j,:) = smooth(tmp_d(j,:),25); 
    end
    tmp_d(:,end+1)=zeros(d,1); 
    V0=tmp_d(:,1);
    tmp_dd = diff(tmp_d,1,2)/dt;
    for j=1:d
        tmp_dd(j,:) = smooth(tmp_dd(j,:),25); 
    end
    tmp_dd(:,end+1)=tmp_dd(:,end);
    clear tmp
    tmp_D(:,1)=V0;
    tmp(:,1)=x0;
    for j=1:d
        sf = polyfit(Time',tmp_d(j,:)',10);
        T(1)=0;
        Dt=0.005;
        T(2)=Dt;
        ii=1;
        while T(ii)<Time(end)
            tmp_DD(j,ii) =polyval(polyder(sf),T(ii));
            tmp_D(j,ii+1)=polyval(sf,T(ii));
%             tmp_DD(j,ii)=polyval(sf,T(ii));
%             tmp_D(j,ii+1)=tmp_D(j,ii)+tmp_DD(j,ii)*dt;
            tmp(j,ii+1)=tmp(j,ii)+tmp_D(j,ii+1)*Dt;
            T(ii+1)=T(ii)+Dt;
            ii=ii+1;
        end
        tmp_DD(j,ii)=tmp_DD(j,ii-1);
    end
    if d==2
  plot(tmp(1,:),tmp(2,:))
     hold on
     figure();
     plot(T,tmp_DD(1,:),'.')
     hold on
      plot(Time,tmp_dd(1,:),'.')
     plot(T,tmp_DD(2,:),'.')
          plot(Time,tmp_dd(2,:),'.')
          close all
               figure();
     plot(T,tmp_D(1,:),'.')
     hold on
          plot(Time,tmp_d(1,:),'.')
     plot(T,tmp_D(2,:),'.')
          plot(Time,tmp_d(2,:),'.')
     hold on
    end
    demos_V{i}=[tmp_D; tmp_DD];
    demos_P{i}=[tmp;tmp_DD];
    demos_O{i}=[tmp;tmp_D;tmp_DD];
end
    

