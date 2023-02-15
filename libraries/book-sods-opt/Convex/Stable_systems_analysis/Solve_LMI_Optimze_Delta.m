function [P,A_New,time]=Solve_LMI_Optimze_Delta(A)
d=size(A,1);
K=size(A,3);
tol=10^(-4);

Method=2;
if Method==1
    t_lower = zeros(d,d,K);
    for i=1:K
        t_upper(:,:,i) = eye(d,d);
    end
    P = sdpvar(d,d);
    Delta=sdpvar(d,d,K,'full');
    ops = sdpsettings('solver','sedumi');
    C=[];
    for i=1:K
        C=C+[A(:,:,i)'*P+P*A(:,:,i) <= +Delta(:,:,i)'*P+P*Delta(:,:,i)-tol*eye(d,d)];
    end
    C=C+[P>=tol*eye(d,d)];
    % Fun=0;
    % for i=1:K
    %     Fun=Fun+norm(Delta(:,:,i));
    % end
    Solver = optimizer(C,[],ops,Delta,P);
    [~, errorcode] = Solver{t_upper};
    if (errorcode~=0)
        for i=1:K
            C=[];
            C=[A(:,:,i)'*P+P*A(:,:,i) < +Delta(:,:,i)'*P+P*Delta(:,:,i)];
            C=C+[P>=tol*eye(d,d)];
            %         Fun=norm(Delta(:,:,i));
            Solver = optimizer(C,[],ops,Delta(:,:,i),P);
            [~, errorcode] = Solver{t_upper(:,:,i)};
            while (errorcode~=0)
                t_upper(:,:,i) = t_upper(:,:,i)*2;
                [~, errorcode] = Solver{t_upper(:,:,i)};
            end
        end
        %
        C=[];
        for i=1:K
            C=C+[A(:,:,i)'*P+P*A(:,:,i) < +Delta(:,:,i)'*P+P*Delta(:,:,i)-tol*eye(d,d)];
        end
        C=C+[P>tol*eye(d,d)];
        Solver = optimizer(C,[],ops,Delta,P);
        [Popt, errorcode] = Solver{t_upper};
        while (errorcode~=0)
            t_upper = t_upper*2;
            [Popt, errorcode] = Solver{t_upper};
        end
        %
        Delta=sdpvar(d,d,K,'full');
        C=[];
        for i=1:K
            C=C+[A(:,:,i)'*Popt+Popt*A(:,:,i) < +Delta(:,:,i)'*Popt+Popt*Delta(:,:,i)-tol*eye(d,d)];
        end
        Fun=0;
        for i=1:K
            Fun=Fun+norm(Delta(:,:,i));
        end
        sol = optimize(C,Fun,ops);
        t_upper=value(Delta);
    end
    
    t_lower=t_upper;
    while ((~(errorcode==1))&&(max(max(sqrt(sum(abs(t_lower).^2,1))))>tol))
        t_lower = t_lower/2;
        [~, errorcode] = Solver{t_lower};
    end
    
    t_works = t_upper;
    tic
    while max(max(sqrt(sum(abs(t_upper-t_lower).^2,1))))>tol
        t_test = (t_upper+t_lower)/2;
        %     disp([norm(t_lower(:,:,1)) norm(t_upper(:,:,1)) norm(t_test(:,:,1))])
        [Popt, errorcode] = Solver{t_test};
        if errorcode==1
            t_lower = t_test;
        else
            t_upper = t_test;
            t_works = t_test;
            Pworks = Popt;
        end
    end
    time=toc;
    Delta=sdpvar(d,d,K,'full');
    C=[];
    for i=1:K
        C=C+[A(:,:,i)'*Pworks+Pworks*A(:,:,i) <= +Delta(:,:,i)'*Pworks+Pworks*Delta(:,:,i)-tol*eye(d,d)];
    end
    Fun=0;
    for i=1:K
        Fun=Fun+norm(Delta(:,:,i));
    end
    sol = optimize(C,Fun,ops);
    Delta = value(Delta);
    for i=1:K
        A_New(:,:,i)=A(:,:,i)-Delta(:,:,i);
    end
    
    % CC=[]
    % for i=1:K
    %     CC=CC+[A_New(:,:,i)'*P+P*A_New(:,:,i) < -tol*eye(d,d)];
    % end
    % sol = optimize(CC,[],ops)
else
    t=sdpvar(1,1);
    P0=eye(d,d);
    P = sdpvar(d,d);
    Y=sdpvar(d,d,K,'full');
    ops = sdpsettings('solver','sedumi','verbose',0);
    C=[];
    for i=1:K
        handle=[-(P0*P0-P0*P-P*P0) Y(:,:,i);transpose(Y(:,:,i)) t*eye(d,d)];
        C=C+[tol*eye(2*d,2*d)<=handle];
    end
    for i=1:K
        C=C+[A(:,:,i)'*P+P*A(:,:,i)+ Y(:,:,i)+transpose(Y(:,:,i)) <= -tol*eye(d,d)];
    end
    C=C+[P>=tol*eye(d,d)];
    C=C+[tol<=t];
    Sol = optimize(C,t,ops);
    if Sol.problem~=0
        keyboard
    end
    Y=value(Y);
    P=value(P);
    for i=1:K
        Delta(:,:,i)=inv(P)*Y(:,:,i);
    end
    disp( sprintf('Value of Delta is  %d %d ',[sum(sum((sum(Delta.^2))))]));
    counter=0;
        tic
    while counter<3
        counter=counter+1;
        Delta=sdpvar(d,d,K,'full');
        C=[];
        for i=1:K
            C=C+[A(:,:,i)'*P+P*A(:,:,i)+Delta(:,:,i)'*P+P*Delta(:,:,i) <= -tol*eye(d,d)];
        end
        Fun=0;
        for i=1:K
            Fun=Fun+norm(Delta(:,:,i));
        end
        Sol = optimize(C,Fun,ops);
%         if Sol.problem~=0
%             keyboard
%         end
        Delta=value(Delta);
        disp( sprintf('Value of Delta is %d ',[sum(sum((sum(Delta.^2))))]));
        for i=1:K
            Y(:,:,i)=P*Delta(:,:,i);
        end
        t=sdpvar(1,1);
        mu=sdpvar(1,1);
        P = sdpvar(d,d);
        Y=sdpvar(d,d,K,'full');
        ops = sdpsettings('solver','sedumi','verbose',0);
        C=[];
        for i=1:K
            handle=[-(P0*P0-P0*P-P*P0) Y(:,:,i);transpose(Y(:,:,i)) t*eye(d,d)];
            C=C+[tol*eye(2*d,2*d)<=handle];
        end
        for i=1:K
            C=C+[A(:,:,i)'*P+P*A(:,:,i)+Y(:,:,i)+transpose(Y(:,:,i)) <= -tol*eye(d,d)];
        end
        C=C+[P>=tol*eye(d,d)];
        C=C+[tol<=t];
        Sol = optimize(C,t,ops);
%         if Sol.problem~=0
%             keyboard
%         end
        Y=value(Y);
        P=value(P);
        for i=1:K
            Delta(:,:,i)=inv(P)*Y(:,:,i);
        end
        disp( sprintf('Value of Delta is  %d ',[sum(sum((sum(Delta.^2))))]));
    end
    time=toc;
    Delta=sdpvar(d,d,K,'full');
    C=[];
    for i=1:K
        C=C+[A(:,:,i)'*P+P*A(:,:,i)+Delta(:,:,i)'*P+P*Delta(:,:,i)<= -tol*eye(d,d)];
    end
    Fun=0;
    for i=1:K
        Fun=Fun+norm(Delta(:,:,i));
    end
    Sol = optimize(C,Fun,ops);
    Delta=value(Delta);
    disp( sprintf('The final Value of Delta is  %d ',[sum(sum((sum(Delta.^2))))]));
    for i=1:K
        A_New(:,:,i)=A(:,:,i)+Delta(:,:,i);
    end
end