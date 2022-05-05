function  Fs = surfaceForce(x,xs)

Fs = [0;0;0];

if(x(3)<xs(3))
%     if (x(1)>0)
        Fs(3) = -(1000 +100000*x(1)^2+400000*x(2)^2)*(x(3)-xs(3)); 
%     else
%         Fs(3) = -(2000 +300000*x(1)^2+100000*x(2)^2)*(x(3)-xs(3)); 
%     end
end

end