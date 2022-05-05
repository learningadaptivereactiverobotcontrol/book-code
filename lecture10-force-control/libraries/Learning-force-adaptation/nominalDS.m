function vd = nominalDS(x,xc,rd,vt,n,normal_distance)
    % Compute circular DS
    vd_circular = circularDS(x,xc,rd);
    % Project circular DS dynamics on surface
    vd_contact = (eye(3)-n*n')*vd_circular;
    % Normalize projected dynamics
    vd_contact = vd_contact/norm(vd_contact);
    % Compute angle between normal vector and projected dynamics
    angle = acos(dot(n,vd_contact));
    theta = angle*(1-tanh(40*normal_distance));
    u = cross(n,vd_contact);
    % Compute rotation matrix to progressively rotate normal vector with 
    % projected dynamics based on the distance to the surface
    R = eye(3);
    if(norm(u)>eps)
        u = u/norm(u);
        K = [0 -u(3) u(2) ; u(3) 0 -u(1) ; -u(2) u(1) 0 ];
        R = eye(3)+sin(theta)*K+(1-cos(theta))*K*K;
    end
    % Compute final desired dynamics 
    vd = R*(vt*n);
end