function [ d_nth_x ] = sgolay_time_derivatives(x, dt, nth_order, ...
                                                n_polynomial, window_size)
%   SGOLAY_TIME_DERIVATIVES Computes Savitzky Golay filter with a moving
%   window and derivatives up to the n-th order 
%   x :             input data size (time, dimension)
%   dt :            sample time
%   nth_order :     max order of the derivatives 
%   n_polynomial :  Order of polynomial fit
%   window_size :   Window length for the filter

%   # Authors: Jose Medina
%   # EPFL, LASA laboratory
%   # Email: jrmout@gmail.com

    if (size(x,1) < window_size) 
        error(['The window size (' num2str(window_size) ...
            ') is greater than the data length (' num2str(size(x,1)) ...
            '). Choose a smaller window size...'] );
    end
    
    [~,g] = sgolay(n_polynomial,window_size);   % Calculate S-G coefficients
    for dim=1:size(x,2)
        y = x(:,dim)';
        half_win  = ((window_size+1)/2) -1; % half window size
        ysize = size(y,2); %number of data points
        for n = (window_size+1)/2 : ysize-(window_size+1)/2,
            for dx_order = 0:nth_order
                  d_nth_x(n,dim,dx_order+1) = dot(y(n-half_win : n+half_win), ...
                      factorial(dx_order)/dt^dx_order * g(:,dx_order+1));
            end
        end
    end

    % Remove the data at the beginning due to the window
    crop_size = (window_size+1)/2;
    d_nth_x = d_nth_x(crop_size:end, :, :);
end

