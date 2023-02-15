function sem=SEM_calc(vect, CI)
% SEM_calc - standard error of the mean, confidence interval
%
% function sem = NBP.SEM_calc(vect, CI) 
%
% Purpose
% Calculate the standard error the mean to a given confidence
% interval (CI). Note that nans do not contribute to the
% calculation of the sample size and are ignored for the SD
% calculation. Output of this function has been checked against
% known working code written in R. 
%
% Inputs
% - vect: A vector upon which the SEM will be calculated. Note that
%         if vect is a matrix then we calculate one SEM for each
%         column. 
%
% - CI [optional]: a p value for a different 2-tailed interval. e.g. 0.01
%   This is a 2-tailed interval.
%
% Outputs
% sem - the standard error of the mean. So to plot the interval it's mu-sem
% to mu+sem. 
%
% Example - plot a 1% interval [rather than the default %5]
% r=randn(1,30);
% S=SEM_calc(r,0.01);
% hist(r)
% hold on
% plot(mean(r), mean(ylim),'r*')
% plot([mean(r)-S,mean(r)+S], [mean(ylim),mean(ylim)],'r-')
% hold off
%
%
% Rob Campbell 
%
%
% Also see - tInterval_Calc, norminv

narginchk(1,2)

if isvector(vect)
  vect=vect(:);
end

% Define an anonymous function to take over from norminv, which is in the Stats ToolBox
myNormInv = @(x) -sqrt(2)*erfcinv(2*x);

if nargin==1
  stdCI = 1.96 ; 
elseif nargin==2
  CI = CI/2 ; %Convert to 2-tail
  stdCI = abs(myNormInv(CI)); % This is the same as doing: abs(norminv(CI,0,1)) 
end

for ii=1:size(vect,2)
    f =  find(~isnan(vect(:,ii)));
    sem(ii) = ( std(vect(f,ii))  ./ sqrt(length(f)) ) * stdCI ;
end
