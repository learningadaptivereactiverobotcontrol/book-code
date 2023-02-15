function mm = minmax(x)
%MINMAX Ranges of matrix rows.
%
%  <a href="matlab:doc minmax">minmax</a>(X) takes a single matrix (or cell array of matrices) and returns
%  an Nx2 value of min and max values for each row of the matrix (or row
%  of matrices).
%
%  Here min-max is calculated for a random matrix:
%
%    x = <a href="matlab:doc rands">rands</a>(4,5)
%    mm = <a href="matlab:doc minmax">minmax</a>(x)
%
%  Here min-max is calculated for random neural network cell data.
%
%    x = <a href="matlab:doc nndata">nndata</a>([1;2],3,4)
%    mm = <a href="matlab:doc minmax">minmax</a>(x)
%
%  See also NNDATA, NNSIZE.

% Mark Beale, 11-31-97
% Copyright 1992-2010 The MathWorks, Inc.

% Check and reformat
if nargin < 1, error(message('nnet:Args:NotEnough')); end
wasMatrix = ~iscell(x);
x = nntype.data('format',x,'Data');

% Min-Max
mm = nnfast.minmax(x);

% Reformat
if wasMatrix, mm = mm{1}; end