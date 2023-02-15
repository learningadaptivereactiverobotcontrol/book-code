function lpvDStoYAML(priors, means, covariances, A, b, attractor, file)
% LPVDSTOYAML  Export a lpvDS model as YAML file.
%   This function exports a lpvDS model to a human readable YAML file such
%   that it can be imported in other programming languages.
%   x_dot = sum_{k=1}^K gamma_k(x)*(A_k*x+b_k)
% 
%   Inputs -----------------------------------------------------------------
%     o priors:      1 x K array representing the prior probabilities of the
%                    K GMM components.
%     o means:       D x K array representing the centers of the K GMM components.
%     o covariances: D x D x K array representing the covariance matrices 
%                    of the K GMM components.
%     o A            D x D x K array representing the A_k matrices of the K
%                    LTI systems in the lpv formulation
%     o b            D x K array representing the b_k matrices of the K
%                    LTI systems in the lpv formulation
%     o attractor:   1 x D array representing the attractor of the training
%                    data
%     o file:        Desired filename (relative or absolute).
%
%   January 2021, Dominic Reber, dominic.reber@epfl.ch

arguments
    priors (1,:) double
    means (:,:) double
    covariances (:,:,:) double
    A (:,:,:) double
    b (:,:) double
    attractor (1,:) double
    file char
end

nb_components = length(priors);
if size(means, 2) ~= nb_components
    error(['The number of columns of the means array has to correspond '...
        'to the length of the priors vector.']);
end
if size(covariances, 3) ~= nb_components
    error(['The third dimension of the covariances array has to correspond '...
        'to the length of the priors vector.']);
end
if size(A, 3) ~= nb_components
    error(['The third dimension of the A array has to correspond '...
        'to the length of the priors vector.']);
end
if size(b, 2) ~= nb_components
    error(['The number of columns of the b array has to correspond '...
        'to the length of the priors vector.']);
end

dim = size(means, 1);
if size(covariances, 1) ~= dim || size(covariances, 2) ~= dim
    error(['The first and second dimensions of the covariances array ' ...
        'have to correspond to the number of rows of the means array.']);
end
if size(A, 1) ~= dim || size(A, 2) ~= dim
    error(['The first and second dimensions of the A array ' ...
        'have to correspond to the number of rows of the means array.']);
end
if size(b, 1) ~= dim
    error(['The number of rows of the b array ' ...
        'have to correspond to the number of rows of the means array.']);
end

if length(attractor) ~= dim
    error(['The size of the attractor ' ...
        'has to correspond to the number of rows of the means array.']);
end

file_id = fopen(file, 'w');
fprintf(file_id, 'dimensionality: %i\n', dim);
fprintf(file_id, 'nb_components: %i\n', nb_components);

fprintf(file_id, 'training_set_attractor: ');
print_array(attractor, file_id, '%6f');

fprintf(file_id, 'priors: ');
print_array(priors, file_id, '%6f');

fprintf(file_id, 'means:\n');
for c=1:nb_components
    fprintf(file_id, '  - ');
    print_array(means(:,c), file_id, '%6f');
end

fprintf(file_id, 'covariances:\n');
for c=1:nb_components
    fprintf(file_id, '  - ');
    print_array(reshape(covariances(:,:,c), 1, []), file_id, '%6f');
end

fprintf(file_id, 'A:\n');
for c=1:nb_components
    fprintf(file_id, '  - ');
    print_array(reshape(A(:,:,c), 1, []), file_id, '%6f');
end

fprintf(file_id, 'b:\n');
for c=1:nb_components
    fprintf(file_id, '  - ');
    print_array(b(:,c), file_id, '%6f');
end

fclose(file_id);

function print_array(array, file_id, format)
    fprintf(file_id, '[ ');
    for i=1:length(array)-1
        fprintf(file_id, [format ', '], array(i));
    end
    fprintf(file_id, [format ' ]\n'], array(end));
end
end
