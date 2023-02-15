function seDStoYAML(priors, means, covariances, attractor, in, out, file)
% SEDSTOYAML  Export a seDS model as YAML file.
%   This function exports a seDS model to a human readable YAML file such
%   that it can be imported in other programming languages.
% 
%   Inputs -----------------------------------------------------------------
%     o priors:      1 x K array representing the prior probabilities of the
%                    K GMM components.
%     o means:       D x K array representing the centers of the K GMM components.
%     o covariances: D x D x K array representing the covariance matrices 
%                    of the K GMM components.
%     o attractor:   1 x P array representing the attractor of the training
%                    data
%     o in:          1 x P array representing the dimensions to consider as
%                    inputs.
%     o out:         1 x Q array representing the dimensions to consider as
%                    outputs (D=P+Q).
%     o file:        Desired filename (relative or absolute).
%
%   January 2021, Dominic Reber, dominic.reber@epfl.ch

arguments
    priors (1,:) double
    means (:,:) double
    covariances (:,:,:) double
    attractor (1,:) double
    in (1,:) int64
    out (1,:) int64
    file char
end

nb_components = length(priors);
if size(means, 2) ~= nb_components
    error(['The number of rows of the means array has to correspond '...
        'to the length of the priors vector.']);
end
if size(covariances, 3) ~= nb_components
    error(['The third dimension of the covariances array has to correspond '...
        'to the length of the priors vector.']);
end

dim = size(means, 1);
if size(covariances, 1) ~= dim || size(covariances, 2) ~= dim
    error(['The first and second dimensions of the covariances array ' ...
        'have to correspond to the number of rows of the means array.']);
end

if length(in) + length(out) ~= dim
    error(['The length of the input and output indices is incorrect '...
        'and does not add up to the dimensionality']);
end

if length(unique([in, out])) < dim
    error('The input indices and output indices overlap.');
end

if any(in < 1) || any(in > dim) || length(unique(in)) < length(in)
    error('The input indices are not unique or out of range.')
end

if any(out < 1) || any(out > dim) || length(unique(out)) < length(out)
    error('The output indices are not unique or out of range.')
end

if length(attractor) ~= length(in)
    error('The length of the attractor and input indices need to correspond')
end

file_id = fopen(file, 'w');
fprintf(file_id, 'dimensionality: %i\n', dim);
fprintf(file_id, 'nb_components: %i\n', nb_components);

fprintf(file_id, 'input_indices: ');
print_array(in - 1, file_id, '%i');

fprintf(file_id, 'output_indices: ');
print_array(out - 1, file_id, '%i');

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

fclose(file_id);

function print_array(array, file_id, format)
    fprintf(file_id, '[ ');
    for i=1:length(array)-1
        fprintf(file_id, [format ', '], array(i));
    end
    fprintf(file_id, [format ' ]\n'], array(end));
end
end
