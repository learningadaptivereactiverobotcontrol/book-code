
function h = plot_gaussian_2d(fig,Mu,Sigma,offset)
figure(fig);
nbSeg = 60;
tt = linspace(-pi,pi,nbSeg)';
s = sqrtm(Sigma);
segs = [cos(tt) sin(tt)] * real(2.*s) + repmat(Mu'+offset',nbSeg,1);
h = patch(segs(:,1), segs(:,2), [0.1 0.1 0.1], 'lineWidth', 1, 'EdgeColor', 'k', 'FaceColor', 'None');
end