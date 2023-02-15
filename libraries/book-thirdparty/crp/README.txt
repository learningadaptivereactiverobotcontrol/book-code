AUTHOR INFORMATION: Frank Wood fwood@cs.brown.edu, 401-351-4222
                    http://www.cs.brown.edu/~fwood/

REQUESTS OF USERS: Please don't use this code for commercial applications
without at least first seeking my permission.  Feedback and bug fixes are
welcome.

OVERVIEW: Contained herein is an implementation of an infinite Gaussian mixture
model, similar to Rasmussen's NIPS 2000 paper, "The Infinite Gaussian Mixture
Model" and my own paper, "A Non-Parametric Bayesian Approach to Spike Sorting",
submitted to EMBS 2006.  This is a fully multivariate implementation, with a
hybrid Gibbs/Metropolis sampler for model parameters.  A small test script is
included.  This code is covered by Brown's copyright policy, a copy of which is
included in each of the matlab files.

REQUIREMENTS: Matlab + statistics toolbox (for iwishrnd) and a fair amount of
time depending on your requirements.

USAGE:  Start matlab, add <install_dir>"/distributions" and
<install_dir>"./utilities" to your path.  cd into <install_dir>"/crp" and type
"test_sampler".  A Gaussian mixture model with 4 components will be created,
samples will be drawn from this model, then the CRP posterior sampler will run
with graphics turned on.  This will show sampler traces and current clustering
for the sampler as it is running.  Substituting your own data in place of the
training data will yield similar, though probably slower results.

CAVEATS:  Of the supporting code, of which only a small portion is used, only a
slightly larger portion is tested.  I am fairly comfortable with this CRP code,
but using any portion of the supporting code (like the gaussian mixture model
code for instance) in an unsupported way (which is any way not exactly like how
it is used here may cause problems).
