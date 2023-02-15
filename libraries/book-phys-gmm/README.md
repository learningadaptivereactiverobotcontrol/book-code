# phys-gmm
This package contains the inference implementation (Gibbs Sampler) for the "Physically Consistent Bayesian Non-Parametric Mixture Model" (PC-GMM) proposed in [1]. This approach is used to **automatically** (no model selection!) fit GMM on **trajectory data** while ensuring that the points clustered in each Gaussian represent/follow a linear dynamics model, in other words the points assigned to each Gaussian should be close in "position"-space and follow the same direction in "velocity"-space.

<p align="center">
  <img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/Lshape_pcgmm.png" width="220">
<img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/Ashape_pcgmm.png" width="220"><img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/Sshape_pcgmm.png" width="220"><img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/Ashape_pcgmm.png" width="220">
</>

### Dependencies
- **[Necessary]** [LightSpeed Matlab Toolbox](https://github.com/tminka/lightspeed): Tom Minka's library which includes highly optimized versions of mathematical functions. Please download/install it in the ```thirdparty/lightspeed``` directory.
- **[Optional]** [crp](http://www.gatsby.ucl.ac.uk/~fwood/code.html): Frank Wood's Infinite Gaussian Mixture Model (IGMM) / Dirichlet process (DP) mixture model Matlab implementation. If you want to use/test ``option 2`` of the given GMM fitting function you must download/install it in the ```thirdparty/crp``` directory. If you are not interested in this, it is not necessary.

### Instructions and Content
This package offers the physically-consistent GMM fitting approach, as well as examples and code for fitting GMM with standard EM approach and the Bayesian non-parametric approach following the Chinese Restaurant Process construction through the ```[Mu, Priors, Sigma] = fit_gmm()``` function by filling its options as follows:
```Matlab
%%%%%%%%%%%%%%%%%% GMM Estimation Algorithm %%%%%%%%%%%%%%%%%%%%%%
% 0: Physically-Consistent Non-Parametric (Collapsed Gibbs Sampler)
% 1: GMM-EM Model Selection via BIC
% 2: CRP-GMM (Collapsed Gibbs Sampler)
est_options = [];
est_options.type             = 1;   % GMM Estimation Algorithm Type   

% If algo 1 selected:
est_options.maxK             = 15;  % Maximum Gaussians for Type 1
est_options.fixed_K          = [];  % Fix K and estimate with EM for Type 1

% If algo 0 or 2 selected:
est_options.samplerIter      = 20;  % Maximum Sampler Iterations
                                    % For type 0: 20-50 iter is sufficient
                                    % For type 2: >100 iter are needed
                                    
est_options.do_plots         = 1;   % Plot Estimation Statistics
est_options.sub_sample       = 1;   % Size of sub-sampling of trajectories

% Metric Hyper-parameters
est_options.estimate_l       = 1;   % Estimate the lengthscale, if set to 1
est_options.l_sensitivity    = 2;   % lengthscale sensitivity [1-10->>100]
                                    % Default value is set to '2' as in the
                                    % paper, for very messy, close to
                                    % self-interescting trajectories, we
                                    % recommend a higher value
est_options.length_scale     = [];  % if estimate_l=0 you can define your own
                                    % l, when setting l=0 only
                                    % directionality is taken into account

% Fit GMM to Trajectory Data
[Priors, Mu, Sigma] = fit_gmm(Xi_ref, Xi_dot_ref, est_options);
```
To test the function, you can either draw 2D data by running the demo script:
```
demo_drawData.m
```
or you can load pre-drawn 2D or real 3D datasets with the following script:
```
demo_loadData.m
```
### Example Datasets
These examples + more datasets are provided in ```
./datasets``` folder. Following we show some **notably challenging trajectory datasets** that cannot be correctly clustered with the standard GMM either through MLE via the EM algorithm **(center)** or the CRP-GMM via collapsed Gibbs sampling **(right)**, but are correctly clustered through our proposed approach **(left)**.

-  **GMM fit on 2D Concentric Circles Dataset**  
<p align="center">
  <img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/concentric-data.png" width="220">
<img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/concentric-pcgmm.png" width="220"><img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/concentric-emgmm.png" width="220"><img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/concentric-crpgmm.png" width="220">
</>

-  **GMM fit on 2D Opposing Motions (Different Targets) Dataset**  
<p align="center">
  <img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/opposing-data.png" width="220">
<img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/opposing-pcgmm.png" width="220"><img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/opposing-emgmm.png" width="220"><img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/opposing-crpgmm.png" width="220">
</>

-  **GMM fit on 2D Multiple Motions (Different Targets) Dataset**  
<p align="center">
  <img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/multiple-data.png" width="220">
<img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/multiple-pcgmm.png" width="220"><img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/multiple-emgmm.png" width="220"><img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/multiple-crpgmm.png" width="220">
</>
  
-  **GMM fit on 2D Multiple Motions (Same Target) Dataset**  
<p align="center">
  <img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/multi-behavior_data.png" width="220">
<img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/multi-behavior_pcgmm.png" width="220"><img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/multi-behavior_emgmm.png" width="220"><img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/multi-behavior_crpgmm.png" width="220">
</>

- **GMM fit on 2D Messy Snake Dataset**  
<p align="center">
  <img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/messy-snake-data.png" width="220">
<img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/messy-snake-pcgmm.png" width="220"><img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/messy-snake-emgmm.png" width="220"><img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/messy-snake-crpgmm.png" width="220">
</>  

### Estimation Statistics
By setting ```est_options.do_plots= 1;``` the function will plot the corresponding estimation statistics for each algorithm. 
- For the PC-GMM we show the values of the posterior distribution p(C|...) and the estimated clusters at each iteration:  
<p align="center">
  <img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/example-PCGMM-stats.png" width="540">
</>  

- For the EM-based Model Selection approach we show the BIC curve computed with increasing K=1,...,15. The 1st and 2nd order numerical derivative of this curve is also plotted and the 'optimal' K is selected as the inflection point:  
<p align="center">
  <img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/example-BIC.png" width="540">
</>  

- For the CRP-GMM we show the values of the posterior distribution p(Z|...) and the estimated clusters at each iteration:  
<p align="center">
  <img src="https://github.com/nbfigueroa/phys-gmm/blob/master/figs/example-CRP-stats.png" width="540">
</>  
  
### Known Issues and Limitations  
The only limitation of the proposed approach is its computational complexity. Although the collapsed gibbs sampler needs significantly less iterations than that for the CRP-GMM, it is computationally taxing as all ``customer assignments`` have to be sampled in each iteration. Specifically, the first iteration is slow as the sampler begins with N clusters....more discussion here.

### Usage
Such physically-consistent clustering is particularly useful for learning Dynamical Systems (DS) that are formulated as Linear Parameter Varying (LPV) systems, as introduced in [1]. To use this approach for DS learning, download the [ds-opt](https://github.com/nbfigueroa/ds-opt.git) package.   

**References**    
> [1] Figueroa, N. and Billard, A. (2018) "A Physically-Consistent Bayesian Non-Parametric Mixture Model for Dynamical System Learning". In Proceedings of the 2nd Conference on Robot Learning (CoRL). 

**Contact**: [Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387) (nadia.figueroafernandez AT epfl dot ch)

**Acknowledgments**
This work was supported by the EU project [Cogimon](https://cogimon.eu/cognitive-interaction-motion-cogimon) H2020-ICT-23-2014.
