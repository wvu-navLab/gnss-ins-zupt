# Robust Incremental State Estimation through Covariance Adaptation

## Overview

This repository contains the software release for "Robust Incremental State Estimation through Covariance Adaptation". The objective of the software release is described through the associated abstract. For a more thorough discussion, please see the [description document](https://github.com/wvu-navLab/ICE/blob/master/description.pdf).

<br/>
<br/>


> Recent advances in the fields of robotics and automation have spurred significant interest in robust state estimation. To enable robust state estimation, several methodologies have been proposed. One such technique, which has shown promising performance, is the concept of iteratively estimating a Gaussian Mixture Model (GMM), based upon the state estimation residuals, to characterize the measurement uncertainty model. Through this iterative process, the measurement uncertainty model is more accurately characterized, which enables robust state estimation through the appropriate de-weighting of erroneous observations. This approach, however, has traditionally required a batch estimation framework to enable the estimation of the measurement uncertainty model, which is not advantageous to robotic applications. In this paper, we propose an efficient, incremental  extension to the measurement uncertainty model estimation paradigm. The incremental covariance estimation (ICE) approach, as detailed within this paper, is evaluated on several collected data sets, where it is shown to provide a significant increase in localization accuracy when compared to other state-of-the-art robust, incremental estimation algorithms. 


<br/>
<br/>

If you utilize this software for an academic purpose, please consider using the following citation:
```
@article{watson2020robust, 
         author={R. M. {Watson} and J. N. {Gross} and C. N. {Taylor} and R. C. {Leishman}},
	 journal={IEEE Robotics and Automation Letters},
	 title={Robust Incremental State Estimation Through Covariance Adaptation},
	 year={2020},
	 volume={5},
	 number={2},
	 pages={3737-3744}
}
```

<br/>
<br/>


<br/>
<br/>

This software benefits from several open-source software packages.
* [*Georgia Tech Smoothing And Mapping (GTSAM)*](https://bitbucket.org/gtborg/gtsam/src/develop/) -- contains factor graph based state estimation algorithms
	* GTSAM was updated for GNSS signal processing within
	    *  [*PPP-BayesTree*](https://github.com/wvu-navLab/PPP-BayesTree) -- contains pseudorange and carrier-phase factors
	    *  [*RobustGNSS*](https://github.com/wvu-navLab/RobustGNSS) -- contains robust GNSS models
* [*GPS Toolkit (GPSTk)*](http://www.gpstk.org/bin/view/Documentation/WebHome) -- contains GNSS observation modeling tools
* [*libcluster*](https://github.com/dsteinberg/libcluster) -- contains variatinal clustering algorithms


<br/>
<br/>
<br/>

<!--
If you utilze this software for an academic purpose, please consider using the following citation:
-->
<!--
```
@article{ watson2019robust,
        title={Robust Incremental State Estimation through Covariance Adaptation},
        author={Watson, Ryan M and Gross, Jason N and Taylor, Clark N and Leishman, Robert C},
        journal={arXiv preprint},
        year={2019}
       }
```
-->

<br/>
<br/>

## How to Install


### 1) Requirements/Recommendations

#### Required
* Boost -->  ```` sudo apt-get install libboost-all-dev ````
	* Note: this has only been tested with boost --version 1.58 
* CMake -->  ```` sudo apt-get install cmake ````
* OpenMP --> ```` sudo apt install libomp-dev ````


### 2) Clone repository to local machine  
````bash

git clone https://github.com/wvu-navLab/ICE.git

````

### 3) Build

````bash

cd ICE
./build.sh

````

### 4) Test

Let's run a test on dataset 1 with low-quality observations. To change the utilized dataset, use one of the following arguments {1,2,3}_{lq,hq} after the *run_me.sh* command. (e.g., ./run_me.sh 3_hq).


````bash
cd examples
chmod +x run_me.sh
./run_me.sh 1_lq
````

This will write all of the generated results to the *test* directory (*../test*). All of the 'truth' data, utilized for validation, is housed in the *~/ICE/data/truth/* directory

For this specific example, the table printed to the screen should match the one provided below.

````bash
type    med_    mean_   var_    max_
L2      2.57    2.52    1.99    10.78
DCS     0.64    0.99    0.95    9.71
MM      1.63    1.66    1.10    10.06
ICE     0.57    0.72    0.46    13.19
````
