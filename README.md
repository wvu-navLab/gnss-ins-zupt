## GNSS-INS-ZUPT

This repository contains the software release for "ZUPT aided GNSS Factor Graph with Inertial Navigation Integration for Wheeled Robots".

<br/>

> In this work, we demonstrate the importance of zero velocity information for GNSS based navigation. The effectiveness of using the zero velocity information with ZUPT for inertial navigation applications have been shown in the literature. Here we leverage this information and add it as a position constraint in a GNSS factor graph. We also compare its performance to a GNSS/INS coupled factor graph. We tested our ZUPT aided factor graph method on three datasets and compared it with the GNSS-only factor graph.


<p align="center">
<img alt="Architecture" src="doc/factorgraph_zuptL2.PNG" width="1200">
</p>

<br/>

This software benefits from several open-source software packages.
* [*Georgia Tech Smoothing And Mapping (GTSAM)*](https://bitbucket.org/gtborg/gtsam/src/develop/) -- contains factor graph based state estimation algorithms
	* GTSAM was updated for GNSS signal processing within
	    *  [*PPP-BayesTree*](https://github.com/wvu-navLab/PPP-BayesTree) -- contains pseudorange and carrier-phase factors
	    *  [*RobustGNSS*](https://github.com/wvu-navLab/RobustGNSS) -- contains robust GNSS models
* [*GPS Toolkit (GPSTk)*](http://www.gpstk.org/bin/view/Documentation/WebHome) -- contains GNSS observation modeling tools





## How to Install


### 1) Requirements/Recommendations

#### Required
* Boost -->  ```` sudo apt-get install libboost-all-dev ````
	* Note: this has only been tested with boost --version 1.58
* CMake -->  ```` sudo apt-get install cmake ````
* OpenMP --> ```` sudo apt install libomp-dev ````


### 2) Clone repository to local machine  
````bash

git clone the repository

````

### 3) Build

````bash

cd ICE
./build.sh

````

### 4) Test


````bash
cd examples
chmod +x run_mel2.sh
./run_mel2.sh
````

### Note
If you see an error as 

````  error while loading shared libraries: libmetis.so: cannot open shared object file: No such file or directory ```` 

Simply, paste this command to your bashrc:

```` export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/*FULL_PATH_OF_REPO*/gnss-ins-zupt/GNSS_INS_ZUPT/lib ```` 


