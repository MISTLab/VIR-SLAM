# VIR-SLAM

## VIR-SLAM: Visual, Inertial, and Ranging SLAM for Single and Multi-Robot Systems

 Monocular cameras coupled with inertial measurements generally give high performance visual inertial odometry. However, drift can be significant with long trajectories, especially when the environment is visually challenging. In this paper, we propose a system that leverages Ultra-WideBand (UWB) ranging with one static anchor placed in the environment to correct the accumulated error whenever the anchor is visible. We also use this setup for collaborative SLAM: different robots use mutual ranging (when available) and the common anchor to estimate the transformation between each other, facilitating map fusion. Our system consists of two modules: a double layer ranging, visual, and inertial odometry for single robots, and a transformation estimation module for collaborative SLAM. We test our system on public datasets by simulating UWB measurements as well as on real robots in different environments. Experiments validate our system and show our method can outperform pure visual-inertial odometry by more than 20\%, and in visually challenging environments, our method works even when the visual-inertial pipeline has significant drift. Furthermore, we can compute the inter-robot transformation matrices for collaborative SLAM at almost no extra computation cost.

 

## Acknowledgements
This project is developed based on [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono).