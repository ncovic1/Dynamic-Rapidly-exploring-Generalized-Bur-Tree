# DRGBT (Dynamic Rapidly-exploring Generalized Bur Tree) algorithm
DRGBT (Dynamic Rapidly-exploring Generalized Bur Tree) algorithm is intended for motion planning in dynamic environments. The main idea behind DRGBT lies in a so-called adaptive horizon, consisting of a set of prospective target nodes that belong to a predefined C-space path, which originates from the current node. Each node is assigned a weight that depends on relative distances and captured changes in the environment. The algorithm continuously uses a suitable horizon assessment to decide when to trigger the replanning procedure.

For more information, please see the full paper at https://www.researchgate.net/publication/354927105_Path_Planning_for_Robotic_Manipulators_in_Dynamic_Environments_Using_Distance_Information

# RGBMT* (Rapidly-exploring Generalized Bur Multi-Tree star) algorithm
RGBMT* (Rapidly-exploring Generalized Bur Multi-Tree star) algorithm is intended for asymptotically optimal motion planning for robotic manipulators in static environments. The main idea is the generation of local/extra trees rooted in random configurations, beside two main trees rooted in initial and goal configurations. Each local tree is expanded towards all other trees via bur of free configuration space (C-space). Each node is assigned a cost-to-come value, which is then used to optimally connect (if possible) all nodes from local trees to a single main tree according to Bellman’s principle of optimality. The algorithm is provably asymptotically optimal, i.e., such that the cost of the returned solution converges almost-surely to the optimum. Main and local trees are aptly grown in order to prevent exploring C-space in regions which are unlikely to yield solutions. A comprehensive simulation study is performed analyzing runtimes and convergence rate, where RGBMT* is compared to state-of-the-art algorithms. Obtained results indicate some promising features of the proposed method.

For more information, please see the full paper at https://www.researchgate.net/publication/373578135_Asymptotically_Optimal_Path_Planning_for_Robotic_Manipulators_Multi-Directional_Multi-Tree_Approach#fullTextFileContent

# C++ implementation
For C++ implementation of DRGBT and RGBMT* algorithms, please refer to https://github.com/roboticsETF/RPMPLv2.git.

For C++ implementation of DRGBT and RGBMT* algorithms on a real robot xArm6, please refer to https://github.com/roboticsETF/xarm6-etf-lab.git.
