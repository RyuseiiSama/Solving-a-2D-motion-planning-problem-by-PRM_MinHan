# Solving a 2D Motion planning problem by PRM + Path searching + Post processing By MinHan

Use the following parameters to edit the bounds and parameters of the PRM

N = 10000  (Number of random nodes)  
REFINE = 10 (Repetitions for post processing)  
MIN_R = 3 (Radius to search for neighbours)   
X_SIZE = 20 (Bounds of environment)  
Y_SIZE = 20 (Bounds of environment)  
N_OBSTACLES = 6 (Number of obstacles)  

PRM was created as a class, with methods to generate the PRM (run_prm) and to find the shortest path (find_shortest_path)  
Edgechecker was also created as a class to check if the edge between 2 points will intersect with obstacles. This class takes in the TriangularObstacle() object to store it as an attribute, which will then be utilised to check the intersection between edges and obstacles.  

EXAMPLES:  
N_OBSTACLES = 5  
![msg631197280-690834](https://github.com/RyuseiiSama/Solving-a-2D-motion-planning-problem-by-PRM_MinHan/assets/84442508/13dfd6c2-281e-4c0e-b789-5043a8ca55b2)  


N_OBSTACLES = 6  
![msg631197280-690862](https://github.com/RyuseiiSama/Solving-a-2D-motion-planning-problem-by-PRM_MinHan/assets/84442508/bd600b2b-cb0d-497d-a947-7bbb865bc114)


