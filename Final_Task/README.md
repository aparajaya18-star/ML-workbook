# Final Task - MaptheMap      
### Approach:     
* Orthomosaicking - PBIM (Patch Based Image Mosaicking) 
* Dataset:      
    - Cloud cover       - NA
    - Lens Distortion   - NA
    - Blur              - Somewhat
    - Size              - 10 to 40 images
* Current Pipeline:
    - Find features using ORB
    - Matching features using BFmatcher    
    - Sorting matches using Lowe's ratio
    - RANSAC for geometric verification
    - Display similarity graph using NetworkX  
    - Use Community detection to group images into regions  
    - Use MaxST to find stitching order  
    - Warp images using homography and Global canvas pre-computation (using corner projection)
    - Blend images using feather blending
    - Repeat the same for both regional and global 
* Changes to be made:
    - Improve Feature alignment(Check homography and transform) [Really Important!!!]
    - Maybe different pipeline(Order of region division and image stitching + no of times it's grouped)
    - Better Scale Invariance
    - Set min parameters, sometimes group is too small in communities
    - Is distinct community grouping really necessary for my pipeline since we are grouping the same map(either diff pipeline with multiple maps or no community division allow overlap in regions for singular map)
    - Have an outlier class/group for images that don't fit into any map since dataset might be noisy

### Concepts/Notes  
* BFMatcher: Compares every descriptor in 2 images and finds best match (Using Hamming distance for ORB [descriptor type = Binary])    
* Lowe's Ratio: Checks if best match is better than second best or if both are similar i.e. best match is just best among bad matches (dist_match1/dist_match2 < Threshold -> 0.7 to 0. 8)    
* RANSAC (RANdom SAmple Consensus): Finds geometrically consistent matches. Maps points from one planar image to another.
    - Randomly pick a small subset of matches (minimum 4 for homography)
    - Compute a homography H
    - Test ALL matches:
        - Project points using H
        - Check error
    - Count inliers (good matches that fit H)
    - Repeat many times
    - Pick the H with maximum inliers
* Maximum Spanning Tree (MaxST):    
    - A spanning tree is a subset of Graph G, such that all the vertices are connected using minimum possible number of edges.
    - ST has no cycles (evenly divide into regions)
    - MaxST is ST with max weight ie ST with max no of outliers
* BFS(Breadth First Search): Method to traverse a Tree such that all nodes present in the same level are traversed completely before traversing the next level.