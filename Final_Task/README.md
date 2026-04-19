# Final Task - MaptheMap      
### Approach:     
* Orthomosaicking - PBIM (Patch Based Image Mosaicking) 
* Dataset:      
    - Cloud cover       - NA
    - Lens Distortion   - NA
    - Blur              - Somewhat
    - Size              - 10 to 40 images
* Recent attempts: Tried sliders as well as Image thresholding for road segmentation but both required a lot of tuning each time a different image was tested (Bad Generalization)
* Current Pipeline:
    - Image Stitching:
        - Find features using SIFT
        - Matching features using BFmatcher    
        - Sorting matches using Lowe's ratio
        - RANSAC for geometric verification
        - Display similarity graph using NetworkX  
        - Use Community detection to group images into regions  
        - Use MaxST to find stitching order  
        - Warp images using homography and Global canvas pre-computation (using corner projection)
        - Blend images using feather blending
        - Repeat the same for both regional and global  
    - Converting to 2D map:
        - Check if map needs orthorectification (Check amount of black pixels near boundary)
        - Detect warped boundary of the image
        - Apply homography to map image such that warped rectangle becomes true rectangle
        - Recheck orthorectification and return original image if unsuccessful
    - Road Segmentation:     

* Feature vector per patch — for each pixel, extract features from a small patch around it:
* HSV mean and std (6 values)
LBP histogram for texture (typically 10-26 values depending on settings)
Sobel gradient magnitude mean and std (2 values)
* Labels from DeepGlobe — each training image has a _mask.png. Roads are labeled as white (255,255,255), everything else black. You convert this to binary 0/1.
* Training — sample balanced patches from DeepGlobe images (equal road/non-road), extract features, train Random Forest.
* Inference — slide over your stitched map, extract same features per patch, predict road/non-road.

### Researched/Possible solutions:
* Spatially Varying Warping
    - Methods: APAP,SPHP and AANAP
    - useful when there is less parallax 
    - Instead of global transformation multiple homographies are calculated locally within an image
    - AANAP: [https://www.cv-foundation.org/openaccess/content_cvpr_2015/ext/1B_005_ext.pdf]
    
* Seam Cutting
    - Methods: seam-guided local alignment (SEAGULL), perception-based seam cutting method
    - useful when there is greater parallax
    - Best seam is found for better spatial preservation after warping  
    - SEAGULL: [https://link.springer.com/chapter/10.1007/978-3-319-46487-9_23]
    - Perception-based seam cutting: [https://link.springer.com/article/10.1007/s11760-018-1241-9]

### Changes to be made:
- Maybe different pipeline(Order of region division and image stitching + no of times it's grouped)
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
* LBP algorithm:
    -   Choose a pixel in the image and select its neighboring pixels in a circular or rectangular region around it.
    -   Take the threshold (intensity of the selected pixel, here it is 50).
    -   Go through every neighboring pixel and check whether its intensity is greater than or less than the threshold.
    Assign 1 to the neighboring pixel, if the intensity of the neighboring pixel is greater than the threshold.
    Assign 0 to the neighboring pixel, if the intensity of the neighboring pixel is less than the threshold.
    -   Combine the binary values for all neighboring pixels to obtain a binary code for the central pixel (Anti-clockwise, starting from the top left corner), and convert it to a decimal value.
    -   Repeat steps 1–4 for each pixel in the image to obtain a binary code for each pixel.
These LBP values are used to construct the histogram. By constructing a histogram of the LBP patterns, we can capture the frequency of occurrence of different texture patterns in the image. This histogram can then be used as a feature vector for texture classification tasks.