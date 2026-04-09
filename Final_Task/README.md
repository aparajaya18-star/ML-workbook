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
* Minimal Spanning Tree (MST): 