# Final Task - MaptheMap        
* Orthomosaicking - PBIM (Patch Based Image Mosaicking) 
* BFMatcher: Compares every descriptor in 2 images and finds best match (Using Hamming distance for ORB [descriptor type = Binary])    
* Lowe's Ratio: Checks if best match is better than second best or both are similar -> best match is just best among bad matches (dist_match2/dist_match1 < Threshold)    
* RANSAC: Find geometrically consistent matches
    - Randomly pick a small subset of matches (minimum 4 for homography)
    - Compute a homography H
    - Test ALL matches:
        - Project points using H
        - Check error
    - Count inliers (good matches that fit H)
    - Repeat many times
    - Pick the H with maximum inliers
* Dataset:      
    - Cloud cover       - NA
    - Lens Distortion   - NA
    - Blur              - Somewhat
    - Size              - 10 to 40 images
* Current Pipeline:
    - Find features using ORB
    - Matching features using BFmatcher    
    - Sorting matches using Lowe's ratio
    - Display similarity graph using NetworkX