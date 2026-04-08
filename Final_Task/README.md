# Final Task - MaptheMap        
* Orthomosaicking - PBIM (Patch Based Image Mosaicking) 
* Any issues with dataset?  
    - Cloud cover       - NA
    - Lens Distortion   - NA
    - Blur              - Somewhat
* Current Pipeline:
    - Find features using ORB
    - Matching features using BFmatcher
    - Sorting matches using Lowe's ratio
    - Display similarity graph using NetworkX