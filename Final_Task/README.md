# Final Task - MaptheMap      

## Overview
An end-to-end pipeline for stitching fragmented aerial/satellite images into 
seamless 2D maps, segmenting roads, and computing shortest paths.

## Setup
pip install opencv-python networkx scikit-image torch torchvision 
scikit-learn matplotlib scikit-image albumentations

## How to Run
1. Place input images in Datasets/dataset_6/
2. Run PBIM.ipynb — produces topdown_image.png
3. Run Unet_segmentation.ipynb — produces segmented_mask.png
4. Run astar.ipynb — set START/END coordinates at top of notebook

## Pipeline
### Phase 1: Image Grouping
- SIFT feature extraction
- BFMatcher + Lowe's ratio test + RANSAC geometric verification
- Similarity graph → Louvain community detection

### Phase 2: Image Stitching  
- Maximum spanning tree → BFS stitching order
- Global canvas pre-computation via corner projection
- Feather blending for seamless output
- Two-level stitching: regional then global

### Phase 3: Orthorectification
- Black pixel boundary detection
- Convex hull approximation → perspective transform

### Phase 4: Road Segmentation
- UNet trained on DeepGlobe dataset (1000 images, 256×256)
- BCE + Dice loss, pos_weight for class imbalance
- Best Dice: ~0.40 on validation set

### Bonus: Shortest Path (A*)
- A* pathfinding on segmented road mask
- Snap-to-road for robust start/end point handling  
- Diagonal movement with corner-cutting prevention
- **Note:** Coordinates are specified directly in the stitched map space. 
  The intended implementation would accept coordinates from original fragmented images and map them to the final stitched canvas via cumulative homography — not completed due to time constraints.

## Known Limitations
- Repeated landmarks cause false matches in feature-based grouping
- Global stitching has lower inlier counts on blended regional maps
- UNet performance dependent on domain similarity to DeepGlobe
- Road segmentation quality affects A* path connectivity

## Dataset
- Stitching: Custom aerial/satellite image fragments
- Road Segmentation: DeepGlobe Road Extraction Dataset

## Attempted Approaches (Road Segmentation)
- **HSV Sliders:** Interactive OpenCV trackbar tuning — good intuition building but required manual retuning per image (poor generalization)
- **Thresholding (Otsu/Adaptive):** Applied on saturation channel — fast but couldn't distinguish roads from same-colored rooftops/shadows
- **Random Forest:** Patch-based classification with HSV, LBP, Sobel features — 81% accuracy but limited by hand-crafted feature expressiveness
- **UNet (Final):** Encoder-decoder with skip connections trained on DeepGlobe — best generalization, ~0.40 Dice on validation set      

### Concepts/Notes 
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