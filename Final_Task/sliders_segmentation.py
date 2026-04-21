# Using sliders to find best HSV values and kernel size to segment road from background 

import cv2
import numpy as np

def nothing(x):
    pass

# Load your topdown image
image = cv2.imread('test_1.jpg')
blurred_img = cv2.GaussianBlur(image, (5,5), 0)
hsv = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)

height, width = image.shape[:2]
img_area = height * width

cv2.namedWindow('Road Segmentation')

# Create trackbars
cv2.createTrackbar('H Low', 'Road Segmentation', 0, 180, nothing)
cv2.createTrackbar('H High', 'Road Segmentation', 180, 180, nothing)
cv2.createTrackbar('S Low', 'Road Segmentation', 0, 255, nothing)
cv2.createTrackbar('S High', 'Road Segmentation', 80, 255, nothing)
cv2.createTrackbar('V Low', 'Road Segmentation', 50, 255, nothing)
cv2.createTrackbar('V High', 'Road Segmentation', 200, 255, nothing)
cv2.createTrackbar('Open K', 'Road Segmentation', 3, 15, nothing)
cv2.createTrackbar('Close K', 'Road Segmentation', 7, 15, nothing)

while True:
    h_low = cv2.getTrackbarPos('H Low', 'Road Segmentation')
    h_high = cv2.getTrackbarPos('H High', 'Road Segmentation')
    s_low = cv2.getTrackbarPos('S Low', 'Road Segmentation')
    s_high = cv2.getTrackbarPos('S High', 'Road Segmentation')
    v_low = cv2.getTrackbarPos('V Low', 'Road Segmentation')
    v_high = cv2.getTrackbarPos('V High', 'Road Segmentation')
    open_k = max(1, cv2.getTrackbarPos('Open K', 'Road Segmentation'))
    close_k = max(1, cv2.getTrackbarPos('Close K', 'Road Segmentation'))

    mask = cv2.inRange(hsv, (h_low, s_low, v_low), (h_high, s_high, v_high))
    
    kernel_open = np.ones((open_k, open_k), np.uint8)
    kernel_close = np.ones((close_k, close_k), np.uint8)
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
    cleaned = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel_close)
    
    contours, _ = cv2.findContours(cleaned, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    overlay = image.copy()

    for cnt in contours:
        area = cv2.contourArea(cnt)
        x, y, w, h = cv2.boundingRect(cnt)
        aspect_ratio = h/float(w)
        
        if img_area*0.045 < area < img_area*0.3: 
            if 0.77 < aspect_ratio < 1.33:
                cv2.drawContours(overlay, [cnt], -1, (0, 0, 255), thickness=-1)
    
    combined = cv2.hconcat([overlay, cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR), cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
    cv2.imshow('Road Segmentation', combined)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        # Save final parameters when quitting
        print(f"Final params: H({h_low},{h_high}) S({s_low},{s_high}) V({v_low},{v_high})")
        print(f"Open kernel: {open_k}, Close kernel: {close_k}")
        break

cv2.destroyAllWindows()

## Best Values without size shape filtering
# H Low: 0
# H High:180
# S Low: 0
# S High: 50
# V Low: 10
# V High: 255
# Open K: 10
# Close K: 4