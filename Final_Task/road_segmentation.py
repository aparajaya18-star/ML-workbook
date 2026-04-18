import cv2
import numpy as np

def nothing(x):
    pass

# Load your topdown image
image = cv2.imread('topdown_image.png')
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

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

    overlay = image.copy()
    overlay[cleaned > 0] = (0, 0, 255)
    
    cv2.imshow('Road Segmentation', overlay)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        # Save final parameters when quitting
        print(f"Final params: H({h_low},{h_high}) S({s_low},{s_high}) V({v_low},{v_high})")
        print(f"Open kernel: {open_k}, Close kernel: {close_k}")
        break

cv2.destroyAllWindows()