import cv2
import numpy as np

# Load the image
image = cv2.imread(r'c:\Users\USER\Desktop\Resources\L-3,T-2\EEE318\front_view.jpg')
original_image = image.copy()

# Convert image to HSV color space (better for color detection)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define color ranges in HSV
# Red has two ranges because it wraps around the hue circle
color_ranges = {
    'Red': [
        (np.array([0, 60, 60]), np.array([15, 255, 255])),
        (np.array([155, 60, 60]), np.array([180, 255, 255]))
    ],
    'Green': [
        (np.array([45, 40, 40]), np.array([90, 220, 220]))   # Balanced - not too tight, sat floor raised
    ],
    'Blue': [
        (np.array([85, 40, 40]), np.array([135, 255, 255]))
    ]
}


# Define colors for bounding boxes (BGR format)
box_colors = {
    'Red': (0, 0, 255),
    'Green': (0, 255, 0),
    'Blue': (255, 0, 0)
}

# Process each color
all_detections = []

for color_name, ranges in color_ranges.items():
    # Create mask for this color
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    
    for lower, upper in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
    
    # Apply morphological operations to clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    # Apply Gaussian blur to smooth edges
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    print(f"\n{color_name} objects detected: {len(contours)}")
    
    # Process each contour
    for idx, contour in enumerate(contours):
        # Filter out small noise
        area = cv2.contourArea(contour)
        if area < 100:  # Adjust this threshold based on your image
            continue
        
        # Get bounding rectangle
        x, y, w, h = cv2.boundingRect(contour)
        
        # Store detection info
        all_detections.append({
            'color': color_name,
            'x': x, 'y': y, 'w': w, 'h': h,
            'area': area
        })
        
        # Draw bounding box with color-specific color
        cv2.rectangle(original_image, (x, y), (x + w, y + h), 
                     box_colors[color_name], 3)
        
        # Add label with color name and height
        label = f"{color_name}: {h}px"
        cv2.putText(original_image, label, (x, y - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_colors[color_name], 2)
        
        # Print details
        print(f"  {color_name} Object {idx + 1}:")
        print(f"    Position: ({x}, {y})")
        print(f"    Width: {w}px, Height: {h}px")
        print(f"    Area: {area}px²")

# Print summary
print(f"\n{'='*50}")
print(f"Total objects detected: {len(all_detections)}")
print(f"{'='*50}")

# Show results
cv2.imshow("Detected Red, Green, Blue Objects", original_image)

# Optional: Save the output image
cv2.imwrite("output_color_detection.jpg", original_image)

# Wait for key press
cv2.waitKey(0)
cv2.destroyAllWindows()