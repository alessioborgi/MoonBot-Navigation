import cv2

# Load the image.
img = cv2.imread('/Users/alessioborgi/Documents/GitHub/Tohoku_TESP/img/occupancy_grid_4channels.png', cv2.IMREAD_UNCHANGED)  
if img is None:
    raise ValueError("Image not found!")

# Print the shape and number of channels.
print(f"Image shape: {img.shape}")

# Check the number of channels.
if img.shape[2] == 2:
    print("The image is grayscale (1 channel).")
elif img.shape[2] >= 3:
    print(f"The image has {img.shape[2]} channels.")
else:
    print("Unexpected image format.")
