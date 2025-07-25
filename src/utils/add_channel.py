import cv2
import numpy as np

# Load image.
img = cv2.imread('/Users/alessioborgi/Documents/GitHub/Tohoku_TESP/img/3_channels/occupancy_grid_with_crater_small.png') 
if img is None:
    raise ValueError("Image not found!")

# Check number of channels.
print(f"Original shape: {img.shape}")

# Add alpha channel (fully opaque).
if img.shape[2] == 3:
    alpha_channel = np.ones(img.shape[:2], dtype=img.dtype) * 255
    img_bgra = cv2.merge([img, alpha_channel])
    print(f"New shape with alpha: {img_bgra.shape}")
else:
    print("Image already has an alpha channel!")
    img_bgra = img 

# Save the new 4-channel image.
cv2.imwrite('/Users/alessioborgi/Documents/GitHub/Tohoku_TESP/img/4_channels/occupancy_grid_with_crater_small_4channels.png', img_bgra)
print("Saved image with alpha channel.")
