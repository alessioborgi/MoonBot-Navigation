import cv2

img = cv2.imread('/Users/alessioborgi/Documents/GitHub/Tohoku_TESP/img/occupancy_grid_4channels.png')  # or .jpg etc.
if img is None:
    raise ValueError("Image not found!")

print(f"Image shape: {img.shape}")

if len(img.shape) == 2:
    print("The image is grayscale (1 channel).")
elif len(img.shape) == 3:
    print(f"The image has {img.shape[2]} channels.")
else:
    print("Unexpected image format.")
