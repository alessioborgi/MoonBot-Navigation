import cv2

# Load your grayscale or binary image
img = cv2.imread('/Users/alessioborgi/Documents/GitHub/Tohoku_TESP/img/filtered_image.png', cv2.IMREAD_GRAYSCALE)

# Apply median filter
filtered = cv2.medianBlur(img, 5)  # 5 is kernel size; use 3, 5, or 7

cv2.imwrite('/Users/alessioborgi/Documents/GitHub/Tohoku_TESP/img/filtered_image1.png', filtered)
