import cv2
import numpy as np

img_path = '/Users/alessioborgi/Documents/GitHub/Tohoku_TESP/img/moon_environment_orig.png'
img = cv2.imread(img_path)
if img is None:
    raise ValueError("Image not found.")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# --- Rock segmentation ---
_, thresh = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY_INV)
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
occupancy_grid = np.ones_like(gray) * 255
cv2.drawContours(occupancy_grid, contours, -1, (0), thickness=cv2.FILLED)
filtered = cv2.medianBlur(cv2.medianBlur(occupancy_grid, 5), 5)

# --- Interactive crater annotation ---
crater_info = {"center": None, "radius": None}

def click_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        if crater_info["center"] is None:
            crater_info["center"] = (x, y)
            print(f"Center set at: {crater_info['center']}")
        elif crater_info["radius"] is None:
            dx = x - crater_info["center"][0]
            dy = y - crater_info["center"][1]
            crater_info["radius"] = int((dx**2 + dy**2)**0.5)
            print(f"Radius set at: {crater_info['radius']} (click distance from center)")
            cv2.destroyAllWindows()

# Show image and let user click center, then edge (for radius)
cv2.namedWindow("Select Crater: center, then edge")
cv2.setMouseCallback("Select Crater: center, then edge", click_callback)

img_show = cv2.cvtColor(filtered, cv2.COLOR_GRAY2BGR)
while True:
    cv2.imshow("Select Crater: center, then edge", img_show)
    if cv2.waitKey(20) & 0xFF == 27 or crater_info["radius"] is not None:
        break
cv2.destroyAllWindows()

# Draw the crater as a filled black circle
if crater_info["center"] and crater_info["radius"]:
    cv2.circle(filtered, crater_info["center"], crater_info["radius"], 0, thickness=-1)
    print("Crater drawn on occupancy grid.")

# --- Save ---
cv2.imwrite('/Users/alessioborgi/Documents/GitHub/Tohoku_TESP/img/occupancy_grid_with_crater_MANUAL.png', filtered)
print("Saved occupancy grid with manually marked crater.")
