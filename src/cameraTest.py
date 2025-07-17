import cv2
import os

save_dir = "trainImages"
os.makedirs(save_dir, exist_ok=True)
cameraIndex = 2
cap = cv2.VideoCapture(cameraIndex)
i = 0
if not cap.isOpened():
    print(f"Camera {cameraIndex} could not be opened.")
else:
    print(f"Camera {cameraIndex} opened successfully.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        cv2.imshow(f'Camera {cameraIndex}', frame)
        k = cv2.waitKey(1)
        if k%256 == 32:
            imgName = f"image_{i}.png"
            imgPath = os.path.join(save_dir, imgName)
            cv2.imwrite(imgPath, frame)
            print(f"{imgName} saved successfully")
            i += 1

    cap.release()
    cv2.destroyAllWindows()
