import cv2
from inference_sdk import InferenceHTTPClient
from PIL import Image, ImageDraw, ImageFont
import numpy as np
import base64 # Import the base64 module
from io import BytesIO # Keep this for PIL to bytes conversion

detectionDelay = 20 
frameCount = detectionDelay
lastResult = 0

# --- Roboflow Inference Client Initialization ---
CLIENT = InferenceHTTPClient(
    api_url="https://serverless.roboflow.com",
    api_key="hh1Q7q5T4PDmmGAIEqrB"
)

# --- Class Colors Mapping ---
class_colors = {
    'Rock': 'red',
    'Yellow Turtle': 'gold',
    'Black Dinosaur': 'darkgreen',
    'Black Turtle': 'blue'
}

# --- Font for Labels ---
try:
    font = ImageFont.truetype("arial.ttf", 15)
except IOError:
    font = ImageFont.load_default()
    print("Could not load 'arial.ttf'. Using default font.")


# --- Camera Initialization ---
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Camera opened successfully. Press 'q' to quit.")

while True:
    ret, frame = cap.read() # Read a frame from the camera

    if not ret:
        print("Failed to grab frame")
        break

    # --- Convert OpenCV frame (NumPy array) to PIL Image ---
    cv2_rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(cv2_rgb_frame)

    frameCount += 1
    if frameCount % detectionDelay == 0 or lastResult == 0:
        try:
            # Convert PIL image to bytes
            img_byte_arr = BytesIO()
            pil_image.save(img_byte_arr, format='JPEG')
            img_bytes = img_byte_arr.getvalue()

            # Base64 encode the image bytes
            base64_image = base64.b64encode(img_bytes).decode('utf-8')

            # Send the base64 encoded image string to Roboflow
            lastResult = CLIENT.infer(base64_image, model_id="manualsegmentation/5")
        except Exception as e:
            print(f"Error during Roboflow inference: {e}")
            lastResult = {'predictions': []}

    result = lastResult

    
    # --- Visualize Results on PIL Image ---
    draw = ImageDraw.Draw(pil_image)

    for prediction in result['predictions']:
        x, y, width, height = prediction['x'], prediction['y'], prediction['width'], prediction['height']
        class_name = prediction['class']
        confidence = prediction['confidence']
        label = f"{class_name} ({confidence:.2f})"

        # Get the color based on the class name
        box_color = class_colors.get(class_name, 'white') # Default to white for unknown classes

        # Calculate bounding box coordinates (top-left and bottom-right)
        x_min = x - width / 2
        y_min = y - height / 2
        x_max = x + width / 2
        y_max = y + height / 2

        # Draw the bounding box
        draw.rectangle([x_min, y_min, x_max, y_max], outline=box_color, width=3)

        # Draw the label text
        text_y = y_min - 20 if y_min - 20 > 0 else y_min + 5
        draw.text((x_min, text_y), label, fill=box_color, font=font)

    # --- Convert PIL Image back to OpenCV format for display ---
    annotated_frame = np.array(pil_image)
    annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)

    # --- Display the annotated frame ---
    cv2.imshow('Live Object Detection', annotated_frame)

    # --- Break the loop on 'q' key press ---
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
    
    for item in result['predictions']:
        if item['class'] == 'Black Turtle':
            print("-------------------------------------------------------------------------------------------------------------")
            print(f"Black Turtle found at camera coordinates [{item['x'], item['y']}], with size [{item['width'], item['height']}]")
    
"""
# --- Release resources ---
cap.release()
cv2.destroyAllWindows()
"""