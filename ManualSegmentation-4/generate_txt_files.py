import glob
import os

relative_path = "/Users/alessioborgi/Documents/GitHub/Tohoku_TESP/ManualSegmentation-4/"
# Generate train.txt
train_imgs = glob.glob(relative_path+"train/*.png") + glob.glob(relative_path+"train/*.jpg")
with open(relative_path+"train.txt", "w") as f:
    for img in train_imgs:
        f.write(os.path.abspath(img) + "\n")

# Generate test.txt
test_imgs = glob.glob(relative_path+"test/*.png") + glob.glob(relative_path+"test/*.jpg")
with open(relative_path+"test.txt", "w") as f:
    for img in test_imgs:
        f.write(os.path.abspath(img) + "\n")

print("train.txt and test.txt generated!")
