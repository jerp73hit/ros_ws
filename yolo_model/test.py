import ultralytics
from ultralytics import YOLO
import yaml
import os

import ultralytics

print(ultralytics.__version__)

# 1. Load the model
model = YOLO("./weights/best.pt")

# 2. Pass the entire directory directly to the model
# The model will process all images, and save=True writes them to a new 'runs/detect/predict' folder
results = model(source="./vaina.jpeg", save=True, show=True)


s = yaml.dump(results)

fd = os.open("res.yaml", os.O_RDWR | os.O_CREAT | os.O_TRUNC)

line = str.encode(s)

numBytes = os.write(fd, line)

print("written: bytes", numBytes)

# close the file descriptor
os.close(fd)
