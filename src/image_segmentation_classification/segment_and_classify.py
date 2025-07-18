# Install Roboflow if not already installed.
# !pip install roboflow

from roboflow import Roboflow

# Initialize Roboflow with your API key.
rf = Roboflow(api_key="hh1Q7q5T4PDmmGAIEqrB")
project = rf.workspace("tespproject2025").project("manualsegmentation")
version = project.version(4)
dataset = version.download("darknet")


                