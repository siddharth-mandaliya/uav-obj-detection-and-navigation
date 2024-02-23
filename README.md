# uav-obj-detection-and-navigation

drone_target_navigation.py: This file contains the code responsible for guiding the drone based on objects detected using the YOLO algorithm.

call_from_yolo.py: Here, you'll find the code to identify objects and humans from the camera feed.

The navigation code offers the operator a list of potential targets and the ability to navigate to them. Once a target is reached, there's an option to activate the servo motor for dropping based on operator input.

Please note: The midpoint for capturing images with the drone may need adjustment based on specific requirements.

The current YOLO model is trained on a dataset comprising 1379 images sized at 1920 x 1080 pixels, taken from a height of 100 feet. For enhanced performance in various scenarios, the model can also be trained on personalized datasets.
