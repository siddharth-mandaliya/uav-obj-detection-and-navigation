# uav-obj-detection-and-navigation

drone_target_navigation.py is has the code for drone navigation based on the objects detected with yolo algorithm.

call_from_yolo.py has the code to detect the objects and humans from th camera feed.

The navigation code presents the operator with the list of all possible target and option to navigate to the targets. After reaching the target it has the option to trigger the servo motor for the drop based on the input by the operator.

The mid-point to take images by the drone needs to be updated as per requirment.

The current yolo model is trained on 1379 images of a 1920 x 1080 px taken from a height of 100 feet. The model can also be trained on personalized dataset for better output in required situations.
