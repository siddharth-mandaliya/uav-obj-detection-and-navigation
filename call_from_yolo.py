import torch
import cv2

class TargetDetection:
    
    def __init__(self, yolo_path, model_path):
 
        # self.capture_index = capture_index
        self.path_of_yolo = yolo_path
        # self.frame_main=detection_frame
        self.path_of_model = model_path
        self.model = self.load_model()
        self.classes = self.model.names
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print("Using Device: ", self.device)

    # def get_video_capture(self):      
    #     return cv2.VideoCapture(self.capture_index)

    def load_model(self):

        model = torch.hub.load(self.path_of_yolo, 'custom', path=self.path_of_model, source='local', force_reload=True)
        
        return model

    def score_frame(self, frame):

        self.model.to(self.device)
        frame = [frame]
        results = self.model(frame)
        
        labels, cord = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]

        # return results
        return labels, cord

    def class_to_label(self, x, jj):
        
        return (str(self.classes[int(x)])+" "+str(jj))
            
    def plot_boxes(self, results1, results2, frame):

        labels, cord = results1, results2
        n = len(labels)
        x_shape, y_shape = frame.shape[1], frame.shape[0]
        objeects = 1
        for i in range(n):
            row = cord[i]
            # Detection only for Objects set by labels[i] == 1
            if labels[i] == 1:
                # Conf Score
                if row[4] >= 0.7:
                    x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
                    bgr = (0, 255, 0)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)
                    cv2.putText(frame, self.class_to_label(labels[i], jj=objeects), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)
                    objeects = objeects + 1

        return frame

    def get_relative_coordinates(self, photo):
        results, results2 = self.score_frame(photo)
        results, results2 = results.tolist(), results2.tolist()

        global detected_humans, detected_objects, input_height, input_width

        detected_objects = []
        detected_humans = []

        input_height, input_width = photo.shape[:2]
        for i in range (len(results)):
            row = results2[i]
            if results[i] == 0:
                if row[4] >= 0.7:
                    location_array = [((results2[i][2]+results2[i][0])/2)*input_width, ((results2[i][3]+results2[i][1])/2)*input_height]
                    detected_humans.append(location_array)
            
            elif results[i] == 1:
                if row[4] >= 0.7:
                    location_array2 = [((results2[i][2]+results2[i][0])/2)*input_width, ((results2[i][3]+results2[i][1])/2)*input_height]
                    detected_objects.append(location_array2)

        # photo = self.plot_boxes(results, results2, photo)

        detected_objects_relative_to_center = []
        detected_humans_relative_to_center = []

        for j in range (len(detected_humans)):
            n_decider = (input_height/2) - detected_humans[j][1]
            e_decider = detected_humans[j][0] - (input_width/2)
            
            relative_location = [f"{n_decider} N, {e_decider} E"]
            detected_humans_relative_to_center.append(relative_location)

        for k in range (len(detected_objects)):
            n_decider2 = (input_height/2) - detected_objects[k][1]
            e_decider2 = detected_objects[k][0] - (input_width/2)
            
            relative_location2 = [f"{n_decider2} N, {e_decider2} E"]
            detected_objects_relative_to_center.append(relative_location2)

        return detected_humans_relative_to_center, detected_objects_relative_to_center
             
    def __call__(self):
        cam = cv2.VideoCapture('nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink', cv2.CAP_GSTREAMER)
       
        result, frame = cam.read()
       
        relative_coordinates_lai_aav = self.get_relative_coordinates
        human_coordinates, object_coordinates = relative_coordinates_lai_aav(frame)

        results, results2 = self.score_frame(frame)
        frame2 = self.plot_boxes(results, results2, frame)

        # fps = 1/np.zeros(end_time - start_time, 2)
        # cv2.putText(frame, f'FPS: {int(fps)}', (20,70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)
        
        cv2.imwrite("Image.jpg", frame2)
        
        return human_coordinates, object_coordinates

def gstreamer_pipeline (capture_width=1920, capture_height=1080, display_width=640, display_height=360, framerate=60, flip_method=0) :   
    return ('nvarguscamerasrc ! ' 
    'video/x-raw(memory:NVMM), '
    'width=(int)%d, height=(int)%d, '
    'format=(string)NV12, framerate=(fraction)%d/1 ! '
    'nvvidconv flip-method=%d ! '
    'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
    'videoconvert ! '
    'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))


def main():
    path_to_yolo = r"./yolov5-transformer"
    path_to_model = r"./yolov5-transformer/pretrained-weights/navigation_detection.pt"
    
    # cam = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    # result, image = cam.read()
    # if result:
    #     cv2.imwrite("Image.jpeg", image)
    #     cv2.waitKey(0)

    detector = TargetDetection(yolo_path=path_to_yolo, model_path=path_to_model)
    human_coordinates, object_coordinates = detector()

    return human_coordinates, object_coordinates

if __name__ == "__main__":
    main()
