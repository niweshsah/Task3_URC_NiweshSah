import cv2
import cv2.aruco as aruco
import numpy as np

def detect_aruco_in_video():

    video_path = '/path/to/your/video.mp4' # very important to write your username also
    
    cap = cv2.VideoCapture(video_path)

    # Check if the webcam is opened correctly
    if not cap.isOpened():
        print(f"Error: Could not open video at {video_path}.")
        return

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

    parameters =  aruco.DetectorParameters()

    while True:

        # Capture frame-by-frame
        ret, frame = cap.read() # ret is a boolean character which returns false if frame is not captured correctly

        # Check if frame is captured correctly
        if not ret:
            print("Error: Could not read frame.")
            break

        # Detect the markers in the frame
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    
        # Draw detected markers
        if np.all(ids is not None): # if not all markers are None, i.e. (if there is any detected id)
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

        # Display the frame with detected markers
        cv2.imshow('Detected ArUco markers', frame)

        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture object and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()


# Call the function to start detection
if __name__ == "__main__":
    detect_aruco_in_video()
