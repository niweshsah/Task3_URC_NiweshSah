import cv2
import cv2.aruco as aruco
import random

def main():

    listOfMarkerID = []

    for j in range(9):

        id = random.randint(0, 249) # id is from 0 to 249

        marker_size = 600

        dictionary=aruco.DICT_6X6_250
        aruco_dict = aruco.getPredefinedDictionary(dictionary)

        marker_image = aruco.generateImageMarker(aruco_dict,id, marker_size)

        cv2.imwrite(f"marker{j + 1}.png", marker_image)

        listOfMarkerID.append(id)

    print(listOfMarkerID)


if __name__ == "__main__":
    main()

