# pip install opencv-contrib-python
# pip install opencv-contrib-python==4.6.0.66

import imutils
import cv2
import numpy as np

DICT_NUM = cv2.aruco.DICT_5X5_100
IM_PATH = "images/marker"

# CREATE ARUCOS
arucoDict1 = cv2.aruco.Dictionary_get(DICT_NUM)
params1 =  cv2.aruco.DetectorParameters()

for i in range(4):
    print(i)
    tag = np.zeros((300, 300, 1), dtype="uint8")
    cv2.aruco.drawMarker(arucoDict1, i, 300, tag, 1)
    top, bottom, left, right = [50, 50, 50, 50]  # example values for a 50-pixel border

    # Add border around the image
    image = cv2.copyMakeBorder(tag, top, bottom, left, right, cv2.BORDER_CONSTANT, value=[255, 255, 255])
    image = cv2.imwrite(f"{IM_PATH}{i}.png", image)
# must make absolutely certain that the type used to generate the ArUco tags is the same type you are using for the detection phase.


# load the input image from disk and resize it
image = cv2.imread("arucos.webp")

def detect_code(image):
    image = imutils.resize(image, width=600)
    cv2.imshow("ArUCo 2", image)


    arucoDict2 = cv2.aruco.Dictionary_get(DICT_NUM)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (markerCorners, markerIds, rejectedCandidates) = cv2.aruco.detectMarkers(image, arucoDict2,
        parameters=arucoParams)

    # verify *at least* one ArUco marker was detected
    if len(markerCorners) > 0:
        # flatten the ArUco IDs list
        ids = markerIds.flatten()

        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(markerCorners, ids):
            # marker corners are always returned in top-left, top-right, bottom-right, and bottom-left order
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            # draw the bounding box of the ArUCo detection
            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
            # compute and draw the center (x, y)-coordinates of the ArUco
            # marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            # draw the ArUco marker ID on the image
            cv2.putText(image, str(markerID),
                (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            print("[INFO] ArUco marker ID: {}".format(markerID))
            # show the output image
            cv2.imshow("Image", image)
            cv2.waitKey(0)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

# image needs to be cropped to each qr code then?
def get_corners(image_path, DICT_NUM, aruco_location):
    image = cv2.imread(image_path)
    image = imutils.resize(image, width=600)

    arucoDict2 = cv2.aruco.Dictionary_get(DICT_NUM)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (markerCorners, markerIds, rejectedCandidates) = cv2.aruco.detectMarkers(image, arucoDict2,
        parameters=arucoParams)

    # verify *at least* one ArUco marker was detected
    if len(markerCorners) > 0:
        # flatten the ArUco IDs list
        ids = markerIds.flatten()

        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(markerCorners, ids):
            # marker corners are always returned in top-left, top-right, bottom-right, and bottom-left order
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            
            # can prob use marker ID here instead
            if aruco_location == 0:
                return bottomLeft
            elif aruco_location == 1:
                return bottomRight
            elif aruco_location == 2:
                return topLeft
            elif aruco_location == 3:
                return topRight

if __name__ == "__main__":
    #detect_code(image)
    print(get_corners("images/marker0.png", DICT_NUM, 0))
