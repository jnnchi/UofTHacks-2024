import numpy as np
import cv2
import math
import imutils
from Line import Line

class CornerRecog:
    def __init__(self, image):
        self.image = image

    def get_board_corners(self):
        # updates self.image
        return self.resize_image() # (self.top_left, self.top_right, self.bottom_left, self.bottom_right)
        

    def resize_image(self):
        """
        returns warped image of chessboard 
        (updates self.image to be warped)
        """
        self.image = cv2.rotate(self.image, cv2.ROTATE_90_CLOCKWISE)
        width, height = self.image.shape[0], self.image.shape[1]
        self.image = self.image[0:int(height),0:int(width)]
        width, height = self.image.shape[0], self.image.shape[1]


        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        # Define the range of green color in HSV
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])

        # Create a mask for green color
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        green_points = []
        for cnt in contours:
            # Calculate the center of the contour
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                green_points.append((cX, cY))

        corners = self.find_extreme_points(green_points)
        for corner in corners:
            cv2.circle(self.image, corners[corner], 1, (0,255, 0), 3)
        
        # Binarize the photo
        adaptiveThresh,img = self.clean_Image(self.image)
        # Black out all pixels outside the border of the chessboard
        mask = self.initialize_mask(adaptiveThresh,img)
        
        top_left = corners["top_left"]
        top_right = corners["top_right"]
        bottom_left = corners["bottom_left"]
        bottom_right = corners["bottom_right"]
        src_pts = np.array([top_left, top_right, bottom_left, bottom_right], dtype='float32')
        dest_pts = np.array([[0, 0], [width, 0], [0,height], [width,height]], dtype='float32')
        M = cv2.getPerspectiveTransform(src_pts, dest_pts)
        self.image = cv2.warpPerspective(self.image, M, (width, height))
        cv2.imshow("ETHRRe", self.image)
        cv2.waitKey(6000)
        #return top_left, top_right, bottom_left, bottom_right
        return self.image

    def find_extreme_points(self,points):
        """
        Find the extreme points in a list of points: top-left, top-right, bottom-right, bottom-left.
        """
        # Initialize extreme points with the first point values
        top_left = top_right = bottom_right = bottom_left = points[0]

        for point in points:
            x, y = point

            # Check for top-left (minimize x + y)
            if x + y < top_left[0] + top_left[1]:
                top_left = point

            # Check for top-right (maximize x - y)
            if x - y > top_right[0] - top_right[1]:
                top_right = point

            # Check for bottom-right (maximize x + y)
            if x + y > bottom_right[0] + bottom_right[1]:
                bottom_right = point

            # Check for bottom-left (maximize y - x)
            if y - x > bottom_left[1] - bottom_left[0]:
                bottom_left = point

        return {
            'top_left': top_left,
            'top_right': top_right,
            'bottom_right': bottom_right,
            'bottom_left': bottom_left
        }
        
    def clean_Image(self,image):
        '''
        Resizes and converts the photo to black and white for simpler analysis
        helper function for corner detection
        '''
        # resize image
        img = imutils.resize(image, width=400, height = 400)

        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        # Setting all pixels above the threshold value to white and those below to black
        # Adaptive thresholding is used to combat differences of illumination in the picture
        adaptiveThresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 125, 1)

        return adaptiveThresh,img

    def initialize_mask(self, adaptiveThresh,img):
        '''
        Finds border of chessboard and blacks out all unneeded pixels
        helper function for corner detection
        '''

        # Find contours (closed polygons)
        contours, hierarchy = cv2.findContours(adaptiveThresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Create copy of original image
        imgContours = img.copy()

        for c in range(len(contours)):
            # Area
            area = cv2.contourArea(contours[c])
            # Perimenter
            perimeter = cv2.arcLength(contours[c], True)
                # Filtering the chessboard edge / Error handling as some contours are so small so as to give zero division
                #For test values are 70-40, for Board values are 80 - 75 - will need to recalibrate if change
                #the largest square is always the largest ratio
            if c == 0:
                Lratio = 0
            if perimeter > 0:
                ratio = area / perimeter
                if ratio > Lratio:
                    largest=contours[c]
                    Lratio = ratio
                    Lperimeter=perimeter
                    Larea = area
            else:
                    pass

        # Draw contours
        cv2.drawContours(imgContours, [largest], -1, (0,0,0), 1)

        # Epsilon parameter needed to fit contour to polygon
        epsilon = 0.1 * Lperimeter
        # Approximates a polygon from chessboard edge
        chessboardEdge = cv2.approxPolyDP(largest, epsilon, True)

        # Create new all black image
        mask = np.zeros((img.shape[0], img.shape[1]), 'uint8')*125
        # Copy the chessboard edges as a filled white polygon size of chessboard edge
        cv2.fillConvexPoly(mask, chessboardEdge, 255, 1)
        # Assign all pixels that are white (i.e the polygon, i.e. the chessboard)
        extracted = np.zeros_like(img)
        extracted[mask == 255] = img[mask == 255]
        # remove strip around edge
        extracted[np.where((extracted == [125, 125, 125]).all(axis=2))] = [0, 0, 20]
        return extracted