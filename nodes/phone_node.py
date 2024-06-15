#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import requests
import imutils  # Make sure imutils is installed
import math
import json
from pub import MinimalPublisher

rclpy.init(args=None)
publisher = MinimalPublisher(nodeName="shareOnlineCV", topic="shareOnline")
rate = publisher.create_rate(1)

class ImageProcessor():
    def __init__(self):
        #publisher = MinimalPublisher(nodeName="shareOnlineCV", topic="shareOnline")
        rate = publisher.create_rate(1)

    def capture_and_process_image(self, url):
        img = self.capture_image(url)
        img = imutils.resize(img, width=800, height=600)
        cv2.imshow("Captured Image", img)
        board = Board(img)
        # when image is given
        board.update_board_layout()
        print(board.board_layout)
        
        array_msg = json.dumps(board.board_layout)
        publisher.timer_callback(array_msg)
        

        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def capture_image(self, url):
        img_resp = requests.get(url)
        img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
        img = cv2.imdecode(img_arr, -1)
        return img

    def process_image(self, cv_image):
        # Dummy image processing logic, replace with your actual logic
        processed_data = [float(cv_image.shape[0]), float(cv_image.shape[1])]
        return processed_data

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()

    url = "http://100.67.11.113:8080/shot.jpg"
    print("Press 'c' to capture an image from the IP Webcam, or 'q' to quit.")

    while True:
        key = input("Press 'c' to capture or 'q' to quit: ")
        if key == 'c':
            image_processor.capture_and_process_image(url)
        elif key == 'q':
            print("Exiting program.")
            break
        else:
            print("Invalid input. Please press 'c' to capture or 'q' to quit.")

    image_processor.destroy_node()
    rclpy.shutdown()

class BoardSlot:
    def __init__(self, location_on_board, coordinates, piece):
        self.location_on_board = location_on_board # (0,0) to (7,7)
        self.coordinates = (int(coordinates[0]), int(coordinates[1])) # pixel coordinates
        self.piece = piece # none, black, or white
        if self.location_on_board[0] % 2 == 0 and self.location_on_board[1] % 2 == 0:
            self.square_color = "white"
        elif self.location_on_board[0] % 2 == 0 and self.location_on_board[1] % 2 != 0:
            self.square_color = "black"
        elif self.location_on_board[0] % 2 != 0 and self.location_on_board[1] % 2 == 0:
            self.square_color = "black"
        else:
            self.square_color = "white"
    
    def __str__(self):
        return(f"Coordinates {self.coordinates}")

    def mark_image(self, image):
        if self.piece == "black":
            cv2.circle(image, self.coordinates, radius=3, color=(0, 255, 0), thickness=-1)
        elif self.piece == "white":
            cv2.circle(image, self.coordinates, radius=3, color=(255, 0, 255), thickness=-1)
        else:
            cv2.circle(image, self.coordinates, radius=3, color=(255, 0, 0), thickness=-1)

class Board: 
    def __init__(self, image):
        self.image = image
        # get corners
        self.image = CornerRecog(self.image).get_board_corners()
        self.chessboard_width = self.image.shape[0]
        self.chessboard_height = self.image.shape[1]
        self.debug = False
        # set self.mapping to use in the next function
        self.map_coordinate_to_board_layout()
        # get initial board layout
        self.set_initial_board_layout()
    
    def map_coordinate_to_board_layout(self):
        # margin of error
        offsetX = 5
        offsetY = 5

        self.mapping = {"x": {i: () for i in range(8)}, "y": {j: () for j in range(8)}}
        
        for r in range(8):
            x = r * self.chessboard_width/8
            if x == 0:
                self.mapping["x"][r] = (x, x + self.chessboard_width/8 + offsetX) # call it by doing mapping[x][0]
            else:
                self.mapping["x"][r] = (x + offsetX, x + self.chessboard_width/8 + offsetX) # call it by doing mapping[x][0]

        for c in range(8):
            y = c * self.chessboard_height/8
            if y==0:
                self.mapping["y"][c] = (y, y + self.chessboard_height/8 + offsetY)
            else:
                self.mapping["y"][c] = (y + offsetY, y + self.chessboard_height/8 + offsetY)
    
    def set_initial_board_layout(self):
        self.board_layout = [[() for _ in range(8)] for _ in range(8)]
        for i in range(8):
            for j in range(8):
                x, y = i * self.chessboard_width/8, j * self.chessboard_height/8
                x += self.chessboard_width/16
                y += self.chessboard_height/16
                self.board_layout[i][j] = (x,y)
        
        self.initial_board_layout = {(r, c): BoardSlot((r, c), 
                                                self.board_layout[r][c], 
                                                "black" if 0<=r<=1
                                                else ("white" if 6<=r<=7 else "none")) 
                            for r in range(len(self.board_layout)) 
                            for c in range(len(self.board_layout[0]))}
        # FOR TESTING ONLY:
        #for key in self.initial_board_layout:
            #self.initial_board_layout[key].mark_image(self.image)
        self.board_layout = self.initial_board_layout

    def update_board_layout(self):
        # MUST CHANGE
        # detect red circles
        detected_circles = self.detect_red_circles()
        # will be x number of red circles
        num_pieces_on_board = 64 - len(detected_circles)

        # you can't update, you need to remake the board layout
        # make a new empty mapping {place on board: red dot yes or no}
        temp_new_layout = {(i, j): None for i in range(8) for j in range(8)}
        # map every red circle to a place on the board
        for circle in detected_circles:
            x, y = circle
            row, col = None, None
            # check with map
            for r in range(8):
                for c in range(8):
                    if self.mapping["x"][r][0] <= x <= self.mapping["x"][r][1] \
                    and self.mapping["y"][c][0] <= y <= self.mapping["y"][c][1]:
                        row, col = r, c
            if row is not None and col is not None:
                temp_new_layout[(row, col)] = circle
        # now you know the squares that don't have red circles
        # then you can crop to those squares and detect black/white

        for key in temp_new_layout:
            if temp_new_layout[key] is None:
                square_img = self.crop_to_square(self.initial_board_layout[key].coordinates, self.image, self.chessboard_height/4, self.chessboard_width/4)
                avg_intensity = self.get_average_intensity(square_img)
                # now update board layout
                if self.board_layout[key].square_color == "white":
                    if avg_intensity > 100:
                        self.board_layout[key].piece = "white"
                    else:
                        self.board_layout[key].piece = "black"
                else: # black
                    if avg_intensity > 60:
                        self.board_layout[key].piece = "white"
                    else:
                        self.board_layout[key].piece = "black"
                #cv2.imshow("sq", square_img)
                #cv2.waitKey(2000)
                #print(f'key {key}, intensity {avg_intensity}, {self.board_layout[key].piece}')
            else:
                self.board_layout[key].piece = "none"        
        
        output_list = [["" for j in range(8)] for i in range(8)]
        for key in self.board_layout:
            r,c = key
            output_list[r][c] = self.board_layout[key].piece
        print(output_list)
        return output_list
    
    def crop_to_square(self, square_center, image, chessboard_height, chessboard_width):
        """
        (helper for update board layout)
        crop image based on center of square, return new image
        """
        new_image = image[int(square_center[1]-chessboard_height/16):int(square_center[1]+chessboard_height/16),int(square_center[0]-chessboard_width/16):int(square_center[0]+chessboard_width/16)]
        return new_image
    
    def get_average_intensity(self, image):
        """
        (helper for update board layout)
        Calculate the average intensity of a grayscale image.
        """
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Calculate the average intensity
        average_intensity = np.mean(image)
        return average_intensity
    
    def detect_red_circles(self):
        width, height, _ = self.image.shape
        output_image = self.image.copy()

        # denoise image
        image = cv2.fastNlMeansDenoisingColored(self.image,None,10,10,7,21)

        # conv to hsv
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # define ranges for red color
        # 1) broad range
        lower_red1 = np.array([0, 110, 50])
        upper_red1 = np.array([100, 255, 255])
        # 2) tighter range
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        # create masks that only show what's in the ranges
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        # add them up to get final mask
        mask = mask1 + mask2

        # set output img to zero (black) everywhere except my mask
        output_image[np.where(mask==0)] = 0

        # now grayscale to detect circles
        gray_image = cv2.cvtColor(output_image, cv2.COLOR_BGR2GRAY)
        gray_image = cv2.medianBlur(gray_image, 5)
        # width/11 can be reduced if the board is smaller
        circles = cv2.HoughCircles(gray_image, cv2.HOUGH_GRADIENT, 1, width / 11,
                                    param1=100, param2=9, # smaller param2 means more circles
                                    minRadius=1, maxRadius=30)
        verified_circles = []
        # draw the detectedcircles
        if circles is not None:
            circles = np.uint16(np.around(circles))
            test = output_image.copy()
            for i in circles[0, :]:
                center = (i[0], i[1])
                verified_circles.append(center)
                # circle center
                cv2.circle(test, center, 1, (0,255, 0), 3)
                # circle outline
                radius = i[2]
                cv2.circle(test, center, radius, (0, 255, 0), 3)

            cv2.imshow('mask Image', test)
            cv2.waitKey(6000)

        # returns a list of all circles
        return verified_circles

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

class Line:

	def __init__(self,x1,x2,y1,y2):
		'''
		Creates a Line object
		'''

		# Endpoints
		self.x1 = x1
		self.x2 = x2
		self.y1 = y1
		self.y2 = y2


		# Change in x and y
		self.dx = self.x2 - self.x1
		self.dy = self.y2 - self.y1

		# Orientation
		if abs(self.dx) > abs(self.dy):
			self.orientation = 'horizontal'
		else:
			if abs(self.dx) < abs(self.dy) - 100:
				self.orientation = 'vertical'
			else:
				self.orientation = "neither"


	def find_intersection(self,other):
		'''
		Finds intersection of this line and other. One line must be horizontal
		and the other must be vertical
		'''

		# Determinant for finding points of intersection
		x = ((self.x1*self.y2 - self.y1*self.x2)*(other.x1-other.x2) - (self.x1-self.x2)*(other.x1*other.y2 - other.y1*other.x2))/ ((self.x1-self.x2)*(other.y1-other.y2) - (self.y1-self.y2)*(other.x1-other.x2))
		y = ((self.x1*self.y2 - self.y1*self.x2)*(other.y1-other.y2) - (self.y1-self.y2)*(other.x1*other.y2 - other.y1*other.x2))/ ((self.x1-self.x2)*(other.y1-other.y2) - (self.y1-self.y2)*(other.x1-other.x2))
		x = int(x)
		y = int(y)

		return (x,y)

if __name__ == '__main__':
    main()