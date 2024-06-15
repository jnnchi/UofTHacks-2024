import numpy as np
import cv2
import math
import imutils
from Line import Line
from BoardSlot import BoardSlot
from CornerRecog import CornerRecog

class Board: 
    def __init__(self, image_path):
        self.image = cv2.imread(image_path)
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

            #cv2.imshow('mask Image', test)
            #cv2.waitKey(6000)

        # returns a list of all circles
        return verified_circles
    
if __name__ == "__main__":
    #test2 works
    board = Board("/Users/jennifer/VSCodeProjects/RoboChess/opencv_temp/test2.jpg")
    # when image is given
    board.update_board_layout()
