def calculate_exact_circle_positions(self):
        output_image = self.image.copy()
        circle_list = self.detect_red_circles()
        circle_coordinates = sorted([[circle[0],circle[1]] for circle in circle_list])
        sorted_coords = sorted(circle_coordinates, key=lambda coord: (coord[1], coord[0]))

        # The resulting sorted list is then sliced into 8 rows with 8 coordinates each
        # We use a step size of 8 since we are creating an 8x8 array
        coords_8x8 = [sorted_coords[i:i+8] for i in range(0, len(sorted_coords), 8)]

        # Sort each row by x-coordinate to ensure they are from left to right
        coords_8x8 = [sorted(row, key=lambda coord: coord[0]) for row in coords_8x8]

        # Finally, we sort the entire array by the y-coordinate of the first element in each row
        # to ensure that the rows are ordered from top to bottom
        coords_8x8 = sorted(coords_8x8, key=lambda row: row[0][1], reverse=True)
        
        # now reverse it so it goes from top to bottom
        coords_8x8 = coords_8x8[::-1]

        if self.debug:
            for row in coords_8x8:
                for center in row:
                    # circle center
                    cv2.circle(output_image, center, 1, (0,255, 0), 10)

                cv2.imshow('mask Image', output_image)
                cv2.waitKey(1000)
                cv2.destroyAllWindows()
                print(row)
    
        #chessboard_locations = {(i, j): coords_8x8[i][j] for i in range(len(coords_8x8)) for j in range(len(coords_8x8[0]))}
        
        circle_positions = {(i, j): BoardSlot((i, j), coords_8x8[i][j]) for i in range(len(coords_8x8)) for j in range(len(coords_8x8[0]))}
        return circle_positions


# unworking
def map_detected_circles_to_grid(self, width, height):
        """
        For each detected circle, check if it's within error bounds of any of the original coordinates.
        Return a list of booleans indicating whether the corresponding original coordinate is covered.
        """
        error_boundx = width/16
        error_boundy = height/16
        circle_list = sorted(self.detect_red_circles())
        covered = [[] for i in range(y)] # [(0,1),[7,7], etc]

        # Calculate the width and height of each cell
        cell_width = (self.initial_circle_positions[0][-1][0] - self.initial_circle_positions[0][0][0]) / (len(self.initial_circle_positions[0]) - 1)
        cell_height = (self.initial_circle_positions[-1][0][1] - self.initial_circle_positions[0][0][1]) / (len(self.initial_circle_positions) - 1)

        for pt in circle_list:
            x, y, _ = pt
            # Calculate which row and column this point should belong to
            col = int((x - self.initial_circle_positions[0][0][0] + error_boundx) // cell_width)
            row = int((y - self.initial_circle_positions[0][0][1] + error_boundy) // cell_height)

            # Check if calculated row and column are within the bounds of the grid
            if 0 <= col < 8 and 0 <= row < 8:
                # Get the original point for the calculated row and column
                x0, y0 = self.initial_circle_positions[row][col]
                # Check if the point is within error bounds of the original point
                if x0 - error_boundx <= x <= x0 + error_boundx and y0 - error_boundy <= y <= y0 + error_boundy:
                    covered[row][col] = True

        for pt in circle_list:
            x,y,_ = pt
            row = 0 # need to estimate what row/col x,y is in
            col = 0
            
            x0,y0 = self.initial_circle_positions[row][col]
            if x0 - error_boundx <= x <= x0 + error_boundx and y0 - error_boundy <= y <= y0 + error_boundy:
                pass

def detect_furthest_corners(self):
        """
        returns four corners of chessboard (like the actual grid)
        """
        all_corner_list, img_width, img_height = self.detect_all_corners()

        top_rightmost = None
        top_leftmost = None
        bottom_rightmost = None
        bottom_leftmost = None

        x_small_constraint, y_small_constraint = .1 * img_width, .1 * img_height
        x_big_constraint, y_big_constraint = .9 * img_width, .9 * img_height

        # Iterate through the coordinates
        for coord in all_corner_list:
            x, y = coord

            # Update top rightmost
            # top rightmost is biggest x smallest y
            # top right: 374, 31
            if top_rightmost is None or ((x >= top_rightmost[0] and y < y_small_constraint) or (y <= top_rightmost[1] and x > x_big_constraint)):
                top_rightmost = (x, y)

            # Update top leftmost
            # smallest x smallest y
            # top left = 18, 36
            if top_leftmost is None or ((x <= top_leftmost[0] and y <= y_small_constraint) or (y <= top_leftmost[1] and x <= x_small_constraint)):
                top_leftmost = (x, y)

            # Update bottom rightmost
            # bottom right = 387, 395
            if bottom_rightmost is None or ((x >= bottom_rightmost[0] and y > y_big_constraint) or (y >= bottom_rightmost[1] and x > x_big_constraint)):
                bottom_rightmost = (x, y)

            # Update bottom leftmost
            # bottom left = 12, 401
            if bottom_leftmost is None or ((x <= bottom_leftmost[0] and y > y_big_constraint) or (y >= bottom_leftmost[1] and x < x_small_constraint)):
                bottom_leftmost = (x, y)
        
        for c in (top_leftmost, top_rightmost, bottom_leftmost, bottom_rightmost):
            cv2.circle(self.image, c, radius=10, color=(0, 255, 0), thickness=-1)

        return (top_leftmost, top_rightmost, bottom_leftmost, bottom_rightmost)

    def detect_all_corners(self):
        """
        detects all 90 degree angles (corners) in an image
        helper function for detect_furthest_corners
        """
        # Binarize the photo
        adaptiveThresh,img = self.clean_Image(self.image)
        # Black out all pixels outside the border of the chessboard
        mask = self.initialize_mask(adaptiveThresh,img)
        # Find edges
        edges,colorEdges = self.findEdges(mask)
        # Find lines
        horizontal, vertical, _ = self.findLines(edges,colorEdges)
        # Find corners
        corners = sorted(self.findCorners(horizontal, vertical, colorEdges))
        
        """
        for corner in corners:
            x, y = corner
            #cv2.circle(colorEdges, (x, y), radius=4, color=(0, 0, 255), thickness=-1)
        # Display the image
        cv2.imshow('Points', colorEdges)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()
        """

        return (corners, colorEdges.shape[0], colorEdges.shape[1])


def findEdges(self, image):
        '''
        Finds edges in the image. Edges later used to find lines and so on
        helper function for corner detection
        '''

        # Find edges
        edges = cv2.Canny(image, 100, 200, None, 3)

        # Convert edges image to grayscale
        colorEdges = cv2.cvtColor(edges,cv2.COLOR_GRAY2BGR)

        return edges,colorEdges

    def findLines (self, edges, colorEdges):
        '''
        Finds the lines in the photo and sorts into vertical and horizontal
        helper function for corner detection
        '''

        # Infer lines based on edges
        lines = cv2.HoughLinesP(edges, 1,  np.pi / 180, 90, np.array([]), minLineLength=200, maxLineGap=80)

        # Draw lines

        a,b,c = lines.shape

        """
        for i in range(a):
            cv2.line(colorEdges, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]), (0,255,0),2,cv2.LINE_AA)
        """
        # Create line objects and sort them by orientation (horizontal or vertical)
        horizontal = []
        vertical = []
        leftmost = None # small x, vert
        rightmost = None # big x, vert
        topmost = None # small y, hor
        bottommost = None # big y, hor
        for l in range(a):
            [[x1,y1,x2,y2]] = lines[l]
            newLine = Line(x1,x2,y1,y2)
            if newLine.orientation == 'horizontal':
                if topmost is None or newLine.y1 < topmost.y1:
                    topmost = newLine
                elif bottommost is None or newLine.y2 > bottommost.y2:
                    bottommost = newLine
                horizontal.append(newLine)
            elif newLine.orientation == 'vertical':
                if leftmost is None or newLine.x1 < leftmost.x1:
                    leftmost = newLine
                elif rightmost is None or newLine.x2 > rightmost.x2:
                    rightmost = newLine
                vertical.append(newLine)
        
        
        cv2.line(colorEdges, (topmost.x1, topmost.y1), (topmost.x2, topmost.y2), (0,0,255),2,cv2.LINE_AA)
        cv2.line(colorEdges, (bottommost.x1, bottommost.y1), (bottommost.x2, bottommost.y2), (0,0,255),2,cv2.LINE_AA)
        cv2.line(colorEdges, (rightmost.x1, rightmost.y1), (rightmost.x2, rightmost.y2), (0,0,255),2,cv2.LINE_AA)
        cv2.line(colorEdges, (leftmost.x1, leftmost.y1), (leftmost.x2, leftmost.y2), (0,0,255),2,cv2.LINE_AA)
        """
        for line in horizontal:
            cv2.line(colorEdges, (line.x1, line.y1), (line.x2, line.y2), (0,255,0),2,cv2.LINE_AA)
        for line in vertical:
            cv2.line(colorEdges, (line.x1, line.y1), (line.x2, line.y2), (0,255,0),2,cv2.LINE_AA)
        """
        return horizontal, vertical, (topmost, bottommost, leftmost, rightmost)

    def findCorners (self, horizontal, vertical, colorEdges):
        '''
        Finds corners at intersection of horizontal and vertical lines.
        helper function for corner detection
        '''

        # Find corners (intersections of lines)
        corners = []
        for v in vertical:
            for h in horizontal:
                s1,s2 = v.find_intersection(h)
                corners.append([s1,s2])

        # remove duplicate corners
        dedupeCorners = []
        for c in corners:
            matchingFlag = False
            for d in dedupeCorners:
                if math.sqrt((d[0]-c[0])*(d[0]-c[0]) + (d[1]-c[1])*(d[1]-c[1])) < 20:
                    matchingFlag = True
                    break
            if not matchingFlag:
                dedupeCorners.append(c)

        #for d in dedupeCorners:
            #cv2.circle(colorEdges, (d[0],d[1]), 10, (0,0,255))
        
        return dedupeCorners