import cv2
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
            
