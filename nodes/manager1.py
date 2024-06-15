import rclpy
import chess
from rclpy.node import Node
from std_msgs.msg import String

class c_board():
    #to do:
    #figure out what to do if checkmate 
    board: chess.Board()
    w_captures: int
    b_captures: int
    Game_State: str
    def __init__(self):
        self.board = chess.Board()
        self.w_captures = 0
        self.b_captures = 0
        self.Game_State = "Finding_Game"
    def make_move(self, move):
        if isinstance(move, str): 
            self.update_move(self, move)
        else:
            cv_move = self.move_from_cv(self, move)
            self.update_move(self, cv_move)

    def update_move(self, move_str:str):

        #move comes in the form of original move, final move. 
        move = chess.Move.from_uci(move_str)
        if self.board.is_capture(move):
        #if piece is captured
            self.update_captures(self)
        
        self.board.push(move)
        
        if self.board.is_checkmate(self.board):
            self.Game_State = "Over"

        # goes last no matter what 

    

    def update_captures(self) -> None:
        #check black or white updates accordingly 
        if self.board.turn == chess.WHITE :
            self.w_captures += 1
        else:
            self.b_captures += 1
    def move_from_cv(self, move: list) -> str:
        start_move = ""
        end_move = ""
        for i in range(8): #  row, 
            for j in range(8): # column 
                piece = chess.board.piece_at(chess.square(j, i)) # piece can = None, piece.colour = chess.white, chess.black
                if not (move[i][j] == "white" and piece.colour == chess.WHITE) or (move[i][j] == "black" and piece.colour == chess.BLACK) or (move[i][j] == "none" and piece is None):
                # not (move matches)
                    # move = final 
                    # piece = original 
                    # find the move !!!!
                    if (j+1) == 1:
                        strj = "a"
                    elif (j+1) == 2:
                        strj = "b"
                    elif (j+1) == 3:
                        strj = "c"
                    elif (j+1) == 4:
                        strj = "d"
                    elif (j+1) == 5:
                        strj = "e"
                    elif (j+1) == 6:
                        strj = "f"
                    elif (j+1) == 7:
                        strj = "g"
                    else: 
                        strj = "h"


                    if move[i][j] == "none" :
                        start_move = strj + str(9-i)

                    else:
                        end_move = strj + str(9-i)

        return start_move + end_move



        
