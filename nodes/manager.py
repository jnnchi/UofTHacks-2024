import rclpy
import chess, json, time
from pub import MinimalPublisher
from sub import MinimalSubscriber


def print_board(board):
    print(board)

def index_to_square(rank_index, file_index):
    ranks = '87654321'
    files = 'abcdefgh'
    return files[file_index] + ranks[rank_index]


def move_from_cv(board, inputStr):

    # str to list
    input = json.loads(inputStr)

    for i in range(8): #  row, 
        for j in range(8): # column 
            piece = board.piece_at(chess.square(j, i)) # piece can = None, piece.colour = chess.white, chess.black
            if not ((input[7-i][j] == "white" and piece is not None and piece.color == chess.WHITE) or (input[7-i][j] == "black" and piece is not None and piece.color == chess.BLACK) or (input[7-i][j] == "none" and piece is None)):
                if (input[7-i][j] == "none"):
                    print(input[7-i][j])
                    sq1 = index_to_square(7-i, j)
                else:
                    print(input[7-i][j])
                    sq2 = index_to_square(7-i, j)
    return f"{sq1}{sq2}"


rclpy.init(args=None)
publisher = MinimalPublisher(nodeName="shareBoardPub", topic="shareBoard")
rate = publisher.create_rate(1)
subscriber = MinimalSubscriber(nodeName="shareOnlineSub", topic="shareOnline")
rate2 = subscriber.create_rate(1)
                 


def main():
    board = chess.Board()
    print("Starting board:")
    print_board(board)

    while not board.is_game_over():

        subInput = None
        while subInput == None or subInput == "":
            rclpy.spin_once(subscriber)
            subInput = subscriber.received
            print(f"subInput {subInput}")
        boardListInput = json.dumps(subInput)

        uci_input = boardListInput
        if len(uci_input) != 4:
            uci_move = move_from_cv(board, uci_input)
            publisher.timer_callback(uci_move)
        else:
            uci_move = uci_input

        try:
            print(uci_move)
            move = chess.Move.from_uci(uci_move)
            if move in board.legal_moves:
                board.push(move)
                print("Move accepted.")
                print("Updated board:")
                print_board(board)

                # Check if there's a capture
                if board.is_capture(move):
                    print("Capture happened!")

            else:
                print("Invalid move. Try again.")
                time.sleep(10)

        except ValueError:
            print("Invalid move format. Try again.")
            time.sleep(10)

    print("Game over.")
    print("Result: ", board.result())

if __name__ == "__main__":
    main()