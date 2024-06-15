import json

def board_to_string(board):
    return json.dumps(board)

def string_to_board(board_str):
    return json.loads(board_str)

def main():
    # Example 2D list representing the board
    boardList = [
        ['black', 'black', 'black', 'black', 'black', 'black', 'black', 'black'],
        ['black', 'black', 'black', 'black', 'black', 'black', 'black', 'black'],
        ['none', 'none', 'none', 'none', 'none', 'none', 'none', 'none'],
        ['none', 'none', 'none', 'none', 'none', 'none', 'none', 'none'],
        ['none', 'none', 'none', 'none', 'none', 'none', 'none', 'none'],
        ['none', 'white', 'none', 'none', 'none', 'none', 'none', 'none'],
        ['white', 'none', 'white', 'white', 'white', 'white', 'white', 'white'],
        ['white', 'white', 'white', 'white', 'white', 'white', 'white', 'white']
    ]

    # Convert 2D list to string
    board_str = board_to_string(boardList)
    print("Board as string:", board_str)

    # Convert string back to 2D list
    new_boardList = string_to_board(board_str)
    print("Board as 2D list:", new_boardList)






if __name__ == "__main__":
    main()
