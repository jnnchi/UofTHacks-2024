import rclpy
import berserk, time
from pub import MinimalPublisher
from sub import MinimalSubscriber
API_TOKEN = "lip_uqXQtfhqr0lFoClLwWxU"

session = berserk.TokenSession(API_TOKEN)
boardClient = berserk.clients.Board(session=session)
client = berserk.Client(session=session)

rclpy.init(args=None)
publisher = MinimalPublisher(nodeName="shareOnlinePub", topic="shareOnline")
rate = publisher.create_rate(1)
subscriber = MinimalSubscriber(nodeName="shareBoardSub", topic="shareBoard")
rate2 = subscriber.create_rate(1)

#create game


#game = client.stream_game_state()


def gameHandler(boardClient : berserk.clients.Board, gameId : str, client : berserk.Client):
    # gameplay loop
    gameState = boardClient.stream_game_state(gameId)
    for event in gameState:
        print(f"GAME STATE ***** \n {event}")
        gameStatus = event['state']['status']
        while gameStatus == "started":
            # game object
            for retry in range(3):
                try:
                    for games in client.games.get_ongoing():
                        if games['gameId'] == gameId:
                            game = games
                    print(f"ONGOING GAME ***** \n {game}")
                    break
                except:             
                    print("API timeout, retrying...")
                    time.sleep(3)

            #check turn
            if (game['isMyTurn']):
                # board turn; update online player's turn
                moveOnline = game['lastMove']
                publisher.timer_callback(moveOnline)
                print(f"P2 last moved {moveOnline}")
                # send board's turn
                moveBoard = None
                while moveBoard is None:
                    rclpy.spin_once(subscriber)
                    moveBoard = subscriber.received
                    print(f"moveboard {moveBoard}")
                print(f"Now playing board: {moveBoard}")
                boardClient.make_move(gameId, moveBoard)
                moveBoard = None
            else:
                # online player turn; update board's turn

                print(f"Board last moved {game['lastMove']}")


            #refresh status
            for retry in range(3):
                try:                   
                    gameStatus = next(boardClient.stream_game_state(gameId))['state']['status']
                    break
                except:
                    print("API timeout, retrying...")    
                    time.sleep(3)
            time.sleep(10)      

        


# game stream loop
while True:
    for event in boardClient.stream_incoming_events():
        print(event)
        if (event['type'] == "challenge"):
            challengeId = event['challenge']['id']
            client.challenges.accept(challengeId)
        elif (event['type'] == "gameStart"):
            gameId = event['game']['gameId']
            boardColor = event['game']['color']
            gameHandler(boardClient, gameId, client)

