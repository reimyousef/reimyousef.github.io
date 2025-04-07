"""
Implements Minimax and Alpha-Beta search with heuristic evaluation,
state caching, and move ordering.
"""

import sys
import time
import random
from othello_shared import find_lines, get_possible_moves, get_score, play_move

cache = {}

def compute_utility(board, color):
    """
    Utility function for terminal states: difference in disk count.
    """
    dark, light = get_score(board)
    return dark - light if color == 1 else light - dark

def compute_heuristic(board, color):
    """
    Heuristic evaluation combining:
    - Corner control
    - Edge occupancy
    - Mobility (move count difference)
    - Piece count
    Adjusts weights based on game stage.
    """
    size = len(board)
    opponent = -color

    CORNER_WEIGHT, EDGE_WEIGHT, MOBILITY_WEIGHT, PIECE_WEIGHT = 25, 5, 10, 1
    corners = [(0, 0), (0, size - 1), (size - 1, 0), (size - 1, size - 1)]
    corner_score = sum(CORNER_WEIGHT for x, y in corners if board[x][y] == color)

    edge_score = 0
    for i in range(1, size - 1):
        for x, y in [(0, i), (size - 1, i), (i, 0), (i, size - 1)]:
            if board[x][y] == color:
                edge_score += EDGE_WEIGHT

    mobility = len(get_possible_moves(board, color)) - len(get_possible_moves(board, opponent))
    mobility_score = MOBILITY_WEIGHT * mobility

    my_pieces = sum(row.count(color) for row in board)
    opponent_pieces = sum(row.count(opponent) for row in board)
    piece_score = PIECE_WEIGHT * (my_pieces - opponent_pieces)

    filled = my_pieces + opponent_pieces
    board_cells = size * size
    if filled < 0.2 * board_cells:
        weights = (1, 2, 3, 0)  # Early game
    elif filled < 0.6 * board_cells:
        weights = (2, 2, 2, 1)  # Mid game
    else:
        weights = (3, 1, 1, 2)  # Endgame

    return (weights[0] * corner_score +
            weights[1] * edge_score +
            weights[2] * mobility_score +
            weights[3] * piece_score)

def set_cached_state(board, best_move, utility):
    cache[str(board)] = (best_move, utility)

def get_cached_state(board):
    return cache.get(str(board))

# --- MINIMAX & ALPHABETA SEARCH ---

def minimax_max_node(board, color, limit, caching):
    moves = get_possible_moves(board, color)
    if not moves or limit == 0:
        return None, compute_utility(board, color)

    if caching:
        cached = get_cached_state(board)
        if cached:
            return cached

    max_utility = -sys.maxsize
    best_move = None

    for move in moves:
        next_board = play_move(board, color, *move)
        _, utility = minimax_min_node(next_board, color, limit - 1, caching)
        if utility > max_utility:
            max_utility, best_move = utility, move

    if caching:
        set_cached_state(board, best_move, max_utility)

    return best_move, max_utility

def minimax_min_node(board, color, limit, caching):
    opponent = 1 if color == 2 else 2
    moves = get_possible_moves(board, opponent)
    if not moves or limit == 0:
        return None, compute_utility(board, color)

    if caching:
        cached = get_cached_state(board)
        if cached:
            return cached

    min_utility = sys.maxsize
    best_move = None

    for move in moves:
        next_board = play_move(board, opponent, *move)
        _, utility = minimax_max_node(next_board, color, limit - 1, caching)
        if utility < min_utility:
            min_utility, best_move = utility, move

    if caching:
        set_cached_state(board, best_move, min_utility)

    return best_move, min_utility

def select_move_minimax(board, color, limit, caching=0):
    return minimax_max_node(board, color, limit, caching)[0]

# --- ALPHA-BETA SEARCH (with ordering) ---

def select_move_alphabeta(board, color, limit, caching=0, ordering=0):
    alpha = -sys.maxsize
    beta = sys.maxsize
    return alphabeta_max_node(board, color, alpha, beta, limit, caching, ordering)[0]

def order_moves(board, moves, color, reverse=False):
    """
    Order moves based on the utility of the resulting board state.
    """
    move_utility = []
    for move in moves:
        new_board = play_move(board, color, move[0], move[1])
        utility = compute_utility(new_board, color)
        move_utility.append((move, utility))
    
    # Sort moves by utility (higher utility first for max, lower for min)
    move_utility.sort(key=lambda x: x[1], reverse=reverse)
    return [move for move, _ in move_utility]

def alphabeta_min_node(board, color, alpha, beta, limit, caching = 0, ordering = 0):
    # IMPLEMENT!
    """
    A helper function for alpha-beta that finds the lowest possible utility (don't forget to utilize and update alpha and beta!)
    """
    opponent_color = 1 if color == 2 else 2 
    moves = get_possible_moves(board, opponent_color)
    cached_value = get_cached_state(board)

    if not moves or limit == 0:
        return None, compute_utility(board, color) 
    if cached_value is not None and caching == 1:
        return cached_value
    
    # if ordering == 1:
    #     moves = order_moves(board, moves, color, reverse=False)  # Sort moves for min node
    
    min_utility = sys.maxsize  # Start with a very large number
    best_move = None
    
    if ordering == 1:
        moves.sort(key=lambda move: compute_utility(play_move(board, color, move[0], move[1]), color))
        
    for move in moves:
        if ordering == 1 and beta <= alpha: 
            break
        new_board = play_move(board, opponent_color, move[0], move[1])
        _, max_utility = alphabeta_max_node(new_board, color, alpha, beta, limit-1, caching , ordering )
        if max_utility < min_utility:
            min_utility = max_utility 
            best_move = move
        if min_utility < beta:
            beta = min_utility
        if beta<=alpha:
            break
    
    if caching == 1:
        set_cached_state(board, best_move, min_utility)
    return  best_move, min_utility

def alphabeta_max_node(board, color, alpha, beta, limit, caching = 0, ordering = 0):
    # IMPLEMENT!
    """
    A helper function for alpha-beta that finds the highest possible utility (don't forget to utilize and update alpha and beta!)
    """
    moves = get_possible_moves(board, color)
    cached_value = get_cached_state(board)

    if not moves or limit == 0:
        return None, compute_utility(board, color)  
    if cached_value is not None and caching == 1:
        return cached_value
    
    max_utility = -sys.maxsize  # Start with a very large number
    best_move = None
    
    if ordering == 1:
        moves.sort(key=lambda move: compute_utility(play_move(board, color, move[0], move[1]), color), reverse=True)
    
    for move in moves:
        if ordering == 1 and alpha >= beta:
            break
        new_board = play_move(board, color, move[0], move[1])
        _, utility = alphabeta_min_node(new_board, color, alpha, beta, limit-1, caching, ordering)
        if utility > max_utility:
            max_utility = utility
            best_move = move
            
        if max_utility>alpha:
            alpha=max_utility
            
        if alpha >=beta:
            break
            
    if caching == 1:
        set_cached_state(board, best_move, max_utility)
    return best_move, max_utility



####################################################
def run_ai():
    """
    This function establishes communication with the game manager.
    It first introduces itself and receives its color.
    Then it repeatedly receives the current score and current board state until the game is over.
    """
    print("Othello AI") # First line is the name of this AI
    arguments = input().split(",")
    
    color = int(arguments[0]) # Player color: 1 for dark (goes first), 2 for light. 
    limit = int(arguments[1]) # Depth limit
    minimax = int(arguments[2]) # Minimax or alpha beta
    caching = int(arguments[3]) # Caching 
    ordering = int(arguments[4]) # Node-ordering (for alpha-beta only)

    if (minimax == 1): eprint("Running MINIMAX")
    else: eprint("Running ALPHA-BETA")

    if (caching == 1): eprint("State Caching is ON")
    else: eprint("State Caching is OFF")

    if (ordering == 1): eprint("Node Ordering is ON")
    else: eprint("Node Ordering is OFF")

    if (limit == -1): eprint("Depth Limit is OFF")
    else: eprint("Depth Limit is ", limit)

    if (minimax == 1 and ordering == 1): eprint("Node Ordering should have no impact on Minimax")

    while True: # This is the main loop
        # Read in the current game status, for example:
        # "SCORE 2 2" or "FINAL 33 31" if the game is over.
        # The first number is the score for player 1 (dark), the second for player 2 (light)
        next_input = input()
        status, dark_score_s, light_score_s = next_input.strip().split()
        dark_score = int(dark_score_s)
        light_score = int(light_score_s)

        if status == "FINAL": # Game is over.
            print
        else:
            board = eval(input()) # Read in the input and turn it into a Python
                                  # object. The format is a list of rows. The
                                  # squares in each row are represented by
                                  # 0 : empty square
                                  # 1 : dark disk (player 1)
                                  # 2 : light disk (player 2)

            # Select the move and send it to the manager
            if (minimax == 1): # run this if the minimax flag is given
                movei, movej = select_move_minimax(board, color, limit, caching)
            else: # else run alphabeta
                movei, movej = select_move_alphabeta(board, color, limit, caching, ordering)
            
            print("{} {}".format(movei, movej))

if __name__ == "__main__":
    run_ai()
