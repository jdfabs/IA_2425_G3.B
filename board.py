import random

class Border:
    def __init__(self):
        self.has_wall = False

class Cell:
    has_been_explored = False
    is_mold_here = False
    is_robot_here = False
    
    is_butter_here = False
    butter_distance = None
    toaster_distance = None

    top_border = Border()
    bottom_border = Border()
    left_border = Border()
    right_border = Border()

    def setTopBorder(self, top_border):
        self.top_border = top_border
    def setBottomBorder(self, bottom_border):
        self.bottom_border = bottom_border
    def setLeftBorder(self, left_border):
        self.left_border = left_border
    def setRightBorder(self, right_border):
        self.right_border = right_border

    def getObjectInCell(self):
        if self.butter_distance == 0 or self.is_butter_here:
            return 0
        elif self.toaster_distance == 0:
            return 2
        elif self.is_mold_here:
            return 1
        elif self.is_robot_here:
            return 3
        
        return -1


    def printBorders(self):
        print(self.top_border.has_wall)
        print(self.bottom_border.has_wall)
        print(self.left_border.has_wall)
        print(self.right_border.has_wall)


class Board:
    def __init__(self):
        self.__matrix = [[Cell() for _ in range(6)] for _ in range(6)]

        # Initialize shared borders
        for x in range(6):
            for y in range(6):
                if x > 0:  # Share top border with the cell above
                    self.__matrix[x][y].setTopBorder(self.__matrix[x-1][y].bottom_border)
                else:  # Create a new top border for the top row
                    self.__matrix[x][y].top_border = Border()
                if y > 0:  # Share left border with the cell to the left
                    self.__matrix[x][y].setLeftBorder(self.__matrix[x][y-1].right_border)
                else:  # Create a new left border for the leftmost column
                    self.__matrix[x][y].left_border = Border()

                    # Create new bottom and right borders for every cell
                self.__matrix[x][y].bottom_border = Border()
                self.__matrix[x][y].right_border = Border()
        
        self.robotPreviousPosition = (0, 0)
        
        self.setupBoard()

    def getCell(self, x, y):
        if not 6 > x > -1 or not 6 > y > -1 :
            return None
        return self.__matrix[x][y]


    def setupBoard(self):
        choice = input("Enter 'manual', 'random' or 'empty': ").strip().lower()

        if choice[0] == "r":
            self.setupRandomBoard()
            self.calculateDistances()
        elif choice[0] == "m":
            self.setupManualBoard()
            self.calculateDistances()
        elif choice[0] == "e":
            self.setupEmptyBoard()
        else:
            print("Invalid choice. Please restart and choose either 'manual' or 'random'.")


    def setupEmptyBoard(self):
        # Ensure the board is cleared first
        for x in range(6):  # Loop through all rows
            for y in range(6):  # Loop through all columns
                cell = self.getCell(x, y)
                cell.is_robot_here = False
                cell.is_mold_here = False
                cell.is_butter_here = False
                cell.toaster_distance = None
                cell.butter_distance = None
                cell.top_border.has_wall = False
                cell.bottom_border.has_wall = False
                cell.left_border.has_wall = False
                cell.right_border.has_wall = False

        # Place the mold at position (5, 5)
        self.getCell(5, 5).is_mold_here = True

        # Place the robot at position (0, 0)
        self.getCell(0, 0).is_robot_here = True

        print("Empty board set up with robot at (0, 0) and mold at (5, 5).")


    def setupRandomBoard(self):
        # Generate exactly 6 random walls, avoiding outer borders
        walls_placed = 0
        while walls_placed < 6:
            x = random.randint(1, 4)  # Avoiding outer borders (1 to 4)
            y = random.randint(1, 4)
            direction = random.choice(["top", "bottom", "left", "right"])
            cell = self.getCell(x, y)

            if direction == "top" and not cell.top_border.has_wall:
                cell.top_border.has_wall = True
                walls_placed += 1
            elif direction == "bottom" and not cell.bottom_border.has_wall:
                cell.bottom_border.has_wall = True
                walls_placed += 1
            elif direction == "left" and not cell.left_border.has_wall:
                cell.left_border.has_wall = True
                walls_placed += 1
            elif direction == "right" and not cell.right_border.has_wall:
                cell.right_border.has_wall = True
                walls_placed += 1

        # Place the mold at (5, 5)
        mold_x, mold_y = 5, 5
        self.getCell(5, 5).is_mold_here = True
        
        # Place the robot at position (0, 0)
        self.getCell(0, 0).is_robot_here = True

        # Place the butter randomly, avoiding the mold and the robot's starting position (0, 0)
        while True:
            butter_x, butter_y = random.randint(0, 5), random.randint(0, 5)
            if (butter_x, butter_y) not in [(mold_x, mold_y), (0, 0)]:
                self.getCell(butter_x, butter_y).butter_distance = 0
                self.butterPreviousPosition = (butter_x, butter_y)
                self.getCell(butter_x, butter_y).is_butter_here = True
                break

        # Place the toaster randomly, avoiding the mold, butter, and the robot's starting position (0, 0)
        while True:
            toaster_x, toaster_y = random.randint(0, 5), random.randint(0, 5)
            if (toaster_x, toaster_y) not in [(mold_x, mold_y), (butter_x, butter_y), (0, 0)]:
                self.getCell(toaster_x, toaster_y).toaster_distance = 0
                break

        print("Random board generated!")


    def setupManualBoard(self):
        # Set mold at (5, 5)
        mold_x, mold_y = 5, 5
        self.getCell(mold_x, mold_y).is_mold_here = True
        print("Mold is set at position (5, 5).")
        
        # Place the robot at position (0, 0) as 'R'
        self.getCell(0, 0).is_robot_here = True

        # Add walls
        walls_placed = 0
        print("Add walls. Enter 'x y direction' (e.g., '2 3 top'). Type 'done' to finish.")
        while walls_placed < 6:
            user_input = input("Wall position (Walls placed: {walls_placed}/6): ")
            if user_input.lower() == "done":
                break
            try:
                x, y, direction = user_input.split()
                x, y = int(x), int(y)

                # Check if within valid range and not on outer borders
                if x in [0, 5] or y in [0, 5]:
                    print("Walls cannot be placed on outer borders. Try again.")
                    continue

                cell = self.getCell(x, y)
                if direction == "top" and not cell.top_border.has_wall:
                    cell.top_border.has_wall = True
                    walls_placed += 1
                elif direction == "bottom" and not cell.bottom_border.has_wall:
                    cell.bottom_border.has_wall = True
                    walls_placed += 1
                elif direction == "left" and not cell.left_border.has_wall:
                    cell.left_border.has_wall = True
                    walls_placed += 1
                elif direction == "right" and not cell.right_border.has_wall:
                    cell.right_border.has_wall = True
                    walls_placed += 1
                else:
                    print("Wall already exists in this direction. Try again.")
            except (ValueError, AttributeError):
                print("Invalid input. Use 'x y direction' format.")

        # Place butter
        print("Place the butter. Enter 'x y'.")
        while True:
            user_input = input("Butter position: ")
            try:
                butter_x, butter_y = map(int, user_input.split())
                if (butter_x, butter_y) not in [(mold_x, mold_y), (0, 0)]:
                    self.getCell(butter_x, butter_y).butter_distance = 0
                    break
                else:
                    print("Butter cannot overlap with the mold or the robot's starting position (0, 0). Try again.")
            except (ValueError, AttributeError):
                print("Invalid input. Use 'x y' format.")

        # Place toaster
        print("Place the toaster. Enter 'x y'.")
        while True:
            user_input = input("Toaster position: ")
            try:
                toaster_x, toaster_y = map(int, user_input.split())
                if (toaster_x, toaster_y) not in [(mold_x, mold_y), (butter_x, butter_y), (0, 0)]:
                    self.getCell(toaster_x, toaster_y).toaster_distance = 0
                    break
                else:
                    print("Toaster cannot overlap with the mold, butter, or the robot's starting position (0, 0). Try again.")
            except (ValueError, AttributeError):
                print("Invalid input. Use 'x y' format.")

        print("Manual board setup complete!")


    def calculateDistances(self):
        # Find positions of butter, toaster, and mold
        butter_position = None
        toaster_position = None
        mold_position = None

        for x in range(6):
            for y in range(6):
                cell = self.getCell(x, y)
                if cell.butter_distance == 0:
                    butter_position = (x, y)
                if cell.toaster_distance == 0:
                    toaster_position = (x, y)
                if cell.is_mold_here:
                    mold_position = (x, y)

        # Ensure all items are placed
        if not butter_position or not toaster_position or not mold_position:
            print("Error: Missing an item. Make sure butter, toaster, and mold are placed.")
            return

        # Reset distances and calculate new ones
        for x in range(6):
            for y in range(6):
                cell = self.getCell(x, y)
                # Reset all distances before recalculating
                cell.butter_distance = None
                cell.toaster_distance = None
                cell.mold_distance = None

                # Compute Manhattan distances
                cell.butter_distance = abs(x - butter_position[0]) + abs(y - butter_position[1])
                cell.toaster_distance = abs(x - toaster_position[0]) + abs(y - toaster_position[1])
                cell.mold_distance = abs(x - mold_position[0]) + abs(y - mold_position[1])


    def displayBoard(self):
        size = 6  # The board is 6x6

        for x in range(size):
            # Print the top border of each row
            top_border = ""
            for y in range(size):
                top_border += "+"
                if self.getCell(x, y).top_border.has_wall:
                    top_border += "---"  # Wall exists
                else:
                    top_border += "   "  # No wall
            top_border += "+"

            print(top_border)

            # Print the cell content and vertical borders
            row_content = ""
            for y in range(size):
                if self.getCell(x, y).left_border.has_wall:
                    row_content += "|"  # Wall exists
                else:
                    row_content += " "  # No wall

                # Determine the cell content
                cell_object = self.getCell(x, y).getObjectInCell()
                current_cell = self.getCell(x,y)
                if cell_object == 0:
                    row_content += " B"  # Butter
                elif cell_object == 3:
                    row_content += " R"  # Robot
                elif current_cell.is_mold_here:
                    row_content += " M"  # Mold
                elif cell_object == 2:
                    row_content += " T"  # Toaster
                elif cell_object == -1 and not self.getCell(x,y).butter_distance is None:
                    row_content += " "
                    #row_content += str(self.getCell(x,y).butter_distance)
                    row_content += " "
                    #pass
                else:
                    row_content += "  "
            

                # Add the right border of the last cell
                if self.getCell(x, size-1).right_border.has_wall:
                    row_content += "|"
                else:
                    row_content += " "

            print(row_content)

        # Print the bottom border for the last row
        bottom_border = ""
        for y in range(size):
            bottom_border += "+"
            if self.getCell(size - 1, y).bottom_border.has_wall:
                bottom_border += "---"  # Wall exists
            else:
                bottom_border += "   "  # No wall
        bottom_border += "+"

        print(bottom_border)


    def displayDistances(self):
        print("\nButter Distances:")
        for x in range(6):
            for y in range(6):
                cell = self.getCell(x, y)
                print("{cell.butter_distance or 'X':>3}", end=" ")
            print()

        print("\nToaster Distances:")
        for x in range(6):
            for y in range(6):
                cell = self.getCell(x, y)
                print("{cell.toaster_distance or 'X':>3}", end=" ")
            print()

        print("\nMold Distances:")
        for x in range(6):
            for y in range(6):
                cell = self.getCell(x, y)
                print("{cell.mold_distance or 'X':>3}", end=" ")
            print()

    def updateButterPosition(self, new_x, new_y):
        """Update the robot's position on the board."""
        # Unpack the previous position
        prev_x, prev_y = self.robotPreviousPosition
        
        # Clear the robot from the previous position
        self.getCell(prev_x, prev_y).is_butter_here = False
        # Set the robot at the new position
        self.getCell(new_x, new_y).is_butter_here = True
        
        # Update the robot's previous position
        self.butterPreviousPosition = (new_x, new_y)


    def updateRobotPosition(self, new_x, new_y):
        """Update the robot's position on the board."""
        # Unpack the previous position
        prev_x, prev_y = self.robotPreviousPosition
        
        # Clear the robot from the previous position
        self.getCell(prev_x, prev_y).is_robot_here = False
        
        # Set the robot at the new position
        self.getCell(new_x, new_y).is_robot_here = True
        
        # Update the robot's previous position
        self.robotPreviousPosition = (new_x, new_y)

    def updateMoldPosition(self,robot_x,robot_y):
        current_x, current_y = None ,None 
        for x in range(6):
            for y in range(6):
                 if self.getCell(x, y).is_mold_here:
                     current_x,current_y = x,y
                     break
            if current_x is not None:
                break
        if current_x is None or current_y is None:
            print("Não foi encontrado nenhum bolor no tabuleiro.")
            return
        
        directions = [
            ("Norte", -1, 0),    
            ("Sul", 1, 0),   
            ("Este", 0, 1),  
            ("Oeste", 0, -1)   
        ]

        possible_moves= []
        shortest_dis=float ('inf')
        
        for direction , dx, dy in directions :
            new_x, new_y = current_x + dx , current_y + dy
            if 0 <= new_x < 6 and 0 <= new_y < 6:
                print(self.getCell(new_x,new_y).is_butter_here)
                print(self.getCell(new_x, new_y).butter_distance)   
            #tentativa de implementar uma das regras se o mld chegar a manteiga 
                if self.getCell(new_x,new_y).is_butter_here: # MOLD TOUCHES BUTTER # ROBOT LOSES
                    print("Bolor encontrou a barra de manteiga em ("+str(new_x)+","+str(new_y+"). Jogo terminado."))
                    exit()
                elif self.getCell(new_x,new_y).is_robot_here: # MOLD TOUCHES ROBOT # ROBOT LOSES
                    print("Bolor encontrou o robo em ({new_x}, {new_y}). Jogo terminado.")
                    exit()
                elif self.getCell(new_x,new_y).toaster_distance == 0: #MOLD TOUCHES TOASTER # ROBOT WIN
                    print("Bolor encontrou a tostadeira em ({new_x}, {new_y}). Jogo terminado.")
                    exit()

            #calcula a distancia po robot
                distance = abs(new_x - robot_x) + abs(new_y - robot_y) #robot_x e _ y passado como parame tros 
                possible_moves.append((distance, direction, new_x, new_y))

        possible_moves.sort(key=lambda move: (move[0], [d[0] for d in directions].index(move[1])))
            
        if possible_moves:
            _, direction, new_x, new_y = possible_moves[0]
        # Atualizar a posição do bolor
            self.getCell(current_x, current_y).is_mold_here = False
            self.getCell(new_x, new_y).is_mold_here = True
            #print("Bolor movido para ({new_x}, {new_y}) na direção {direction}. Distância ao robô: {abs(new_x - robot_x) + abs(new_y - robot_y)}")
        else:
            print("Não há movimentos válidos. O bolor permanece em ({current_x}, {current_y}).")