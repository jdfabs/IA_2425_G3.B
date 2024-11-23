class Border:
    def __init__(self):
        self.__has_wall = False
    def setWall(self, has_wall):
        self.__has_wall = has_wall
    def hasWall(self):
        return self.__has_wall


class Cell:
    def __init__(self):
        self.__data = [-1, -1, -1,-1]
        self.__top_border = None
        self.__bottom_border = None
        self.__left_border = None
        self.__right_border = None

    #getters and setters
    def getMoldDistance(self):
        return self.__data[0]
    def getButterDistance(self):
        return self.__data[1]
    def getToasterDistance(self):
        return self.__data[2]
    def getObjectInCell(self):
        return self.__data[3]

    def getTopBorder(self):
        return self.__top_border
    def getBottomBorder(self):
        return self.__bottom_border
    def getLeftBorder(self):
        return self.__left_border
    def getRightBorder(self):
        return self.__right_border

    def setMoldDistance(self, distance): # -1 <= dist <= 2
        if distance < -1 or distance > 2:
            print("Invalid mold distance!")
            return
        self.__data[0] = distance
    def setButterDistance(self, distance): # 0 <= dist <= 10
        if distance < 0 or distance > 10:
            print("Invalid butter distance!")
            return
        self.__data[1] = distance
    def setToasterDistance(self, distance): # -1 <= dist <= 1
        if distance < -1 or distance > 1:
            print("Invalid toaster distance!")
            return
        self.__data[2] = distance
    def setObjectInCell(self, object_in_cell): # -1 NOTHING, 0 BUTTER, 1 MOLD, 2 TOASTER
        if object_in_cell < -1 or object_in_cell > 2:
            print("Invalid object!")
            return
        self.__data[3] = object_in_cell

    def initTopBorder(self):
        self.__top_border = Border()
    def initBottomBorder(self):
        self.__bottom_border = Border()
    def initLeftBorder(self):
        self.__left_border = Border()
    def initRightBorder(self):
        self.__right_border = Border()

    def setTopBorder(self, top_border):
        self.__top_border = top_border
    def setBottomBorder(self, bottom_border):
        self.__bottom_border = bottom_border
    def setLeftBorder(self, left_border):
        self.__left_border = left_border
    def setRightBorder(self, right_border):
        self.__right_border = right_border



    def printBorders(self):
        print(self.getTopBorder().hasWall())
        print(self.getBottomBorder().hasWall())
        print(self.getLeftBorder().hasWall())
        print(self.getRightBorder().hasWall())


class Board:
    def __init__(self):
        self.__matrix = [[Cell() for _ in range(6)] for _ in range(6)]

        # Initialize shared borders
        for x in range(6):
            for y in range(6):
                if x > 0:  # Share top border with the cell above
                    self.__matrix[x][y].setTopBorder(self.__matrix[x-1][y].getBottomBorder())
                else:  # Create a new top border for the top row
                    self.__matrix[x][y].initTopBorder()

                if y > 0:  # Share left border with the cell to the left
                    self.__matrix[x][y].setLeftBorder(self.__matrix[x][y-1].getRightBorder())
                else:  # Create a new left border for the leftmost column
                    self.__matrix[x][y].initLeftBorder()

                # Create new bottom and right borders for every cell
                self.__matrix[x][y].initBottomBorder()
                self.__matrix[x][y].initRightBorder()

    def getCell(self, x, y):
        if not 6 > x > -1 or not 6 > y > -1 :
            return None
        
        return self.__matrix[x][y]




    def displayBoard(self):
        size = 6  # The board is 6x6

        for x in range(size):
            # Print the top border of each row
            top_border = ""
            for y in range(size):
                top_border += "+"
                if self.getCell(x, y).getTopBorder().hasWall():
                    top_border += "---"  # Wall exists
                else:
                    top_border += "   "  # No wall
            top_border += "+"

            print(top_border)

            # Print the cell content and vertical borders
            row_content = ""
            for y in range(size):
                if self.getCell(x, y).getLeftBorder().hasWall():
                    row_content += "|"  # Wall exists
                else:
                    row_content += " "  # No wall

                # Determine the cell content
                cell_object = self.getCell(x, y).getObjectInCell()
                if cell_object == 0:
                    row_content += " B "  # Butter
                elif cell_object == 1:
                    row_content += " M "  # Mold
                elif cell_object == 2:
                    row_content += " T "  # Toaster
                elif cell_object == -1 and not self.getCell(x,y).getButterDistance() == -1:
                    row_content += " "
                    row_content += str(self.getCell(x,y).getButterDistance())
                    row_content += " "
                else:
                    row_content += "   "
            
                

            # Right border of the last cell in the row
            if self.getCell(x, size - 1).getRightBorder().hasWall():
                row_content += "|"
            else:
                row_content += " "

            print(row_content)

        # Print the bottom border of the last row
        bottom_border = ""
        for y in range(size):
            bottom_border += "+"
            if self.getCell(size - 1, y).getBottomBorder().hasWall():
                bottom_border += "---"
            else:
                bottom_border += "   "
        bottom_border += "+"

        print(bottom_border)

