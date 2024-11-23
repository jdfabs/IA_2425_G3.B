class Border:
    def __init__(self):
        self.has_wall = False

class Cell:
    has_been_explored = False
    is_mold_here = False

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
        if self.butter_distance == 0:
            return 0
        if self.toaster_distance == 0:
            return 2
        if self.is_mold_here:
            return 1
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
                if cell_object == 0:
                    row_content += " B "  # Butter
                elif cell_object == 1:
                    row_content += " M "  # Mold
                elif cell_object == 2:
                    row_content += " T "  # Toaster
                elif cell_object == -1 and not self.getCell(x,y).butter_distance is None:
                    row_content += " "
                    row_content += str(self.getCell(x,y).butter_distance)
                    row_content += " "
                else:
                    row_content += "   "
            
                

            # Right border of the last cell in the row
            if self.getCell(x, size - 1).right_border.has_wall:
                row_content += "|"
            else:
                row_content += " "

            print(row_content)

        # Print the bottom border of the last row
        bottom_border = ""
        for y in range(size):
            bottom_border += "+"
            if self.getCell(size - 1, y).bottom_border.has_wall:
                bottom_border += "---"
            else:
                bottom_border += "   "
        bottom_border += "+"

        print(bottom_border)

