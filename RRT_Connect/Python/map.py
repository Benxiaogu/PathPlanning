class Map:
    def __init__(self,width,height):
        self.width = width
        self.height = height
        self.boundary = None
        self.obs_circle = None
        self.obs_rectangle = None
        self.generate()

    def generate(self):
        """
         Generate a map with obstacles.
        """
        x, y = self.width, self.height

        # boundary of environment:
        self.boundary = [ # (ox,oy,width,height)
            [0, 0, 1, y],
            [0, y, x, 1],
            [1, 0, x, 1],
            [x, 1, 1, y]
        ]

        # user-defined obstacles:
        self.obs_rectangle = [ # (ox,oy,width,height)
            [14, 12, 8, 2],
            [18, 22, 8, 3],
            [26, 7, 2, 12],
            [32, 14, 10, 2]
        ]

        self.obs_circle = [ # (ox,oy,radius)
            [7, 12, 3],
            [46, 20, 2],
            [15, 5, 2],
            [37, 7, 3],
            [37, 23, 3]
        ]