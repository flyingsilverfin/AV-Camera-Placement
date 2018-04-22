class Road(object):

    def __init__(self, path, width=3.0, side_offset=1.5):
        """ Define a road with a width, wide of road offset etc.
        
        Default to road width of 3m - common in cities
        Side offest = 1.5m is estimate of pavement width/building offsets
            Used for as a camera placement constraint
        """


        self.path = path
        self.width = width
        self.side_offset = side_offset



    def get_path_tracker(self):
        """ Returns an encapsulated path that holds some state """

        
