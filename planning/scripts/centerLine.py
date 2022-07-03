
#!/usr/bin/env python3

from fsd_common_msgs.msg import Waypoint
from generate import MapHandle


class CenterLine(MapHandle):
    def __init__(self):
        super(CenterLine, self).__init__()

    def run(self):
        self.publishWaypoints()

    def calculate_points(self, yellow, blue):
        p = Waypoint()
        p.position.x = (yellow.location.x + blue.location.x - 6) / 2.0
        # x-6 ---> offset for fsds simulator
        p.position.y = (yellow.location.y + blue.location.y) / 2.0

        # p.position.x = (yellow.position.x + blue.position.x) / 2.0
        # p.position.y = (yellow.position.y + blue.position.y) / 2.0
        p.position.z = 0.0
        self.waypoints_stamped.waypoints.append(p)
