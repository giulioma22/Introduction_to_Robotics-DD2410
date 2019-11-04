#!/usr/bin/env python3

"""
    # Giulio Martinelli
    # 19960222T351
    # giulioma@kth.se
"""

# Python standard library
from math import cos, sin, atan2, fabs, pi

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()
        
        robot_x = pose.pose.position.x
        robot_y = pose.pose.position.y
        
        #Calculating the rectangle for the updates
        min_ind_x = 500
        min_ind_y = 500
        max_ind_x = -500
        max_ind_y = -500
        
        free_list = []
        occupied_list = []   
        
        #Every range of one state of the robot
        for i in range(len(scan.ranges)):

            if scan.range_min < scan.ranges[i] < scan.range_max:
                tot_x = - origin.position.x + robot_x + scan.ranges[i] * cos(robot_yaw + scan.angle_min + scan.angle_increment * i)
                tot_y = - origin.position.y + robot_y + scan.ranges[i] * sin(robot_yaw + scan.angle_min + scan.angle_increment * i)
                
                index_x = int(tot_x / resolution)
                index_y = int(tot_y / resolution)
                
                if self.is_in_bounds(grid_map, tot_x, tot_y) is True:
                    
                    occupied_list.append([index_x, index_y])
                       
                    start = (int((-origin.position.x + robot_x) / resolution), int((-origin.position.y + robot_y) / resolution))
                    end = (index_x, index_y)
                    trace = self.raytrace(start, end)
                    for j in range(len(trace)):
                        (trace_x, trace_y) = trace[j]
                        free_list.append([trace_x, trace_y])
                        
                #Minimum x, y indeces
                if index_x < min_ind_x:
                    min_ind_x = index_x
                if index_y < min_ind_y:
                    min_ind_y = index_y
                    
                #Maximum x, y indeces    
                if index_x > max_ind_x:
                    max_ind_x = index_x
                if index_y > max_ind_y:
                    max_ind_y = index_y
                
        #Free spots added to grid            
        for k in range(len(free_list)):
            self.add_to_map(grid_map, free_list[k][0], free_list[k][1], self.free_space)
        
        #Occupied spots added to grid            
        for l in range(len(occupied_list)):
            self.add_to_map(grid_map, occupied_list[l][0], occupied_list[l][1], self.occupied_space)
  
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = min_ind_x
        # The minimum y index in 'grid_map' that has been updated
        update.y = min_ind_y
        # Maximum x index - minimum x index + 1 (width rectangle)
        update.width = max_ind_x - min_ind_x + 1
        # Maximum y index - minimum y index + 1 (height rectangle)
        update.height = max_ind_y - min_ind_y + 1
        # The map data inside the rectangle, in row-major order.
        update.data = []
        
        #Send the data for the rectangle
        for j in range(update.height):
            for i in range(update.width):
                update.data.append(grid_map[min_ind_x + i, min_ind_y + j])                
        
        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """
        
        circle_ind = [[5,0],[2,1],[3,1],[4,1],[5,1],[6,1],[7,1],[8,1],\
                     [1,2],[2,2],[3,2],[4,2],[5,2],[6,2],[7,2],[8,2],[9,2],\
                     [1,3],[2,3],[3,3],[4,3],[5,3],[6,3],[7,3],[8,3],[9,3],\
                     [1,4],[2,4],[3,4],[4,4],[5,4],[6,4],[7,4],[8,4],[9,4],\
                     [0,5],[1,5],[2,5],[3,5],[4,5],[5,5],[6,5],[7,5],[8,5],[9,5],[10,5],\
                     [1,6],[2,6],[3,6],[4,6],[5,6],[6,6],[7,6],[8,6],[9,6],\
                     [1,7],[2,7],[3,7],[4,7],[5,7],[6,7],[7,7],[8,7],[9,7],\
                     [1,8],[2,8],[3,8],[4,8],[5,8],[6,8],[7,8],[8,8],[9,8],\
                     [2,9],[3,9],[4,9],[5,9],[6,9],[7,9],[8,9],[5,10]]
                    
        for j in range(grid_map.get_height()):
            for i in range(grid_map.get_width()):
                
                if grid_map[i, j] == self.occupied_space:
                    for k in range(len(circle_ind)):
                        add_x = i - self.radius + circle_ind[k][0]
                        add_y = j - self.radius + circle_ind[k][1]
                        if not grid_map[add_x, add_y] == self.occupied_space:
                            self.add_to_map(grid_map, add_x, add_y, self.c_space)

        # Return the inflated map
        return grid_map
