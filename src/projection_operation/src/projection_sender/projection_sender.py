#!/usr/bin/env python
# Class for publishing commands to the projection system

# Custom Imports
#from shared_utils.maze_debug import MazeDB

# ROS Imports
import rospy
import rospkg
from std_msgs.msg import Int32MultiArray, MultiArrayDimension

# Other
import csv
import os

class ProjectionOperation:
    # ------------------------ CLASS VARIABLES ------------------------
    def __init__(self):
        rospy.loginfo('[ProjectionOperation:__init__] PROJECTION SENDER NODE STARTED')
        self.package_path = rospkg.RosPack().get_path('projection_operation')
        self.num_chambers = 9  # Number of chambers
        self.num_surfaces = 9  # Number of surfaces (8 walls + 1 floor)
        self.blank_image_config = [[0 for _ in range(self.num_surfaces)] for _ in range(self.num_chambers)]

        # Publisher for the 'projection_image' topic
        self.image_pub = rospy.Publisher('projection_image', Int32MultiArray, queue_size=10)
        rospy.sleep(1)  # Allow some time for the publisher to be set up

        # Initialize the node (if not already initialized)
        if not rospy.core.is_initialized():
            rospy.init_node('projection_operation_node', anonymous=True)

        # Initialize image_config with default values (0)
        self.image_config = self.blank_image_config

        # # self.image_config[chamber][surface] = image_index  # Set the image index for the specified chamber and surface
        self.set_floor_image(2) 
        self.set_all_walls_image(1)

        # Send the image configuration message
        self.publish_image_message(self.image_config)

        # Rate to publish at 10 Hz
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()
    
    def set_all_walls_image(self, image_index):
        """
        Set all walls in all chambers to the specified image index.
        
        Args:
            image_index (int): The index of the image to set for all walls.
        """
        for chamber in range(self.num_chambers):
            for wall in range(8):
                self.image_config[chamber][wall] = image_index
    
    def set_wall_image(self, chamber, wall, image_index):
        if (wall < 0 or wall > 7) or (chamber < 0 or chamber > 8):
            rospy.logwarn(f"[projection_sender:set_wall_image] Invalid chamber {chamber} or wall {wall}. Must be in range [0, 8] and [0, 7] respectively.")
            return
        
        # Set the image index for the specified chamber and wall
        self.image_config[chamber][wall] = image_index
    
    def set_floor_image(self, image_index):
        for chamber in range(9):
            self.image_config[chamber][8] = image_index

    def setup_layout(self, dim1, dim2):
        """Helper function to set up the layout for a 2-dimensional array."""
        layout = []

        # Define first dimension (rows)
        dim1_layout = MultiArrayDimension()
        dim1_layout.label = "rows"
        dim1_layout.size = dim1
        dim1_layout.stride = dim1 * dim2
        layout.append(dim1_layout)

        # Define second dimension (columns)
        dim2_layout = MultiArrayDimension()
        dim2_layout.label = "columns"
        dim2_layout.size = dim2
        dim2_layout.stride = dim2
        layout.append(dim2_layout)

        return layout
    
    def set_config_from_csv(self, image_config, file_path, data_type):
        image_config = self.blank_image_config
        csv_path = os.path.join(self.package_path, '..', '..', 
                                'data', 'csv_configs', 'blank_config.csv')
        with open(csv_path, mode='r') as csvfile:
            csv_reader = csv.reader(csvfile)

            rdr = csv.reader(filter(lambda row: row[0]!='#', csv_reader))
            data = [row for row in rdr]

            if len(data) != self.num_chambers:
                rospy.logwarn(f"[projection_sender:set_config_from_csv] Expected {self.num_chambers} rows, got {len(data)}")
                return
            for i, row in enumerate(data):
                if len(row) != self.num_surfaces:
                    rospy.logwarn(f"[projection_sender:set_config_from_csv] Expected {self.num_surfaces} columns, got {len(row)} in row {i}")
                    return
                
                for j in range(self.num_surfaces):
                    try:
                        image_config[i][j] = int(row[j])
                    except ValueError:
                        rospy.logwarn(f"[projection_sender:set_config_from_csv] Invalid value '{row[j]}' at row {i}, column {j}")
                        return
        
        self.image_config = image_config

    def publish_image_message(self, image_config):
        """
        Send the data from the CSV as an Int32MultiArray message.
                
        Args:
            image_config (list): A 10x8 list that will be modified in place.
        """
        # Create the Int32MultiArray message
        projection_data = Int32MultiArray()
        projection_data.layout.dim = self.setup_layout(9, 9)  

        # Flatten the 9x9 array into a single list
        flat_data = [image_config[i][j] for i in range(9) for j in range(9)]
        projection_data.data = flat_data

        # Publish the CSV data message
        self.image_pub.publish(projection_data)

        # Log the sent message data
        rospy.loginfo("[projection_sender:publish_image_message] Sent the following data:")
        for i in range(9):
            rospy.loginfo("Data[%d] = %s", i, str(image_config[i]))


# Main function to start the node
if __name__ == '__main__':
    rospy.init_node('projection_sender')
    try:
        sender = ProjectionOperation()
    except rospy.ROSInterruptException:
        pass
