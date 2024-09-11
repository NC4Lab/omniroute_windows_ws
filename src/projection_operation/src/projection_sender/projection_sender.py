import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayDimension
import numpy as np
import os

class ProjectionSender:
    def __init__(self):
        rospy.loginfo('Projection Sender node started')

        # Publisher for the 'projection_image' topic
        self.image_pub = rospy.Publisher('projection_image', Int32MultiArray, queue_size=10)

        # Sleep for 5 seconds before sending the test message
        rospy.loginfo("Waiting for 5 seconds before sending the test message...")
        rospy.sleep(10)

        # Send the hard-coded test image after the delay
        self.send_test_image_message()

        # Rate to publish at 10 Hz
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_projection_image()
            r.sleep()

    def setup_layout(self, rows, columns):
        """Helper function to set up the layout for a multi-dimensional array."""
        layout = []

        # Define rows
        row_dim = MultiArrayDimension()
        row_dim.label = "rows"
        row_dim.size = rows
        row_dim.stride = rows * columns  # Total number of elements (rows * columns)
        layout.append(row_dim)

        # Define columns
        col_dim = MultiArrayDimension()
        col_dim.label = "columns"
        col_dim.size = columns
        col_dim.stride = columns  # Number of elements per row
        layout.append(col_dim)

        return layout

    def send_test_image_message(self):
        """Send the hard-coded 10x10 test image as an Int32MultiArray message."""
        rospy.loginfo("Setting up and sending hard-coded test image")

        # Manually hard-code each row with specific values
        self.test_projection_image = [[0]*10 for _ in range(10)]
        self.test_projection_image[0] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
        self.test_projection_image[1] = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        self.test_projection_image[2] = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        self.test_projection_image[3] = [3, 3, 3, 3, 3, 3, 3, 3, 3, 3]
        self.test_projection_image[4] = [4, 4, 4, 4, 4, 4, 4, 4, 4, 4]
        self.test_projection_image[5] = [5, 5, 5, 5, 5, 5, 5, 5, 5, 5]
        self.test_projection_image[6] = [6, 6, 6, 6, 6, 6, 6, 6, 6, 6]
        self.test_projection_image[7] = [7, 7, 7, 7, 7, 7, 7, 7, 7, 7]
        self.test_projection_image[8] = [8, 8, 8, 8, 8, 8, 8, 8, 8, 8]
        self.test_projection_image[9] = [9, 9, 9, 9, 9, 9, 9, 9, 9, 9]

        # Create the Int32MultiArray message
        projection_data = Int32MultiArray()

        # Set up the layout using the helper function
        projection_data.layout.dim = self.setup_layout(10, 10)

        # Flatten the 10x10 array into a single list
        flat_data = [self.test_projection_image[i][j] for i in range(10) for j in range(10)]
        projection_data.data = flat_data

        # Publish the test message
        self.image_pub.publish(projection_data)

        # Log the sent message data
        rospy.loginfo("Sent the following hard-coded test data:")
        for i in range(10):
            for j in range(10):
                rospy.loginfo("Data[%d][%d] = %d", i, j, self.test_projection_image[i][j])

    def setup_arrays(self):
        """Set up the arrays to be sent over the 'projection_image' topic."""
        rospy.loginfo('Setting up arrays')

        # Example of setting up arrays
        self.projection_data = Int32MultiArray()

        # Set up the layout using the helper function
        self.projection_data.layout.dim = self.setup_layout(10, 10)

        # Initialize the array with dummy integer values for now
        self.projection_data.data = np.random.randint(0, 100, size=100).tolist()  # 100 integer elements (10x10)

    def publish_projection_image(self):
        """Publish the multi-array data to the 'projection_image' topic."""
        rospy.loginfo("Publishing projection data")

        # Update data if needed (optional logic to update data before publishing)
        self.projection_data.data = np.random.randint(0, 100, size=100).tolist()  # Random integers for example
        
        # Publish the data
        self.image_pub.publish(self.projection_data)


# Main function to start the node
if __name__ == '__main__':
    rospy.init_node('projection_sender')
    try:
        sender = ProjectionSender()
    except rospy.ROSInterruptException:
        pass
