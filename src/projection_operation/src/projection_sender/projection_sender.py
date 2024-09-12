import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayDimension
import csv
import os

class ProjectionSender:
    def __init__(self):
        rospy.loginfo('[ProjectionSender:__init__] PROJECTION SENDER NODE STARTED')

        # Publisher for the 'projection_image' topic
        self.image_pub = rospy.Publisher('projection_image', Int32MultiArray, queue_size=10)

        # Sleep for before sending the CSV-based message
        rospy.loginfo("[ProjectionSender:__init__] Waiting for 10 seconds before sending the CSV-based message...")
        rospy.sleep(5)

        # Read data from CSV
        csv_path = os.path.join(os.path.dirname(__file__), 'data', 'wall_cofig', 'test2.csv')
        self.csv_image_config = self.read_csv_and_structure(csv_path)

        # Send the image configuration message
        self.send_image_message(self.csv_image_config)

        # Rate to publish at 10 Hz
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

    def setup_layout(self, dim1, dim2, dim3):
        """Helper function to set up the layout for a multi-dimensional array."""
        layout = []

        # Define first dimension (depth)
        dim1_layout = MultiArrayDimension()
        dim1_layout.label = "dim1"
        dim1_layout.size = dim1
        dim1_layout.stride = dim1 * dim2 * dim3
        layout.append(dim1_layout)

        # Define second dimension (rows)
        dim2_layout = MultiArrayDimension()
        dim2_layout.label = "dim2"
        dim2_layout.size = dim2
        dim2_layout.stride = dim2 * dim3
        layout.append(dim2_layout)

        # Define third dimension (columns)
        dim3_layout = MultiArrayDimension()
        dim3_layout.label = "dim3"
        dim3_layout.size = dim3
        dim3_layout.stride = dim3
        layout.append(dim3_layout)

        return layout

    def read_csv_and_structure(self, file_path):
        """Read the CSV and structure the data into a 4x10x8 array."""
        structured_data = []
        with open(file_path, mode='r') as csvfile:
            csv_reader = csv.reader(csvfile)
            next(csv_reader)  # Skip the header row

            # Initialize empty layers to store 10 rows at a time
            current_layer = []
            row_count = 0

            for row in csv_reader:
                # Ignore the first two columns and take columns 2-9 (which are index 2 to 9 in 0-based indexing)
                data_row = list(map(int, row[2:10]))

                # Add the row to the current layer
                current_layer.append(data_row)
                row_count += 1

                # Once we have 10 rows, add the layer to the structured data
                if row_count == 10:
                    structured_data.append(current_layer)
                    current_layer = []  # Reset for the next layer
                    row_count = 0

        return structured_data

    def send_image_message(self, csv_image_config):
        """Send the data from the CSV as an Int32MultiArray message."""
        rospy.loginfo("[ProjectionSender:send_csv_image_message] Sending CSV-based data")

        # Create the Int32MultiArray message
        projection_data = Int32MultiArray()

        # Set up the layout using the helper function (4x10x8 array)
        projection_data.layout.dim = self.setup_layout(4, 10, 8)

        # Flatten the 4x10x8 array into a single list
        flat_data = [csv_image_config[layer][i][j] for layer in range(4) for i in range(10) for j in range(8)]
        projection_data.data = flat_data

        # Publish the CSV data message
        self.image_pub.publish(projection_data)

        # Log the sent message data
        rospy.loginfo("[ProjectionSender:send_csv_image_message] Sent the following CSV-based data:")
        for layer in range(4):
            for i in range(10):
                rospy.loginfo("Layer[%d] Data[%d] = %s", layer, i, str(csv_image_config[layer][i]))

# Main function to start the node
if __name__ == '__main__':
    rospy.init_node('projection_sender')
    try:
        sender = ProjectionSender()
    except rospy.ROSInterruptException:
        pass
