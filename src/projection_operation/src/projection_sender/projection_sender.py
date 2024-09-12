import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayDimension
import csv
import os

class ProjectionOperation:
    def __init__(self):
        rospy.loginfo('[ProjectionOperation:__init__] PROJECTION SENDER NODE STARTED')

        # Publisher for the 'projection_image' topic
        self.image_pub = rospy.Publisher('projection_image', Int32MultiArray, queue_size=10)

        # TEMP
        rospy.sleep(5)

        # Initialize image_config as a 10x8 array with default values
        self.image_config = [[0 for _ in range(8)] for _ in range(10)]

        # Read data from walls and floor CSV
        walls_csv_path = os.path.join(os.path.dirname(__file__), 'data', 'image_config', 'walls_cfg_0.csv')
        self.image_config = self.csv_read_and_store(self.image_config, walls_csv_path, "walls")

        floors_csv_path = os.path.join(os.path.dirname(__file__), 'data', 'image_config', 'floor_cfg_0.csv')
        self.image_config = self.csv_read_and_store(self.image_config, floors_csv_path, "floor")

        # Send the image configuration message
        self.publish_image_message(self.image_config)

        # Rate to publish at 10 Hz
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

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

    def csv_read_and_store(self, image_config, file_path, data_type):
        """
        Read the CSV and structure the data into either a 10x8 array for 'walls'
        or extract a single value for 'floor' and modify the given image_config.
        
        Args:
            image_config (list): A 10x8 list that will be modified in place.
            file_path (str): The path to the CSV file containing the data.
            data_type (str): A string that specifies whether to process the data as
                            'walls' or 'floor'. 
                            - 'walls': Updates the 10x8 array for wall configuration.
                            - 'floor': Updates a single value in the last entry of dim1 and first entry of dim2.

        Returns:
            list: 
                - If data_type is 'walls', returns the modified 10x8 list.
                - If data_type is 'floor', returns the updated list with floor value set.
        """
        
        with open(file_path, mode='r') as csvfile:
            csv_reader = csv.reader(csvfile)

            if data_type == "walls":
                next(csv_reader)  # Skip the header row
                # For walls, store data as a 10x8 array
                for row_idx, row in enumerate(csv_reader):
                    if row_idx < 10:
                        # Ignore the first column and take columns 1-8 (which are index 1 to 8 in 0-based indexing)
                        data_row = list(map(int, row[1:9]))
                        image_config[row_idx] = data_row

            elif data_type == "floor":
                first_row = next(csv_reader)  # Get the first data row
                floor_value = int(first_row[0])  # Read the value from the first column
                image_config[-1][0] = floor_value  # Store the value in the last entry of dim1 and first entry of dim2

            else:
                rospy.logwarn(f"[csv_read_and_store] Invalid data_type: {data_type}. Expected 'walls' or 'floor'.")
                return None

        return image_config

    def publish_image_message(self, image_config):
        """
        Send the data from the CSV as an Int32MultiArray message.
                
        Args:
            image_config (list): A 10x8 list that will be modified in place.
        """
        rospy.loginfo("[ProjectionOperation:publish_image_message] Sending CSV-based data")

        # Create the Int32MultiArray message
        projection_data = Int32MultiArray()

        # Set up the layout using the helper function (10x8 array)
        projection_data.layout.dim = self.setup_layout(10, 8)

        # Flatten the 10x8 array into a single list
        flat_data = [image_config[i][j] for i in range(10) for j in range(8)]
        projection_data.data = flat_data

        # Publish the CSV data message
        self.image_pub.publish(projection_data)

        # Log the sent message data
        rospy.loginfo("[ProjectionOperation:publish_image_message] Sent the following CSV-based data:")
        for i in range(10):
            rospy.loginfo("Data[%d] = %s", i, str(image_config[i]))

# Main function to start the node
if __name__ == '__main__':
    rospy.init_node('projection_sender')
    try:
        sender = ProjectionOperation()
    except rospy.ROSInterruptException:
        pass
