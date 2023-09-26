import time
import requests
import rospy
from std_msgs.msg import Int32

# Replace with your OctoPrint API key
API_KEY = 'your_api_key'

# Replace with your OctoPrint address
OCTOPRINT_ADDRESS = 'http://your_octoprint_address/api/job'

# Set the polling interval (in seconds)
POLLING_INTERVAL = 60  # 60 seconds

def get_print_status():
    headers = {
        'X-Api-Key': API_KEY,
    }
    response = requests.get(OCTOPRINT_ADDRESS, headers=headers)
    response.raise_for_status()  # will raise an error if the HTTP request failed
    return response.json()['state']

def wait_for_print_to_complete():
    while True:
        status = get_print_status()
        if status == 'Operational':
            break
        time.sleep(POLLING_INTERVAL)

def publish_message():
    # Initialize the ROS node
    rospy.init_node('octoprint_notifier')

    # Create a Publisher object
    pub = rospy.Publisher('octoprint_status', Int32, queue_size=10)

    # Wait for the print to complete
    wait_for_print_to_complete()

    # Publish the message
    msg = Int32()
    msg.data = 1  # you can use any integer value to signify that the print is complete
    pub.publish(msg)

    # Keep the script alive until the node is manually terminated
    rospy.spin()

# Example usage
if __name__ == '__main__':
    publish_message()