import time
import requests
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int8

# Replace with your OctoPrint API key
API_KEY = 'your_api_key'

# Replace with your OctoPrint address
OCTOPRINT_ADDRESS = 'http://your_octoprint_address/api/job'

# Set the polling interval (in seconds)
POLLING_INTERVAL = 60  # 60 seconds

printer_id = 0

def get_print_status():
    headers = {
        'X-Api-Key': API_KEY,
    }
    response = requests.get(OCTOPRINT_ADDRESS, headers=headers)
    response.raise_for_status()  # will raise an error if the HTTP request failed
    return response.json()['state']

def wait_for_print_to_complete():
    while True:
        #status = get_print_status()
        #if status == 'Operational':
        #    break
        print("Printer To Remove")
        global printer_id
        printer_id = int(input('--->'))
        if printer_id != 0:
            break
        time.sleep(POLLING_INTERVAL)

def publish_message():
    # Initialize the ROS node
    rospy.init_node('octoprint_notifier')
    # Create a Publisher object
    pub_status = rospy.Publisher('/octoprint_status', Bool, queue_size=10)
    pub_id = rospy.Publisher('/octoprint_finished_printers' ,Int8,queue_size=10)
    # Wait for the print to complete
    wait_for_print_to_complete()
    # Publish the message
    msg = Bool()
    msg.data = True
    pub_status.publish(msg)
    
    msg = Int8()
    msg.data = printer_id
    pub_id.publish(msg)
    
    # Keep the script alive until the node is manually terminated
    rospy.spin()

# Example usage
if __name__ == '__main__':
    publish_message()