import rospy
import time

def main():
    rospy.init_node('Node1')
    for ii in range(10):
        #this command will get time in second from roscore, very useful 
        current_time = rospy.Time.now().to_sec()

        print(f'{current_time}')
        time.sleep(1)

main()
