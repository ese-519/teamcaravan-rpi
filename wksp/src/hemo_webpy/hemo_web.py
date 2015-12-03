#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import web
        
urls = (
    '/(.*)', 'hello'
)
app = web.application(urls, globals())

def start():

    rospy.init_node('hemo_webpy')

    app.run()
    
    rospy.spin();
    

class hello:        
    def GET(self, name):
        if not name: 
            name = 'World'
        rospy.loginfo("Some response");
        return 'Hello, ' + name + '!'

if __name__ == "__main__":
    #app.run()
    start()