#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import web
from hemo_map import *
import os
import json

rootPath = '/home/ubuntu/hemo_code/new_code/wksp/src/hemo_webpy/'
contentPath = rootPath + 'static/'
        
urls = (
    '/', 'Index',
    '/static/(.*)', 'Images' #this is where the image folder is located....
)

### Templates
t_globals = {
    'datestr': web.datestr,
    'json_encode': json.dumps
}
render = web.template.render(rootPath + 'templates', base='base', globals=t_globals)


app = web.application(urls, globals())
m = Map("levine.mp")

class Index:
  def GET(self):
      """ Show page """  
      #return render.index(m)
      s = '\"\{ x1: 0, y1: 0, x2: 100, y2:100 \}\"'
      return render.index(s)

class Images:
    def GET(self,name):
      with open (contentPath + name) as myfile:
            data = myfile.read()
            return data
    
    
    
#        with open(os.path.join(contentPath, name), 'wb') as f:
#          return f
'''
        ext = name.split(".")[-1] # Gather extension

        cType = {
            "png":"images/png",
            "jpg":"images/jpeg",
            "gif":"images/gif",
            "ico":"images/x-icon"            }

        return open(contentPath+'/%s'%name,"rb").read() # Notice 'rb' for reading images
        if name in os.listdir(contentPath):  # Security
            web.header("Content-Type", cType[ext]) # Set the Header
            return open(contentPath+'/%s'%name,"rb").read() # Notice 'rb' for reading images
        else:
            raise web.notfound()
'''

def start():

    rospy.init_node('hemo_webpy')

    app.run()
    
    rospy.spin();        

if __name__ == "__main__":
    #app.run()
    start()