#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import web
from hemo_map import *
import os
import json

rootPath = '/home/ubuntu/hemo_code/new_code/wksp/src/hemo_webpy/'
contentPath = rootPath + 'static/app/'
        
urls = ( 
   '/', 'index',
   '/map', 'mapData',
   '/pos', 'botPos',
   '/req/(.+)', 'sendRequest',
   '/(css/.+)', 'resource',
   '/(js/.+)', 'resource',
   '/(bower_components/.+)', 'resource',
   '/(partials/.+)', 'resource',
   '/(favicon.ico)', 'resource'
)

app = web.application(urls, globals())

global m, destLoc
destLoc = 1
m = Map("levine.mp")

class index:
    def GET(self):
      with open (contentPath + 'index.html') as myfile:
        data = myfile.read()
        return data

class botPos:
    global curLoc, destLoc

    def GET(self):
        m = Map("/home/ubuntu/hemo_code/new_code/wksp/src/hemo_webpy/levine.mp")
        c = m.getLines()
        s = c[curLoc]
        d = c[destLoc]
        res_dic = {}

        if (c != None):
            res_dic["x"] = s[0]
            res_dic["y"] = s[1]
            res_dic["x2"] = d[0]
            res_dic["y2"] = d[1]

        return json.dumps(res_dic, sort_keys=True)

class mapData:
    def GET(self):
        m = Map("/home/ubuntu/hemo_code/new_code/wksp/src/hemo_webpy/levine.mp")
        map_line_data = m.getLines()
        dic = {}

        idx = 1;
        #{"tasks": {"1": {"idx": "1", "name": "one thing"}, "2": {"idx": "2", "name": "sfsdf"}, "3": {"idx": "3", "name": "this"}, "4": {"idx": "4", "name": "that?"}}}

        res_dic = {}
        res_dic["size"] = str(map_line_data[0])

        for item in map_line_data[1:]:
            line = {  "x1" : str(item[0]),
                      "y1" : str(item[1]),
                      "x2" : str(item[2]),
                      "y2" : str(item[3])
            }

            dic[str(idx)] = line
            idx += 1
            
            res_dic["lines"] = dic

        return json.dumps(res_dic, sort_keys=True)

class sendRequest:
  def POST(self, dest):
    # print dest
    ROS_INFO(dest);

class resource:
  def GET(self, url):
    with open (contentPath + url) as myfile:
            data = myfile.read()
            return data

def ROS_INFO(s):
    rospy.loginfo(s)

def start():

    rospy.init_node('hemo_webpy')

    ROS_INFO("hemo_webpy STARTED")

    app.run()
    
    rospy.spin();        

if __name__ == "__main__":
    # app.run()
    start()