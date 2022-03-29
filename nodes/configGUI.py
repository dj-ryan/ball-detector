#!/usr/bin/env python
import roslib; roslib.load_manifest('ball_detector')
import rospy
import wx
from geometry_msgs.msg import Vector3

class configGUIFrame(wx.Frame):
    def __init__(self, parent, title):
        wx.Frame.__init__(self, parent, title=title, size=(500,200))
        panel = wx.Panel(self, 1)

        # Create the low HSV sliders
        self.sliderLowH = wx.Slider(panel, -1,
                                    rospy.get_param('thresh/low/h',0),
                                    0, 255, wx.DefaultPosition, (200, -1),
                           wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)
        self.sliderLowS = wx.Slider(panel, -1,
                                    rospy.get_param('thresh/low/s',0),
                                    0, 255, wx.DefaultPosition, (200, -1),
                           wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)
        self.sliderLowV = wx.Slider(panel, -1,
                                    rospy.get_param('thresh/low/v',0),
                                    0, 255, wx.DefaultPosition, (200, -1),
                           wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)

        #Add sliders to a box layout
        lowBox = wx.BoxSizer(wx.VERTICAL)
        lowBox.Add(wx.StaticText(panel,-1,'Low HSV Threshold'),0,wx.ALIGN_CENTER)

        tmpBox = wx.BoxSizer(wx.HORIZONTAL)
        tmpBox.Add(self.sliderLowH,1,wx.ALIGN_LEFT)
        tmpBox.Add(wx.StaticText(panel,-1,'H'),0,wx.ALIGN_CENTER|wx.TOP|wx.LEFT,15)
        lowBox.Add(tmpBox,1,wx.ALIGN_CENTER)

        tmpBox = wx.BoxSizer(wx.HORIZONTAL)
        tmpBox.Add(self.sliderLowS,1,wx.ALIGN_LEFT)
        tmpBox.Add(wx.StaticText(panel,-1,'S'),0,wx.ALIGN_CENTER|wx.TOP|wx.LEFT,15)
        lowBox.Add(tmpBox,1,wx.ALIGN_CENTER)

        tmpBox = wx.BoxSizer(wx.HORIZONTAL)
        tmpBox.Add(self.sliderLowV,1,wx.ALIGN_LEFT)
        tmpBox.Add(wx.StaticText(panel,-1,'V'),0,wx.ALIGN_RIGHT|wx.TOP|wx.LEFT,15)
        lowBox.Add(tmpBox,1,wx.ALIGN_CENTER)

        #Create the high HSV sliders
        self.sliderHighH = wx.Slider(panel, -1,
                                     rospy.get_param('thresh/high/h',0),
                                     0, 255, wx.DefaultPosition, (200, -1),
                           wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)
        self.sliderHighS = wx.Slider(panel, -1,
                                     rospy.get_param('thresh/high/s',0),
                                     0, 255, wx.DefaultPosition, (200, -1),
                           wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)
        self.sliderHighV = wx.Slider(panel, -1,
                                     rospy.get_param('thresh/high/v',0),
                                     0, 255, wx.DefaultPosition, (200, -1),
                           wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)

        # Add high HSV sliders to a layout
        highBox = wx.BoxSizer(wx.VERTICAL)
        highBox.Add(wx.StaticText(panel,-1,'High HSV Threshold'),0,wx.ALIGN_CENTER)
        highBox.Add(self.sliderHighH,1,wx.ALIGN_CENTER)
        highBox.Add(self.sliderHighS,1,wx.ALIGN_CENTER)
        highBox.Add(self.sliderHighV,1,wx.ALIGN_CENTER)

        # Add the slider boxes to the main box
        hBox = wx.BoxSizer(wx.HORIZONTAL)
        hBox.Add(lowBox,1)
        hBox.Add(highBox,1)

        # Add the final box
        panel.SetSizer(hBox)

        # Create the callback on adjustment
        self.Bind(wx.EVT_SLIDER,self.sliderMoved)

        # Create the ROS publisher
        self.threshHighPub = rospy.Publisher('thresh/high',Vector3)
        # Create the ROS publisher
        self.threshLowPub = rospy.Publisher('thresh/low',Vector3)

        # Make us visible
        self.Show(True)

    def sliderMoved(self,event):
        # Get the slider values
        lh = self.sliderLowH.GetValue()
        ls = self.sliderLowS.GetValue()
        lv = self.sliderLowV.GetValue()
        hh = self.sliderHighH.GetValue()
        hs = self.sliderHighS.GetValue()
        hv = self.sliderHighV.GetValue()
        rospy.loginfo('New setting: HSV Low: %d,%d,%d High: %d,%d,%d',
                      lh,ls,lv,hh,hs,hv)
        # Publish the Vector3 messages
        self.threshLowPub.publish(x=lh,y=ls,z=lv)
        self.threshHighPub.publish(x=hh,y=hs,z=hv)
        
def startGUI():
    app = wx.App(False)
    frame = configGUIFrame(None, "Configuration GUI")
    rospy.init_node('HSV_configGUI')
    app.MainLoop()


if __name__ == '__main__':
    try:
        startGUI()
    except rospy.ROSInterruptException: pass
