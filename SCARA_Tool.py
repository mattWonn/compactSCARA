#! /usr/bin/python3
#-*-coding: utf-8 -*-

class SCARA_Tool (object, metaclass = ABCMeta):
    """
    Tool on the end of L2, does whatever a tool does.
    A tool has to have an offset from tool0
    tool 0 is defined as the bottom of the Z axis
    a tool needs:
    a radial offset in degrees from axis of L2
    a length offset from end of L2
    a Z-offset from bottom Z-axis positioner
     
    """
    
    #abstact methods each tool class must implement
    @abstractmethod
    def __init__ (self, initData):
        """
        init data is a dictionary with at least these fields
        """
        self.radialOffset = initData.radialOffset
        self.lengthOffset = initData.lengthOffset
        self.zOffset = initData.zOffset
        pass
        
    @abstractmethod
    def setup (self):
        pass
         
    @abstractmethod
    def run (self, command):
        pass
        """
        command could be 0 for open 1 for close, could be anything you like
        """
