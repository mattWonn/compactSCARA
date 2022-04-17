#! /usr/bin/python3
#-*-coding: utf-8 -*-

class SCARA_Tool_Pen (SCARA_Tool):
    """
    Just a pen, in a holder. Used for drawing on a surface
    """
    
    def __init__ (self, initDict):
         super().__init__ (initDict)

    def setup (self):
        pass
         
    def run (self, runData):
        pass
        """
        Pen does not do anything here
        """
