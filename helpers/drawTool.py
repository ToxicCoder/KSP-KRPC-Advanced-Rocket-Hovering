import time
from math import sin, cos, pi, cosh
from colorsys import hsv_to_rgb
from numpy import clip
import krpc

class helpers:
	def __init__(self, conn, ref_frame):
		self.conn = conn
		self.vessel = self.conn.space_center.active_vessel
		self.body = self.vessel.orbit.body
		self.drawing = self.conn.drawing
		self.ref_frame = ref_frame
	
	def drawAxes(self, length):
		self.lineX = self.drawing.add_line((0, 0, 0), (length, 0, 0), self.ref_frame)
		self.lineY = self.drawing.add_line((0, 0, 0), (0, length, 0), self.ref_frame)
		self.lineZ = self.drawing.add_line((0, 0, 0), (0, 0, length), self.ref_frame)
		
		self.lineX.color = (255, 0, 0)
		self.lineY.color = (0, 255, 0)
		self.lineZ.color = (0, 0, 255)