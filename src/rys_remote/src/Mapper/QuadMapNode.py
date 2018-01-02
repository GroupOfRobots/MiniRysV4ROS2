import math

class QuadMapNode(object):
	'''
	A quad-map tree, holding a single node of the map.
	Any subtree is equivalent in functionality to a whole tree.
	Holds either a number of scans performed on the node along with 'occupied' scans count or 4 sub-cells.
	'''

	def __init__(self, size, minSize, x = 0, y = 0, freeThreshold = 0.8, scanCount = 0):
		super(QuadMapNode, self).__init__()

		mapSizePower = math.log(size / minSize, 2)
		if mapSizePower != int(mapSizePower):
			raise ValueError('size/mapSize must be a power of 2')

		self.size = size
		self.minSize = minSize
		self.freeThreshold = freeThreshold
		self.x = x
		self.y = y

		# We'll use these over and over again, so calculate them once
		self.xMin = self.x - self.size / 2
		self.xMax = self.x + self.size / 2
		self.yMin = self.y - self.size / 2
		self.yMax = self.y + self.size / 2

		self.scanCount = scanCount
		self.occupiedScanCount = 0
		'''list of sub-cells, ccw starting with topright'''
		self.subCells = None

	def addScan(self, a, b, x0, x1):
		'''
		Process incoming occupancy scan.
		Scan is a line with y=ax+b equation spanning from x0 to x1 (where x1 is occupancy point!)
		If scan ends within this cell, it's an "cell occupied" one; if it only goes "through", it's a "cell free" one.
		'''

		# TODO: edge cases of a == 0 or a == Inf
		# TODO 2: proof-read

		# First, check whether the scan line goes through this cell
		yAtXMin = a * self.xMin + b
		yAtXMax = a * self.xMax + b
		if yAtXMin >= self.yMax and yAtXMax >= self.yMax:
			return
		if yAtXMin < self.yMin and yAtXMax < self.yMin:
			return

		if x0 < self.xMin and x1 < self.xMin:
			return
		if x0 >= self.xMax and x1 >= self.xMax:
			return

		y0 = a * x0 + b
		y1 = a * x1 + b
		if y0 >= self.yMax and y1 >= self.yMax:
			return
		if y0 < self.yMin and y1 < self.yMin:
			return

		# Second, decide whether it's a hit or miss scan and increase proper counters
		# If the scan 'ends' within this cell - it's a hit;
		if x1 >= self.xMin and x1 < self.xMax and y1 >= self.yMin and y1 < self.yMax:
			# Pass the scan into the subcells (if needed)
			if self.size > self.minSize:
				# If sub-cells were not initialized, do it now
				if self.subCells is None:
					topRightCell = QuadMapNode(self.size / 2, self.minSize, self.x + self.size / 4, self.y + self.size / 4, self.freeThreshold, 0)
					topLeftCell = QuadMapNode(self.size / 2, self.minSize, self.x - self.size / 4, self.y + self.size / 4, self.freeThreshold, 0)
					bottomLeftCell = QuadMapNode(self.size / 2, self.minSize, self.x - self.size / 4, self.y - self.size / 4, self.freeThreshold, 0)
					bottomRightCell = QuadMapNode(self.size / 2, self.minSize, self.x + self.size / 4, self.y - self.size / 4, self.freeThreshold, 0)
					self.subCells = [topRightCell, topLeftCell, bottomLeftCell, bottomRightCell]

				for cell in self.subCells:
					cell.addScan(a, b, x0, x1)
			else:
				# Add decay to already existing scans...
				self.scanCount = self.scanCount * 0.98
				self.occupiedScanCount = self.occupiedScanCount * 0.98
				# ... and add a new scan
				self.occupiedScanCount = self.occupiedScanCount + 1
		elif self.subCells is not None:
			# Else (it 'ends' outside - it's a miss), pass empty scan to sub-cells if they exist
			for cell in self.subCells:
				cell.addScan(a, b, x0, x1)
		else:
			# The scan is totally outside.
			pass

		# Also, increase scan count of this cell
		self.scanCount = self.scanCount + 1

	def isKnown(self, x = None, y = None):
		if self.scanCount == 0:
			return False

		if x is None and y is None:
			return self.isKnown(self.x, self.y)
		elif x is None or y is None:
			raise TypeError('missing either x or y')

		if self.subCells:
			if x >= self.x and y >= self.y:
				return self.subCells[0].isKnown(x, y)
			if x < self.x and y >= self.y:
				return self.subCells[1].isKnown(x, y)
			if x < self.x and y < self.y:
				return self.subCells[3].isKnown(x, y)
			if x >= self.x and y < self.y:
				return self.subCells[4].isKnown(x, y)

		return self.scanCount > 0

	def isFree(self, x = None, y = None):
		if self.scanCount == 0:
			return True

		if x is None and y is None:
			return self.isFree(self.x, self.y)
		elif x is None or y is None:
			raise TypeError('missing either x or y')

		if self.subCells:
			if x >= self.x and y >= self.y:
				return self.subCells[0].isFree(x, y)
			if x < self.x and y >= self.y:
				return self.subCells[1].isFree(x, y)
			if x < self.x and y < self.y:
				return self.subCells[3].isFree(x, y)
			if x >= self.x and y < self.y:
				return self.subCells[4].isFree(x, y)

		# return false if occupied scan count is higher than 1-threshold
		return self.isKnown() and (self.occupiedScanCount / self.scanCount) < (1.0 - self.freeThreshold)

	def getOccupancyMap(self):
		'''Build and return simplified occupancy map as a list of (x, y) tuples'''

		if self.subCells is None:
			if self.isFree():
				return list()
			else:
				return [(self.x, self.y, self.occupiedScanCount / self.scanCount)]

		occupied = list()
		for cell in self.subCells:
			subCellList = cell.getOccupancyMap()
			occupied.extend(subCellList)

		return occupied
