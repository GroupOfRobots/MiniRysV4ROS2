#!/usr/bin/env python3

import sys
from UI import RysRemoteUI

def main(args = None):
	if args is None:
		args = sys.argv

	ui = RysRemoteUI('rys_remote')
	sys.exit(ui.exec_(args))

if __name__ == '__main__':
	main()