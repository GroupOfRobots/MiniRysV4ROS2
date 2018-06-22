#!/usr/bin/env sh
# Generate .py files from Qt Designer's .ui files.

if [ $# -lt 1 ]; then
	echo "No arguments provided."
	exit 1
fi

while [ $# -gt 0 ]; do
	echo -n "Parsing ${1}... "
	pyuic5 -i 0 -x "${1}" -o "${1%%.ui}.py"
	echo "Done!"
	shift
done
