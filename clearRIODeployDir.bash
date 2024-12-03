#!/bin/bash

# Simple script which clears NamedCommands in AutoBuilder
#  and auto choices in SendableChooser object
#  by removing the `deploy` directory

REMOTE_USER="admin"
REMOTE_HOST="roboRIO-2059-frc.local"
REMOTE_FOLDER="/home/lvuser/deploy"

ssh ${REMOTE_USER}@${REMOTE_HOST} "rm -r ${REMOTE_FOLDER}"

# Check if the command succeeded
if [ $? -eq 0 ]; then
	echo "Command success."
else
	echo "Command failed."
fi
