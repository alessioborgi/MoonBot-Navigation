#!/usr/bin/env bash
#

# ssh robot@ev3dev 'echo "move 20" > /home/robot/cmdfifo'

# Usage: ./send_cmd.sh <command> [args...]
#   e.g. ./send_cmd.sh move 20
#        ./send_cmd.sh steer  30
#        ./send_cmd.sh stop

EV3_HOST="ssh robot@ev3dev"
PIPE="/home/robot/cmdfifo"

if [ -z "$1" ]; then
  echo "Usage: $0 <cmd> [args...]"; exit 1
fi

# assemble the message
MSG="$1"
shift
for a in "$@"; do
  MSG="$MSG $a"
done

# send it
ssh "$EV3_HOST" "echo \"$MSG\" > $PIPE"
