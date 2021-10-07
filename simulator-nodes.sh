#! /usr/bin/env bash

# This script spawns the necessary ROS nodes for the UT AUTOmata
# simulator in a tmux session.
#
# All command-line flags are passed to the simulator.
#
# Usage:
#   $ ./simulator-nodes.sh [--localize] [--ws-only]
#
# To inspect the session:
#   $ tmux attach -t simulator-nodes
#   - To cycle through each window:
#       CTRL+B, n
#   - To detach from the session:
#       CTRL+B, d
#
# To kill the session:
#   $ tmux kill-session -t simulator-nodes

# Flags should be mutually exclusive

set -e

SESSION="simulator-nodes"

tmux new-session -d -s "$SESSION"

tmux rename-window -t 0 "roscore"

tmux send-keys -t "roscore" "roscore" C-m

if [ "$1" != "--ws-only" ]; then
    tmux new-window -t "${SESSION}:1" -n "simulator"
    tmux send-keys -t "simulator" "roscd ut_automata && ./bin/simulator $@" C-m
fi

tmux new-window -t "${SESSION}:2" -n "websocket"
tmux send-keys -t "websocket" "roscd ut_automata && ./bin/websocket" C-m
