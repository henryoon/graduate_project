#!/bin/bash

for socket in /tmp/.X11-unix/X*; do
    display=":${socket##*/X}"
    if ! lsof "$socket" >/dev/null 2>&1; then
        echo "Killing unused X session: $display"
        pkill -f "Xvfb $display"
        rm -f "$socket"
    fi
done
