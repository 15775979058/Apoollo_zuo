#!/usr/bin/env bash
grep -q uvcvideo /etc/modules
if [ $? -eq 1 ]; then
    echo "uvcvideo clock=realtime" | sudo tee -a /etc/modules
fi
