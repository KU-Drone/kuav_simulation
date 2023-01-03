#!/usr/bin/env bash
XTERM=$(pidof xterm)
echo ${XTERM}
if [[ $XTERM = *[!\ ]* ]];then
    echo "a"
    kill -SIGINT ${XTERM} 
fi

MAVPROXY=$(pgrep -f mavproxy.py)
echo ${MAVPROXY}
if [[ $MAVPROXY = *[!\ ]* ]];then
    echo "b"
    kill -SIGKILL ${MAVPROXY} 
fi