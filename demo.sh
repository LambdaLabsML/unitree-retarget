#!/bin/bash

case "$1" in
  dance)
    python run_retarget.py --mocap_file ./data/g1/dance1_subject2_rev.csv --music_file robot_merge2.wav --scale_arm 1.25
    ;;

  kill)
    sshpass -p 123 ssh unitree@192.168.123.164 pkill -f "aplay.*"
    ;;

  *)
    echo "Usage: $0 {dance|kill}"
    exit 1
esac
