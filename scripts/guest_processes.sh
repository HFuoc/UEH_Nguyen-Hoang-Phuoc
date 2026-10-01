#!/usr/bin/env bash
ps -eo pid,ppid,etime,args | grep -E 'crc-.*sh|evaluate.sh|verify_sensors.py|gzserver' | head -30
