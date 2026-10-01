#!/usr/bin/env bash
free -m
ps -eo pcpu,pmem,comm --sort=-pcpu | head -8
docker stats --no-stream --format '{{.Name}} CPU={{.CPUPerc}} MEM={{.MemUsage}}'
