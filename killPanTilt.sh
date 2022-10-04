#!/bin/bash
kill `ps aux | grep main.py | head -n 1 | awk {'print $2;'}`









