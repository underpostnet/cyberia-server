#!/bin/bash
# Script to deploy Cyberia Server via SSH

cd /home/dd/cyberia-server
git reset
underpost cmt . cd ssh-cyberia-server --empty
underpost push . underpostnet/cyberia-server
