#!/bin/bash
# Deploy Cyberia Server via GitHub Actions workflow dispatch
set -e

cd /home/dd/engine/cyberia-server
underpost push . underpostnet/cyberia-server
gh workflow run cyberia-server.cd.yml -R underpostnet/cyberia-server -f job=deploy
