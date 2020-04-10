#!/bin/bash
sudo renice -n -0 `ps -C transcoderd -o pid= `
sudo renice -n -0 `ps -C laterald -o pid= `
sudo renice -n -0 `ps -C controlsd -o pid= `
sudo renice -n 0 `ps -C influxd -o pid= `
sudo renice -n 0 `ps -C dashboard -o pid= `
sudo renice -n -0 `ps -C boardd -o pid= `
