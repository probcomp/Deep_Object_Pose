#!/usr/bin/env bash
set -u
set -e

wget https://derendering-data.s3.amazonaws.com/dope-weights/cracker_60.pth -O weights/cracker_60.pth
wget https://derendering-data.s3.amazonaws.com/dope-weights/mustard_60.pth -O weights/mustard_60.pth
