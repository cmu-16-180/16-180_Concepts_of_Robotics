#!/bin/bash

# Requires ImageMagick's convert

for f in *.png ; do
  convert $f -resize x150 "thumb_$f"
done
