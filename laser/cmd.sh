#!/bin/sh
# python3 svg_to_ilda.py rec.svg -s 10 -t 100 --offset-y 0 --offset-x 50 --green 93

python3 svg_to_ilda_with_center.py rec.svg -s 10 -t 100 --offset-y 50 --offset-x 100 --green 103 --center-yaml "../pc_generation/center.yaml"