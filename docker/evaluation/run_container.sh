#!/bin/bash
rocker --nvidia --x11 --user --net host --privileged --volume output:/output -- aichallenge-eval
