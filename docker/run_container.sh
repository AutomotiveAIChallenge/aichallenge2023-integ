#!/bin/bash

rocker --nvidia --x11 --user --net host --privileged --volume aichallenge:/aichallenge -- aichallenge-train
