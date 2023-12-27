#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

docker build -t osrf/car_demo:$(git rev-parse --abbrev-ref HEAD) $DIR
