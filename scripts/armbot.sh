#!/bin/bash

docker=true
[[ $docker = true ]] && dockerArg=docker || dockerArg=""

if [[ -z "$1" ]]; then
    echo "No argument supplied"
else
    ./scripts/extended/run.sh "$1" "$dockerArg"
fi
