#!/bin/bash

YAML=$1
NS=$2

if [ -z "$1" ]; then
  echo "Usage: $(basename $0) <prompter config> [<namespace>]"
  echo Produces a JSON Grammar array for vosk from a prompter config yaml.
  echo It calls afterwards the vosk node service set_grammar if a namespace is provided.
  echo The Grammar can also be configured using a vosk node parameter.
  exit 1
fi

JSON="["$(cat $YAML| grep ^name| cut -d':' -f2| tr ' ' "\n"| grep -v '^$'| \
tr '[:upper:]' '[:lower:]'| sort -u| while read w; do echo -n "\"$w\","; done)'"[unk]"]'

echo $JSON

[ -n "$NS" ] || exit 0

echo calling service set_grammar
ros2 service call ${NS}/set_grammar voskros/srv/SetGrammar '{list:'$JSON'}'

