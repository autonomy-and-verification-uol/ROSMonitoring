#!/usr/bin/env bash

# get dir of this script
# https://stackoverflow.com/a/246128/1202636
here="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# specify monitor alias path for modules imported by the spec (like deep_subdict)
swipl -p monitor="$here" "$here"/ws_monitor.pl -- "$@"
