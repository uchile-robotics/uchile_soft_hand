#!/bin/sh

# useful variables
_THIS_DIR="$(rospack find bender_hand)/shell"

alias bender_hwcheck_hand="rosrun bender_hand hw_check.py"

# Source shell tools OBSOLETE!
#. "$_THIS_DIR"/shell_tools.sh

unset _THIS_DIR