#!/bin/bash
# This defines some bash helpers

list_targets()
{
    # This script lists all the targets available
    
    # Empty list of targets
    AVAILABLE_TARGETS=""
    
    targets=$(ls targets/ | grep '\.sh$')
    # Iterate all the files and fill list of targets without the extension
    for f in $targets; do
        AVAILABLE_TARGETS="${f%.*},$AVAILABLE_TARGETS"
        echo "$AVAILABLE_TARGETS"
    done
}