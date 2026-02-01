#!/bin/bash
REF=${1-master} # branch or tag; defaults to 'master' if parameter 1 not present
REMOTE=pybind11 # just a name to identify the remote
REPO=git@github.com:pybind/pybind11.git # replace this with your repository URL
FOLDER=pybind11 # where to mount the subtree

git remote add $REMOTE --no-tags $REPO
if [[ -d $FOLDER ]]; then # update the existing subtree
    git subtree pull $REMOTE $REF --prefix=$FOLDER --squash -m "Merging '$REF' into '$FOLDER'"
else # add the subtree
    git subtree add  $REMOTE $REF --prefix=$FOLDER --squash -m "Merging '$REF' into '$FOLDER'"
fi
git remote remove $REMOTE