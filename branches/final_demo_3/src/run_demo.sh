#!/bin/sh
rm morph lib*.so *.cfg
ln -s demo${1}/* .
./run.sh
