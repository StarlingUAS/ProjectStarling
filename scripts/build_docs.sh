#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR"  >/dev/null

cp ../README.md ../docs
# sed -i 's/](docs\//](/g' README.md

mkdocs build -f ../mkdocs.yml

popd >/dev/null
