#!/bin/bash

SCRIPTSDIR="${BASH_SOURCE%/*}"
ROOTDIR="$SCRIPTSDIR/.."
DOCSDIR="$SCRIPTSDIR/../docs"
SYSTEMDIR="$SCRIPTSDIR/../system"
DEPLOYMENTDIR="$SCRIPTSDIR/../deployment"
CONTROLLERSDIR="$SCRIPTSDIR/../controllers"
SIMULATORDIR="$SCRIPTSDIR/../simulator"

# Specify READMEs to copy into source file directories
# sed -i 's/](docs\//](/g' README.md
cp $DOCSDIR/README.md $ROOTDIR/README.md
sed -i 's/](\//](\/docs\//g' $ROOTDIR/README.md
cp $DOCSDIR/details/kubernetes-dashboard.md $DEPLOYMENTDIR/README.md
cp $DOCSDIR/guide/example-controller.md $CONTROLLERSDIR/README.md
cp $DOCSDIR/details/starling-mavros.md $SYSTEMDIR/mavros/README.md
# cp $DOCSDIR/details/starling-ui.md $SYSTEMDIR/ui/README.md

mkdocs build -f $ROOTDIR/mkdocs.yml
