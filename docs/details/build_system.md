# The Build System

At present the build system is based on Docker's `buildx bake`, using a `bake.hcl` file to configure the build. The
`docker-bake.hcl` file contains definitions for each image to be built and which platforms they are to be built for.

At the moment, `bake` doesn't have a way of setting dependencies between images so the images need to be built in the
correct order. At present, there are three groups defined in the `bake.hcl` file to set the ordering. These are named
`stage1` through `stage3`. The `build_local.sh` script calls `buildx bake` for each of these stages in turn, ensuring
that all dependencies are in place for the next stage of the build.

Another aspect of this lack of dependency tracking is that images that extend other images need to explicitly use the
the correct version. In order to keep this portable, Dockerfiles that depend on an existing image have had a `VERSION`
build argument added. This is appended to the end of the name of the parent Docker image. The default value is `latest`,
which is equivalent to leaving it blank in Docker tooling.

To create a set of images with matching tags, use an environment variable `VERSION`. This will be appended as a tag to
the images and passed as the `VERSION` build argument to dockerfiles that depend on other `starling-*` images.

Multiplatform support is more complicated. The local docker image store cannot handle multi-platform images so
`buildx`'s default `docker` driver cannot be used. `build_local_multiplatform.sh` deals with this by spawning a new
builder using the `docker-container` driver alongside a local container registry that the builder interacts with. This
significantly complicates things. **At present there is a problem with the ARM builds**

Similar to the `VERSION` argument outlined above, the use of a local registry requires a `REGISTRY` build argument be
used in the Dockerfiles. Again this is provided by the `bake.hcl` script. It defaults to blank, which is equivalent to
using Docker Hub. Once built, the images can be pulled from the local registry using a `localhost:5000/` prefix.

The `bake.hcl` script takes values from the environment to be passed on to the Dockerfiles. Two of these are the
`VERSION` and `REGISTRY` arguments outlined above. One other value can be supplied to the `bake.hcl` script: `NAMEDTAG`.
It defaults to blank. If set, all images will be tagged with both the tag specified by `VERSION` and that specified by
`NAMEDTAG`. This is primarily to ease the use of GitHub actions.

## The GitHub Actions Workflows

When a tag of the form `vX.Y.Z` is pushed to the repo, a workflow will be started. This workflow builds all the images
from the `bake.hcl` script, tags them with both the version tag and `:latest`, and pushes them to Docker Hub. A similar
workflow exists for development images. A tag of the form `vX.Y.Z-dev` will cause the images to be built and tagged with
both the tag name and `nightly`, before being pushed to DockerHub.

The actions workflows attempt to make use of `buildx`'s cache to repository. The `:cache` and `:cache-dev` tags are used
for each image for the caches. In order to do this, the `bake.hcl` file isn't really used in the action workflow so
keeping them in sync will unfortunately require some careful maintainence (Another action?). There's also likely the
opportunity to streamline the two workflows into one, which should reduce maintainence.
