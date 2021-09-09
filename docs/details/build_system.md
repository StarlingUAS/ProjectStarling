# Project Starling Development and Build System

[TOC]

## Development Workflow

As this project gets larger and releases get pushed out, the standard single master branch is not longer viable. Therefore we start using a more scalable (and safer) method of project management (read [this article](https://nvie.com/posts/a-successful-git-branching-model/) for more info).

* **master** branch is reserved solely for core releases and should always remain stable. Master should only be updated and modified via pull requests, and each pull request should correspond to a new tagged release. A release will then trigger the continuous integration github action (see below) to update all of the docker images.
* **dev** branch is for general development and should always remain ahead of the **master** branch. All feature branches should merge into **dev** when verified. When enough features have been added, **dev** can raise a PR to merge into master. For testing **dev** releases can be created to trigger dev builds on ":nightly" tags.

![development method](../img/git-modelx.png)


## Building

### Building locally and for ARM using buildx bake

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

To create a set of images with matching tags, use an environment variable `BAKE_VERSION`. This will be appended as a tag
to the images and passed as the `VERSION` build argument to dockerfiles that depend on other `starling-*` images.

Multiplatform support is more complicated. The local docker image store cannot handle multi-platform images so
`buildx`'s default `docker` driver cannot be used. `build_local_multiplatform.sh` deals with this by spawning a new
builder using the `docker-container` driver alongside a local container registry that the builder interacts with. This
significantly complicates things.

To setup your local machine to do the multiplatform builds, you need to install the required emulators for the `arm64`
builds. Luckily someone has already done the hard work. All that should be required is:

```
docker run --privileged --rm tonistiigi/binfmt --install arm64
```

Similar to the `VERSION` argument outlined above, the use of a local registry requires a `REGISTRY` build argument be
used in the Dockerfiles. Again this is provided by the `bake.hcl` script. It defaults to blank, which is equivalent to
using Docker Hub. Once built, the images can be pulled from the local registry using a `localhost:5000/` prefix.
Override by setting `BAKE_REGISTRY` in the environment before calling `bake`.

The `bake.hcl` script takes values from the environment to be passed on to the Dockerfiles. Two of these are the
`BAKE_VERSION` and `BAKE_REGISTRY` arguments outlined above. `BAKE_RELEASENAME` can also be supplied to the `bake.hcl`
script. `BAKE_RELEASENAME` defaults to blank. If it is set, all images will be tagged with both the tag specified by
`BAKE_VERSION` (or `latest` if that is not set) and that specified by `BAKE_RELEASENAME`.

Finally, there are some further options to control caching. The main options are `BAKE_CACHETO_NAME` and
`BAKE_CACHEFROM_NAME`, which by default are blank. These exists to allow local builds to be cached from online sources
and pushed to a local registry or to allow local builds to be used to populate the online caches. When they are blank,
no caches will be used. This allows for one-directional caching, _e.g._ refreshing the online caches from local builds.
Two further caching options are available: `BAKE_CACHETO_REGISTRY` and `BAKE_CACHEFROM_REGISTRY`. These control the
destination and source registries for the cache images. By default, they will be blank, equivalent to using Docker Hub.


### The GitHub Actions Workflows

#### Releases

When a tag of the form `vX.Y.Z` is pushed to the repo, a workflow will be started. This workflow builds all the images
from the `bake.hcl` script, tags them with both the version tag and `:latest`, and pushes them to Docker Hub.

#### Development Releases
A similar workflow exists for development images.  A tag of the form `vX.Y.Z-dev` will cause the images to be built and tagged with
both the tag name and `:nightly`, before being pushed to DockerHub. These additional tags are controlled by setting the
`BAKE_RELEASENAME` environment variable.

The actions workflows attempt to make use of `buildx`'s cache to repository. The `:cache` and `:cache-dev` tags are used
for each image for the caches. This is done by setting the `BAKE_CACHENAME` variable. There's likely the opportunity to
streamline the two workflows into one, which should reduce maintainence.