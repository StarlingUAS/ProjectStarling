# Project Starling Development and Build System

[TOC]

## Development Workflow

As this project gets larger and releases get pushed out, the standard single master branch is not longer viable.
Therefore we start using a more scalable (and safer) method of project management (read
[this article](https://nvie.com/posts/a-successful-git-branching-model/) for more info).

* **master** branch is reserved solely for core releases and should always remain stable. Master should only be updated
and modified via pull requests, and each pull request should correspond to a new tagged release. A release will then
trigger the continuous integration github action (see below) to update all of the docker images.

* **dev** branch is for general development and should always remain ahead of the **master** branch. All feature
branches should merge into **dev** when verified. When enough features have been added, **dev** can raise a PR to merge
into master. For testing **dev** releases can be created to trigger dev builds on ":nightly" tags.

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
A similar workflow exists for development images.  A tag of the form `vX.Y.Z-dev` will cause the images to be built and
tagged with both the tag name and `:nightly`, before being pushed to DockerHub. These additional tags are controlled by
setting the `BAKE_RELEASENAME` environment variable.

The actions workflows attempt to make use of `buildx`'s cache to repository. The `:cache` and `:cache-dev` tags are used
for each image for the caches. This is done by setting the `BAKE_CACHENAME` variable. There's likely the opportunity to
streamline the two workflows into one, which should reduce maintainence.

#### Images on DockerHub

The set of images are automatically updated on DockerHub. Each image will have a set of tags:

 - `:latest` tracking the most recent push to `master` branch
 - `:nightly` tracking the most recent push to the `dev` branch
 - `:vX.Y.Z` fixed release tags
 - `:${BRANCH}` tracking most recent push to PR branches while active

#### Updating the cache

The builds can be run locally to update the cache if GitHub is timing out. `starling-mavros` is usually the culprit.
The command below will update the dev caches on GHCR, using the existing dev cache as a base.

```bash
BAKE_REGISTRY=ghcr.io/ BAKE_CACHETO_REGISTRY=ghcr.io/ BAKE_PREFIX=starlinguas BAKE_CACHEFROM_NAME=cache-dev BAKE_CACHETO_NAME=cache-dev docker buildx bake -f buildtools/docker-bake.hcl starling-mavros
```

## Testing

Starling now has the beginning of an automated test suite. Tests are written using the
[Bash Automated Testing System (BATS)](https://github.com/bats-core/bats-core) and are located in `./tests`. To run the
tests, install `bats` using the instructions from
[its docs](https://bats-core.readthedocs.io/en/stable/installation.html) and run `bats -r tests` from the main Starling
directory.

The GitHub workflow that builds the images also runs any tests defined for an image before pushing it up to DockerHub.
The current setup ensures all per-image tests pass before a combined set is uploaded at once.

### Organisation

Tests for a particular image should be put in a folder with the same name as that image (specifically that image's
target name in the `docker-bake.hcl` file). Any folder heirarchy above this is ignored when the workflow searches for
tests. For ease of finding tests for an image, the recommendation is to mirror the main repository structure, *i.e.* put
`system/*` image tests in `tests/system/*`. There is also a `tests/integration` folder that contains tests that are to
be run against a completed set of images. At present these are not part of the automated workflow but can be used
locally.

### Writing Tests

> The BATS documentation has some information on
[writing BATS tests](https://bats-core.readthedocs.io/en/stable/writing-tests.html).

There are a few different "styles" of test in the suite. These include:
- "Single-shot"
  These are tests that can be run by replacing an image's `CMD` through `docker run` call to check that the starting
  state of a container is as expected. Some examples of this type can be found in
  `tests/system/starling-mavros/test_vehicle_namespace.bats`
- "Runtime"
  These tests are run within the context of a running container to check that the correct things are running. This
  typically inlcludes starting a container and using `docker exec` to check some internal container state. An example
  can be found in `tests/system/starling-mavros/test_args.bats`. The `setup
- "External"
  Like "runtime" tests these need a running container, but they are able to run without needing access to the container
  internals. Simple cases can be structured in a similar fashion to the "runtime" test example. However, if the test
  itself needs tools, it may be necessary to run the test itself from an image. The `tests/integration/px4_basic` test
  provides an example of this which is more fully explained below.

#### `tests/integration/px4_basic`

This test setup is relatively complicated. It is intended to test that a set of MAVROS, SITL and simulator containers
can sucessfully communicate with each other. It does this by using BATS to run `docker-compose` with the
`--exit-code-from` argument. This argument causes Compose to stop all other containers when a specified container exits
and return the specified container's exit code as its own. This allows BATS to treat Compose as a simple test function
which returns 0 on success.

The Compose file is used to both define the setup of the other containers in the test, and to point to a Dockerfile that
provides the instructions to build the image that acts as the test function. This test image can be any Docker image. In
this case it is based on a ROS image to provide some ROS tooling within the container. The Dockerfile here just installs
some required ROS message types and copies in a test script setting it as the default command.

When the test image is run by Compose, it runs the test script and returns the result. Here the result is based on
checking the contents of a ROS message. If the test passes, the result propogates as the return code of the container,
which is in turn the return code of the `docker-compose` call, which finally results in BATS detecting and logging a
test result.
