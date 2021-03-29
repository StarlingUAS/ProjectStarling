# The Build System

At present the build system is based on Docker's `buildx bake`, using a `bake.hcl` file to configure the build.

At the moment, `bake` doesn't have a way of setting dependencies between images so the images need to be built in the
correct order. At present, there are three groups defined in the `bake.hcl` file to set the ordering. These are named
`stage1` through `stage3`. The `build_images.sh` script calls `buildx bake` for each of these stages in turn, ensuring
that all dependencies are in place for the next stage of the build.

Another aspect of this lack of dependency tracking is that images that extend other images need to explicitly use the
the correct version. In order to keep this portable, Dockerfiles that depend on an existing image have had a `VERSION`
build argument added. This is appended to the end of the name of the parent Docker image. The default value is `latest`,
which is equivalent to leaving it blank in Docker tooling.

To create a set of images with matching tags, use an environment variable `VERSION`. This will be appended as a tag to
the images and passed as the `VERSION` build arguement to dockerfiles that depend on other `starling-*` images.
