variable "TAG" {
    default = "latest"
    }

group "default" {
    targets = ["starling-ui","starling-controller-base"]
}

target "starling-ui" {
    context = "system/ui"
    tags = ["uobflightlabstarling/starling-ui:${TAG}"]
    platforms = ["linux/amd64"]
}

target "starling-controller-base" {
    context = "system/controller-base"
    tags = ["uobflightlabstarling/starling-controller-base:${TAG}"]
    platforms = ["linux/amd64", "linux/arm64"]
}

target "starling-mavros" {
    context = "system/mavros"
    tags = ["uobflightlabstarling/starling-mavros:${TAG}"]
    platforms = ["linux/amd64", "linux/arm64"]
}
