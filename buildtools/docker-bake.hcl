variable "TAG" {
    default = "latest"
    }

group "default" {
    targets = ["starling-ui"]
}

target "starling-ui" {
    context = "system/ui"
    tags = ["uobflightlabstarling/starling-ui:${TAG}"]
    platforms = ["linux/amd64"]
    }

target "starling-mavros" {
    context = "system/mavros"
    tags = ["uobflightlabstarling/starling-mavros:${TAG}"]
    platforms = ["linux/amd64", "linux/arm64"]
    }
