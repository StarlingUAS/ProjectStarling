variable "VERSION" {
    default = "latest"
}

variable "REGISTRY" {
    default = ""
}

variable "NAMEDTAG" {
    default = ""
}

/*
 * Groups for target ordering
 */
group "stage1" {
    targets = ["system", "simulator"]
}

group "stage2" {
    targets = ["simulator-px4", "example_python_controller"]    
}

group "stage3" {
    targets = ["starling-sim-iris"]
}

/*
 * System targets
 */
group "system" {
    targets = ["starling-ui", "starling-controller-base", "starling-mavros"]
}

target "starling-ui" {
    context = "system/ui"
    tags = [
        "${REGISTRY}uobflightlabstarling/starling-ui:${VERSION}",
        notequal("",NAMEDTAG) ? "${REGISTRY}uobflightlabstarling/starling-ui:${NAMEDTAG}": "",
        ]
    platforms = ["linux/amd64"]
}

target "starling-controller-base" {
    context = "system/controller-base"
    tags = [
        "${REGISTRY}uobflightlabstarling/starling-controller-base:${VERSION}",
        notequal("",NAMEDTAG) ? "${REGISTRY}uobflightlabstarling/starling-controller-base:${NAMEDTAG}": "",
        ]
    platforms = ["linux/amd64", "linux/arm64"]
}

target "starling-mavros" {
    context = "system/mavros"
    tags = [
        "${REGISTRY}uobflightlabstarling/starling-mavros:${VERSION}",
        notequal("",NAMEDTAG) ? "${REGISTRY}uobflightlabstarling/starling-mavros:${NAMEDTAG}": "",
        ]
    platforms = ["linux/amd64", "linux/arm64"]
}


/* 
 * Simulator targets
 */
// Build this target before simulator-px4
group "simulator" {
    targets = [
        "starling-sim-base-core"
        ]
}

target "starling-sim-base-core" {
    context = "simulator/base/core"
    tags = [
        "${REGISTRY}uobflightlabstarling/starling-sim-base-core:${VERSION}",
        notequal("",NAMEDTAG) ? "${REGISTRY}uobflightlabstarling/starling-sim-base-core:${NAMEDTAG}": "",
        ]
    platforms = ["linux/amd64"]
}

group "simulator-px4" {
    targets = [
        "starling-sim-base-px4",
        "starling-sim-px4-sitl"
        ]
}

// This target depends on starling-sim-base-core
target "starling-sim-base-px4" {
    context = "simulator/base/px4"
    args = { 
        "VERSION": "${VERSION}",
        "REGISTRY": "${REGISTRY}"
        }
    tags = [
        "${REGISTRY}uobflightlabstarling/starling-sim-base-px4:${VERSION}",
        notequal("",NAMEDTAG) ? "${REGISTRY}uobflightlabstarling/starling-sim-base-px4:${NAMEDTAG}": "",
        ]
    platforms = ["linux/amd64"]
}

target "starling-sim-px4-sitl" {
    context = "simulator/base/px4"
    dockerfile = "sitl.Dockerfile"
    tags = [
        "${REGISTRY}uobflightlabstarling/starling-sim-px4-sitl:${VERSION}",
        notequal("",NAMEDTAG) ? "${REGISTRY}uobflightlabstarling/starling-sim-px4-sitl:${NAMEDTAG}": "",
        ]
    platforms = ["linux/amd64"]
}

/*
 * Controller targets
 */
// This target depends on starling-controller-base
target "example_python_controller" {
    context = "controllers/example_controller_python"
    args = { 
        "VERSION": "${VERSION}",
        "REGISTRY": "${REGISTRY}"
        }
    tags = [
        "${REGISTRY}uobflightlabstarling/example_controller_python:${VERSION}",
        notequal("",NAMEDTAG) ? "${REGISTRY}uobflightlabstarling/example_controller_python:${NAMEDTAG}": "",
        ]
    platforms = ["linux/amd64", "linux/arm64"]
}

/*
 * Vehicle targets
 */
// This target depends on starling-sim-base-px4
target "starling-sim-iris" {
    context = "simulator/vehicles/iris"
    args = { 
        "VERSION": "${VERSION}",
        "REGISTRY": "${REGISTRY}"
        }
    tags = [
        "${REGISTRY}uobflightlabstarling/starling-sim-iris:${VERSION}",
        notequal("",NAMEDTAG) ? "${REGISTRY}uobflightlabstarling/starling-sim-iris:${NAMEDTAG}": "",
        ]
    platforms = ["linux/amd64"]
}
