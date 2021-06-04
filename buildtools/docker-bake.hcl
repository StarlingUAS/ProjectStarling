variable "BAKE_VERSION" {
    default = "latest"
}

variable "BAKE_REGISTRY" {
    default = ""
}

variable "BAKE_RELEASENAME" {
    default = ""
}

variable "BAKE_CACHE_REGISTRY" {
    default = "${BAKE_REGISTRY}"
}

variable "BAKE_CACHENAME" {
    default = ""
}

variable "BAKE_CACHENAME_FROM" {
    default = "${BAKE_CACHENAME}"
}

variable "BAKE_CACHENAME_TO" {
    default = "${BAKE_CACHENAME}"
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
        "${BAKE_REGISTRY}uobflightlabstarling/starling-ui:${BAKE_VERSION}",
        notequal("",BAKE_RELEASENAME) ? "${BAKE_REGISTRY}uobflightlabstarling/starling-ui:${BAKE_RELEASENAME}": "",
        ]
    platforms = ["linux/amd64"]
    cache-to = [ notequal("",BAKE_CACHENAME_TO) ? "${BAKE_CACHE_REGISTRY}uobflightlabstarling/starling-ui:${BAKE_CACHENAME_TO}" : "" ]
    cache-from = [ notequal("",BAKE_CACHENAME_FROM) ? "${BAKE_CACHE_REGISTRY}uobflightlabstarling/starling-ui:${BAKE_CACHENAME_FROM}" : "" ]
}

target "starling-controller-base" {
    context = "system/controller-base"
    tags = [
        "${BAKE_REGISTRY}uobflightlabstarling/starling-controller-base:${BAKE_VERSION}",
        notequal("",BAKE_RELEASENAME) ? "${BAKE_REGISTRY}uobflightlabstarling/starling-controller-base:${BAKE_RELEASENAME}": "",
        ]
    platforms = ["linux/amd64", "linux/arm64"]
    cache-to = [ notequal("",BAKE_CACHENAME_TO) ? "${BAKE_CACHE_REGISTRY}uobflightlabstarling/starling-controller-base:${BAKE_CACHENAME_TO}" : "" ]
    cache-from = [ notequal("",BAKE_CACHENAME_FROM) ? "${BAKE_CACHE_REGISTRY}uobflightlabstarling/starling-controller-base:${BAKE_CACHENAME_FROM}" : "" ]
}

target "starling-mavros" {
    context = "system/mavros"
    tags = [
        "${BAKE_REGISTRY}uobflightlabstarling/starling-mavros:${BAKE_VERSION}",
        notequal("",BAKE_RELEASENAME) ? "${BAKE_REGISTRY}uobflightlabstarling/starling-mavros:${BAKE_RELEASENAME}": "",
        ]
    platforms = ["linux/amd64", "linux/arm64"]
    cache-to = [ notequal("",BAKE_CACHENAME_TO) ? "${BAKE_CACHE_REGISTRY}uobflightlabstarling/starling-mavros:${BAKE_CACHENAME_TO}" : "" ]
    cache-from = [ notequal("",BAKE_CACHENAME_FROM) ? "${BAKE_CACHE_REGISTRY}uobflightlabstarling/starling-mavros:${BAKE_CACHENAME_FROM}" : "" ]
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
        "${BAKE_REGISTRY}uobflightlabstarling/starling-sim-base-core:${BAKE_VERSION}",
        notequal("",BAKE_RELEASENAME) ? "${BAKE_REGISTRY}uobflightlabstarling/starling-sim-base-core:${BAKE_RELEASENAME}": "",
        ]
    platforms = ["linux/amd64"]
    cache-to = [ notequal("",BAKE_CACHENAME_TO) ? "${BAKE_CACHE_REGISTRY}uobflightlabstarling/starling-sim-base-core:${BAKE_CACHENAME_TO}" : "" ]
    cache-from = [ notequal("",BAKE_CACHENAME_FROM) ? "${BAKE_CACHE_REGISTRY}uobflightlabstarling/starling-sim-base-core:${BAKE_CACHENAME_FROM}" : "" ]
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
        "VERSION": "${BAKE_VERSION}",
        "REGISTRY": "${BAKE_REGISTRY}"
        }
    tags = [
        "${BAKE_REGISTRY}uobflightlabstarling/starling-sim-base-px4:${BAKE_VERSION}",
        notequal("",BAKE_RELEASENAME) ? "${BAKE_REGISTRY}uobflightlabstarling/starling-sim-base-px4:${BAKE_RELEASENAME}": "",
        ]
    platforms = ["linux/amd64"]
    cache-to = [ notequal("",BAKE_CACHENAME_TO) ? "${BAKE_CACHE_REGISTRY}uobflightlabstarling/starling-sim-base-px4:${BAKE_CACHENAME_TO}" : "" ]
    cache-from = [ notequal("",BAKE_CACHENAME_FROM) ? "${BAKE_CACHE_REGISTRY}uobflightlabstarling/starling-sim-base-px4:${BAKE_CACHENAME_FROM}" : "" ]
}

target "starling-sim-px4-sitl" {
    context = "simulator/base/px4"
    dockerfile = "sitl.Dockerfile"
    tags = [
        "${BAKE_REGISTRY}uobflightlabstarling/starling-sim-px4-sitl:${BAKE_VERSION}",
        notequal("",BAKE_RELEASENAME) ? "${BAKE_REGISTRY}uobflightlabstarling/starling-sim-px4-sitl:${BAKE_RELEASENAME}": "",
        ]
    platforms = ["linux/amd64"]
    cache-to = [ notequal("",BAKE_CACHENAME_TO) ? "${BAKE_CACHE_REGISTRY}uobflightlabstarling/starling-sim-px4-sitl:${BAKE_CACHENAME_TO}" : "" ]
    cache-from = [ notequal("",BAKE_CACHENAME_FROM) ? "${BAKE_CACHE_REGISTRY}uobflightlabstarling/starling-sim-px4-sitl:${BAKE_CACHENAME_FROM}" : "" ]
}

/*
 * Controller targets
 */
// This target depends on starling-controller-base
target "example_python_controller" {
    context = "controllers/example_controller_python"
    args = { 
        "VERSION": "${BAKE_VERSION}",
        "REGISTRY": "${BAKE_REGISTRY}"
        }
    tags = [
        "${BAKE_REGISTRY}uobflightlabstarling/example_controller_python:${BAKE_VERSION}",
        notequal("",BAKE_RELEASENAME) ? "${BAKE_REGISTRY}uobflightlabstarling/example_controller_python:${BAKE_RELEASENAME}": "",
        ]
    platforms = ["linux/amd64", "linux/arm64"]
    cache-to = [ notequal("",BAKE_CACHENAME_TO) ? "${BAKE_CACHE_REGISTRY}uobflightlabstarling/example_controller_python:${BAKE_CACHENAME_TO}" : "" ]
    cache-from = [ notequal("",BAKE_CACHENAME_FROM) ? "${BAKE_CACHE_REGISTRY}uobflightlabstarling/example_controller_python:${BAKE_CACHENAME_FROM}" : "" ]
}

/*
 * Vehicle targets
 */
// This target depends on starling-sim-base-px4
target "starling-sim-iris" {
    context = "simulator/vehicles/iris"
    args = { 
        "VERSION": "${BAKE_VERSION}",
        "REGISTRY": "${BAKE_REGISTRY}"
        }
    tags = [
        "${BAKE_REGISTRY}uobflightlabstarling/starling-sim-iris:${BAKE_VERSION}",
        notequal("",BAKE_RELEASENAME) ? "${BAKE_REGISTRY}uobflightlabstarling/starling-sim-iris:${BAKE_RELEASENAME}": "",
        ]
    platforms = ["linux/amd64"]
    cache-to = [ notequal("",BAKE_CACHENAME_TO) ? "${BAKE_CACHE_REGISTRY}uobflightlabstarling/starling-sim-iris:${BAKE_CACHENAME_TO}" : "" ]
    cache-from = [ notequal("",BAKE_CACHENAME_FROM) ? "${BAKE_CACHE_REGISTRY}uobflightlabstarling/starling-sim-iris:${BAKE_CACHENAME_FROM}" : "" ]
}
