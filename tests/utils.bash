#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

function get_test_tag {
    TEST_FILE="$(basename $BATS_TEST_FILENAME .bats)"
    TEST_SUITE="$(realpath --relative-to="$SCRIPT_DIR" $BATS_TEST_DIRNAME)"
    TEST_TAG="$TEST_SUITE:$TEST_FILE"
    echo "$TEST_TAG"
}