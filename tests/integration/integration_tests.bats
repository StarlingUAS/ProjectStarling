#!/usr/bin/env bats

for testdir in $(find $BATS_TEST_DIRNAME -mindepth 1 -maxdepth 1 -type d); do
	@test "Integration test in $(basename ${testdir})" {
		docker-compose -f $testdir/test.docker-compose.yml up --exit-code-from test
	}
done
