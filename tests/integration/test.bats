#!/usr/bin/env bats

for testdir in $(ls $BATS_TEST_DIRNAME/tests); do
	@test "${testdir}" {
		docker-compose -f $BATS_TEST_DIRNAME/tests/$testdir/test.docker-compose.yml up --exit-code-from test
	}
done
