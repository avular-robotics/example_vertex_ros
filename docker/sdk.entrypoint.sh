#!/bin/bash
#

if [ -z "${ENTRYPOINT_QUIET_LOGS:-}" ]; then
    exec 3>&1
else
    exec 3>/dev/null
fi

ENV_FILES=()

for env in "${ENV_FILES[@]}"
do
    if [ -f "${env}" ]; then
        echo >&3 "$0: Sourcing ${env}"
        source "${env}"
    fi
done

exec "$@"
