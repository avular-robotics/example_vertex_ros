#!/bin/bash
#

if [ -z "${ENTRYPOINT_QUIET_LOGS:-}" ]; then
    exec 3>&1
else
    exec 3>/dev/null
fi

ENV_FILES=()
ENV_FILES+=("${HOME}/.profile" "/opt/ros/${ROS_DISTRO}/setup.bash")

for env in "${ENV_FILES[@]}"
do
    if [ -f "${env}" ]; then
        echo >&3 "$0: Sourcing ${env}"
        source "${env}"
    fi
done

rosdep install --from-paths /home/user/ws/src --ignore-src -y
chown -R $(whoami) /home/user/ws/

exec "$@"
