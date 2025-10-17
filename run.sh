#!/bin/bash
set -e

# Detect current workspace
WORKSPACE_DIR="$(pwd)/ros2_ws"

# Check if the folder exists
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "❌ Workspace not found: $WORKSPACE_DIR"
    echo "Please make sure you're running this script from your project root."
    exit 1
fi

# Container parameters
CONTAINER_NAME="cont"
IMAGE_NAME="wetexplorer"
USER_NAME="ros"

# Check if the container is already running
if docker ps --filter "name=${CONTAINER_NAME}" --format "{{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "🔁 Container '${CONTAINER_NAME}' is already running — opening bash shell..."
    docker exec -it "${CONTAINER_NAME}" bash
else
    echo "🚀 Starting new container '${CONTAINER_NAME}' using workspace: ${WORKSPACE_DIR}"
    docker run --rm --name "${CONTAINER_NAME}" -it \
        --gpus all \
        --user "${USER_NAME}" \
        --network=host --ipc=host \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY \
        -v "${WORKSPACE_DIR}":/ros2_ws \
        -v /dev:/dev \
        --device-cgroup-rule='c *:* rmw' \
        "${IMAGE_NAME}" /bin/bash
fi

