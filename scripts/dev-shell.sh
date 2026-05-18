#!/usr/bin/env bash
# opens a shell in the development container
set -e
cd "$(dirname "$0")/.."
docker compose -f docker/docker-compose.yml run --rm dev bash