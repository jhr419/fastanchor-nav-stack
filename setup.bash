#!/usr/bin/env bash

# 保留旧入口；新的统一环境入口位于 user/setup_env.sh。
SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SETUP_DIR/user/setup_env.sh"
