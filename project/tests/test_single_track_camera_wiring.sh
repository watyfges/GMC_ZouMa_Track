#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

TRACK_CPP="${PROJECT_DIR}/code/hardware/trackuvc.cpp"
TRACK_H="${PROJECT_DIR}/code/hardware/trackuvc.h"
AI_CPP="${PROJECT_DIR}/code/hardware/aiuvc.cpp"
MAIN_CPP="${PROJECT_DIR}/user/main.cpp"

require_contains() {
    local file="$1"
    local pattern="$2"
    if ! grep -Eq "$pattern" "$file"; then
        echo "missing pattern '$pattern' in $file" >&2
        exit 1
    fi
}

require_not_contains() {
    local file="$1"
    local pattern="$2"
    if grep -Eq "$pattern" "$file"; then
        echo "unexpected pattern '$pattern' in $file" >&2
        exit 1
    fi
}

require_contains "$TRACK_CPP" 'track_cam\.init\("/dev/video_track", TRACK_W, TRACK_H, TRACK_CAMERA_FPS\)'
require_contains "$TRACK_H" '#define TRACK_CAMERA_FPS 60'
require_contains "$TRACK_H" 'bool track_uvc_get_bgr_frame\(cv::Mat &out_frame\);'
require_contains "$AI_CPP" 'track_uvc_get_bgr_frame\(out_frame\)'
require_not_contains "$AI_CPP" 'ai_cam\.init\("/dev/video_ai"'
require_not_contains "$AI_CPP" 'static zf_device_uvc ai_cam'
require_not_contains "$MAIN_CPP" 'ai_uvc_init\(\);'
