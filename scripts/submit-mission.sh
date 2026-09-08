#!/usr/bin/env bash
set -euo pipefail

scenario="${1:-all}"
mission_id="${2:-${scenario}-$(date +%s)}"
ros2 action send_goal /drone/survey drone_observer_msgs/action/SurveyMission \
  "{mission_id: '${mission_id}', scenario: '${scenario}'}" --feedback
