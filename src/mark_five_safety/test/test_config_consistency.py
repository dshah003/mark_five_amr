"""The watchdog's caps must never exceed Nav2's velocity_smoother limits (S4)."""

import os

import pytest

yaml = pytest.importorskip('yaml')

HERE = os.path.dirname(os.path.abspath(__file__))
SAFETY_YAML = os.path.join(HERE, '..', 'config', 'safety_watchdog.yaml')
NAV2_YAML = os.path.join(HERE, '..', '..', 'mark_five_bot', 'config', 'nav2_params.yaml')


def _load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def test_safety_caps_at_or_below_nav2_limits():
    if not os.path.exists(NAV2_YAML):
        pytest.skip('nav2_params.yaml not found next to this package')
    safety = _load(SAFETY_YAML)['safety_watchdog']['ros__parameters']
    smoother = _load(NAV2_YAML)['velocity_smoother']['ros__parameters']
    max_v, min_v = smoother['max_velocity'], smoother['min_velocity']
    assert safety['max_linear_velocity'] <= min(max_v[0], -min_v[0])
    assert safety['max_angular_velocity'] <= min(max_v[2], -min_v[2])


def test_safety_defaults_match_plan():
    safety = _load(SAFETY_YAML)['safety_watchdog']['ros__parameters']
    assert safety['input_topic'] == '/cmd_vel_safe'
    assert safety['output_topic'] == '/cmd_vel_gated'
    assert safety['heartbeat_timeout'] <= 1.0
    assert safety['max_linear_velocity'] <= 0.20
    assert safety['max_angular_velocity'] <= 1.0
