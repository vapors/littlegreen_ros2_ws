from __future__ import annotations

from pathlib import Path

from lgh_st3215_tools.calibration_common import (
    REFERENCE_MODEL_ZERO,
    REFERENCE_POLICY_DEFAULT,
    calibration_updates_from_proposal,
    classify_correction,
    derived_raw_limits,
    expected_step_for_reference,
    load_servo_map,
    patch_joint_map_mirror_text,
    patch_servo_map_calibration_text,
    proposed_center_step_for_reference,
    raw_limits_valid,
)


def _servo_map_path() -> Path:
    return Path(__file__).parents[2] / "lgh_st3215_driver" / "config" / "servo_map.yaml"


def _joint_map_path() -> Path:
    return (
        Path(__file__).parents[2]
        / "littlegreen_biped_pkg"
        / "src"
        / "configs"
        / "joint_map.yaml"
    )


def test_model_zero_capture_sets_center_to_measured_step() -> None:
    _, joints = load_servo_map(_servo_map_path())
    joint = joints[3]
    assert proposed_center_step_for_reference(joint, 1999.4, REFERENCE_MODEL_ZERO) == 1999


def test_policy_default_expected_step_matches_current_left_knee() -> None:
    _, joints = load_servo_map(_servo_map_path())
    joint = joints[3]
    assert expected_step_for_reference(joint, REFERENCE_POLICY_DEFAULT) == 1647


def test_small_center_change_preserves_radian_limits_without_range_conflict() -> None:
    _, joints = load_servo_map(_servo_map_path())
    joint = joints[5]  # left ankle roll; historical tool flagged a two-step change
    new_center = joint.center_step - 2
    lo, hi = derived_raw_limits(joint, new_center)
    assert (lo, hi) == (joint.min_step - 2, joint.max_step - 2)
    assert raw_limits_valid(lo, hi)
    assert classify_correction(-2, 25, 100, True) == "FINE_SOFTWARE_CORRECTION"


def test_partial_proposal_patches_servo_and_joint_maps() -> None:
    _, joints = load_servo_map(_servo_map_path())
    joint = joints[0]
    new_center = joint.center_step + 1
    lo, hi = derived_raw_limits(joint, new_center)
    proposal = {
        "joints": [
            {
                "name": joint.name,
                "proposed_center_step": new_center,
                "derived_min_step": lo,
                "derived_max_step": hi,
            }
        ]
    }
    updates = calibration_updates_from_proposal(proposal)
    servo_text = patch_servo_map_calibration_text(_servo_map_path().read_text(), updates)
    joint_text = patch_joint_map_mirror_text(_joint_map_path().read_text(), updates)
    assert f"center_step: {new_center}" in servo_text
    assert f"min_step: {lo}" in servo_text
    assert f"max_step: {hi}" in servo_text
    assert f"servo_center_step: {new_center}" in joint_text
    assert f"servo_min_step: {lo}" in joint_text
    assert f"servo_max_step: {hi}" in joint_text
