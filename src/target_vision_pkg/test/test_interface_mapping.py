from target_vision_pkg.vision_target import (
    local_fine_data_from_dx_dy,
    select_flight_vision_stage,
)


def test_target_error_is_mapped_to_existing_flight_axes():
    assert local_fine_data_from_dx_dy(25, -40) == (40, -25)
    assert local_fine_data_from_dx_dy(-12, 7) == (-7, 12)


def test_vision_stage_follows_existing_descent_state_and_height():
    assert select_flight_vision_stage(False, None, 70.0) == 1
    assert select_flight_vision_stage(False, 20.0, 70.0) == 1
    assert select_flight_vision_stage(True, None, 70.0) == 2
    assert select_flight_vision_stage(True, 70.0, 70.0) == 2
    assert select_flight_vision_stage(True, 69.9, 70.0) == 3
