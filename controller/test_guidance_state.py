from controller.guidance_state_machine import GuidanceStateMachine, GuidanceState


def test_initial_state():
    gsm = GuidanceStateMachine()
    assert gsm.get_state() == GuidanceState.CRUISE
