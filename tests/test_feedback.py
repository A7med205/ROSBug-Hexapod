import unittest

from robot_core import (
    BatchExecutionResult,
    BatchExecutionStatus,
    Command,
    ControllerMode,
    CoordinatorEventKind,
)
from robot_core.coordinator import HexapodCoordinator


class StructuredFeedbackTest(unittest.TestCase):
    def test_default_mode_is_auto_and_rejection_is_structured(self):
        coordinator = HexapodCoordinator()

        feedback = coordinator.request_with_feedback(Command.walk(1, steps=2))

        self.assertEqual(coordinator.mode, ControllerMode.AUTO)
        self.assertFalse(feedback.accepted)
        self.assertEqual(feedback.code, "command_rejected")
        self.assertEqual(
            coordinator.drain_events()[-1].kind,
            CoordinatorEventKind.COMMAND_REJECTED,
        )

    def test_auto_operation_has_identity_progress_and_completion_event(self):
        coordinator = HexapodCoordinator()
        coordinator.request_with_feedback(Command.skip_stand_up())
        coordinator.drain_events()

        feedback = coordinator.request_with_feedback(Command.walk(1, steps=2))
        self.assertTrue(feedback.accepted)
        self.assertIsNotNone(feedback.operation_id)
        self.assertEqual(coordinator.status().auto_requested_steps, 2)

        while coordinator.auto_job is not None:
            batch = coordinator.next_batch()
            self.assertIsNotNone(batch)
            coordinator.complete_batch(batch.goal_id)

        events = coordinator.drain_events()
        self.assertEqual(events[-1].kind, CoordinatorEventKind.OPERATION_COMPLETED)
        self.assertEqual(events[-1].operation_id, feedback.operation_id)
        self.assertIsNone(coordinator.status().operation_id)

    def test_batch_execution_result_exposes_success_without_boolean_ambiguity(self):
        completed = BatchExecutionResult(
            7,
            BatchExecutionStatus.COMPLETED,
            "done",
        )
        failed = BatchExecutionResult(
            8,
            BatchExecutionStatus.REJECTED,
            "rejected",
        )

        self.assertTrue(completed.succeeded)
        self.assertFalse(failed.succeeded)


if __name__ == "__main__":
    unittest.main()
