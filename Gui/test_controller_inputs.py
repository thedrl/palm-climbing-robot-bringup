import math
import unittest

from controller_inputs import (
    ACTIONS, BindingLearner, available_errors, conflicts, evaluate, format_binding,
    get_binding, normalize_binding, register_aliases, set_binding,
)


def axis(index=0, rest=-1.0, direction=1):
    return {"kind": "axis", "index": index, "rest": rest, "direction": direction}


def hat(index=0, x=0, y=1):
    return {"kind": "hat", "index": index, "x": x, "y": y}


class BindingTests(unittest.TestCase):
    def test_legacy_buttons_and_canonical_button_dictionary(self):
        self.assertEqual(normalize_binding(0), 0)
        self.assertEqual(normalize_binding({"kind": "button", "index": 4}), 4)
        self.assertEqual(get_binding({"actuator_select_buttons": [0, 1, 2, 3]}, "actuator_select_2"), 2)
        self.assertEqual(get_binding({"wheel_fwd_button": 4}, "wheel_fwd"), 4)

    def test_malformed_bindings_are_not_inputs(self):
        malformed = [None, True, False, -1, 1.5, "1", [], {}, {"index": 0},
                     {"kind": "button", "index": True}, {"kind": "button", "index": -1},
                     {"kind": "button", "index": "1"}, {"kind": "other", "index": 0},
                     axis(rest=float("nan")), axis(rest=float("inf")), axis(rest=2),
                     axis(rest=True), axis(direction=0), axis(direction=True), axis(index=-1),
                     hat(x=1, y=1), hat(y=0), hat(y=2), hat(x=True),
                     {"kind": "axis", "index": 0, "direction": 1}]
        for value in malformed:
            with self.subTest(value=value):
                self.assertIsNone(normalize_binding(value))

    def test_setting_preserves_unrelated_configuration(self):
        mapping = {"actuator_select_buttons": [0, 1, 2, 3], "wheel_axis": 1}
        set_binding(mapping, "actuator_pair_02", hat(x=-1, y=0))
        set_binding(mapping, "actuator_select_1", {"kind": "button", "index": 7})
        self.assertEqual(mapping["actuator_select_buttons"], [0, 7, 2, 3])
        self.assertEqual(get_binding(mapping, "actuator_pair_02"), hat(x=-1, y=0))
        self.assertEqual(mapping["wheel_axis"], 1)
        set_binding(mapping, "actuator_select_1", "invalid")
        self.assertIsNone(get_binding(mapping, "actuator_select_1"))
        self.assertIsNone(get_binding(mapping, "unrecognized"))
        with self.assertRaises(ValueError):
            set_binding(mapping, "unrecognized", 0)

    def test_set_select_repairs_missing_list_without_aliasing_it(self):
        original = [0, 1]
        mapping = {"actuator_select_buttons": original}
        set_binding(mapping, "actuator_select_3", 8)
        self.assertEqual(mapping["actuator_select_buttons"], [0, 1, None, 8])
        self.assertEqual(original, [0, 1])

    def test_formats_all_input_kinds(self):
        self.assertEqual(format_binding(0), "Button 0")
        self.assertIn("Up", format_binding(hat()))
        self.assertIn("-1.00", format_binding(axis()))
        self.assertEqual(format_binding(None), "Not mapped")

    def test_button_and_cardinal_hat_evaluation(self):
        mapping = {"stop_button": 0, "actuator_pair_02_button": hat(x=-1, y=0)}
        result = evaluate(mapping, [], [True], [(-1, 0)])
        self.assertEqual(set(result), set(ACTIONS))
        self.assertTrue(result["stop"])
        self.assertTrue(result["actuator_pair_02"])
        self.assertFalse(result["actuator_pair_13"])
        self.assertFalse(evaluate(mapping, [], [False], [(-1, 1)])["actuator_pair_02"])

    def test_axis_both_trigger_rests_and_centered_sticks(self):
        for rest, direction in [(-1.0, 1), (1.0, -1), (0.0, 1), (0.0, -1)]:
            with self.subTest(rest=rest, direction=direction):
                mapping = {"wheel_fwd_button": axis(rest=rest, direction=direction)}
                self.assertFalse(evaluate(mapping, [rest], [], [])["wheel_fwd"])
                self.assertFalse(evaluate(mapping, [rest + direction * .54], [], [])["wheel_fwd"])
                self.assertTrue(evaluate(mapping, [rest + direction * .56], [], [])["wheel_fwd"])

    def test_axis_hysteresis_releases_and_cannot_latch_on_invalid_input(self):
        mapping = {"wheel_fwd_button": axis(rest=0)}
        result = evaluate(mapping, [.55], [], [])
        self.assertTrue(result["wheel_fwd"])
        result = evaluate(mapping, [.40], [], [], result)
        self.assertTrue(result["wheel_fwd"])
        result = evaluate(mapping, [.30], [], [], result)
        self.assertFalse(result["wheel_fwd"])
        self.assertFalse(evaluate(mapping, [.40], [], [], result)["wheel_fwd"])
        for invalid in [math.nan, math.inf, -math.inf, "0.8", None, True, 5]:
            with self.subTest(invalid=invalid):
                self.assertFalse(evaluate(mapping, [invalid], [], [], {"wheel_fwd": True})["wheel_fwd"])

    def test_unavailable_or_malformed_input_never_activates(self):
        mapping = {"stop_button": 3, "wheel_fwd_button": axis(index=1),
                   "actuator_pair_02_button": hat(index=1)}
        self.assertFalse(any(evaluate(mapping, [], [], []).values()))
        self.assertFalse(any(evaluate(mapping, None, None, None).values()))
        self.assertEqual(len(available_errors(mapping, [0], [0], [(0, 0)])), 3)
        self.assertEqual(available_errors(mapping, [0, 0], [0] * 4, [(0, 0)] * 2), [])

    def test_duplicate_button_conflict_is_actionable(self):
        mapping = {"actuator_pulse_fwd_button": 5, "stop_button": {"kind": "button", "index": 5}}
        errors = conflicts(mapping)
        self.assertEqual(len(errors), 1)
        self.assertIn("Stop", errors[0])
        self.assertIn("Actuator forward", errors[0])
        self.assertIn("Button 5", errors[0])

    def test_same_direction_axis_conflicts_regardless_rest(self):
        mapping = {"wheel_fwd_button": axis(rest=-1), "stop_button": axis(rest=0)}
        self.assertEqual(len(conflicts(mapping)), 1)
        mapping["stop_button"] = axis(rest=0, direction=-1)
        self.assertEqual(conflicts(mapping), [])

    def test_hats_conflict_only_for_same_hat_and_direction(self):
        mapping = {"wheel_fwd_button": hat(), "stop_button": hat()}
        self.assertEqual(len(conflicts(mapping)), 1)
        mapping["stop_button"] = hat(y=-1)
        self.assertEqual(conflicts(mapping), [])
        mapping["stop_button"] = hat(index=1)
        self.assertEqual(conflicts(mapping), [])


class AliasTests(unittest.TestCase):
    def test_button_axis_aliases_conflict_despite_different_rest(self):
        for rest, direction in [(-1, 1), (1, -1), (0, 1), (0, -1)]:
            with self.subTest(rest=rest, direction=direction):
                mapping = {"speed_increase_button": 7,
                           "actuator_pulse_rev_button": axis(index=4, rest=rest, direction=direction)}
                self.assertEqual(conflicts(mapping), [])
                register_aliases(mapping, [7, axis(index=4, rest=0, direction=direction)])
                errors = conflicts(mapping)
                self.assertEqual(len(errors), 1)
                self.assertIn("Button 7", errors[0])
                self.assertIn("Axis 4", errors[0])

    def test_alias_groups_merge_transitively_and_survive_serialization(self):
        import json
        mapping = {"stop_button": 7, "actuator_pair_02_button": hat()}
        register_aliases(mapping, [7, axis(index=4)])
        register_aliases(mapping, [axis(index=4), hat()])
        self.assertEqual(len(mapping["input_alias_groups"]), 1)
        restored = json.loads(json.dumps(mapping))
        self.assertEqual(len(conflicts(restored)), 1)

    def test_unmerged_loaded_alias_groups_are_transitive(self):
        mapping = {"stop_button": 7, "actuator_pair_02_button": hat(),
                   "input_alias_groups": [[7, axis(index=4)], [axis(index=4), hat()]]}
        self.assertEqual(len(conflicts(mapping)), 1)

    def test_invalid_and_duplicate_aliases_do_not_create_groups(self):
        mapping = {}
        register_aliases(mapping, [None, True, -1, {}, 7, {"kind": "button", "index": 7}])
        self.assertEqual(mapping["input_alias_groups"], [])
        self.assertEqual(conflicts({"input_alias_groups": [None, {}, [True, {}]]}), [])

    def test_opposite_axis_direction_not_an_alias_without_observation(self):
        mapping = {"stop_button": 7, "wheel_rev_button": axis(index=4, direction=-1)}
        register_aliases(mapping, [7, axis(index=4)])
        self.assertEqual(conflicts(mapping), [])


class BindingLearnerTests(unittest.TestCase):
    def test_button_zero_returns_only_after_release_and_is_one_shot(self):
        learner = BindingLearner([0], [0], [(0, 0)])
        self.assertIsNone(learner.sample([0], [0], [(0, 0)]))
        self.assertIsNone(learner.sample([0], [1], [(0, 0)]))
        self.assertIsNone(learner.sample([0], [1], [(0, 0)]))
        self.assertEqual(learner.sample([0], [0], [(0, 0)]), 0)
        self.assertEqual(learner.aliases, [0])
        self.assertIsNone(learner.sample([1], [0], [(0, 1)]))

    def test_trigger_collects_late_digital_alias_and_prefers_axis(self):
        learner = BindingLearner([0, 0, 0, 0, -1], [0] * 8, [])
        self.assertIsNone(learner.sample([0, 0, 0, 0, -.4], [0] * 8, []))
        self.assertIsNone(learner.sample([0, 0, 0, 0, 1], [0] * 7 + [1], []))
        self.assertIsNone(learner.sample([0, 0, 0, 0, -.5], [0] * 8, []))
        binding = learner.sample([0, 0, 0, 0, -1], [0] * 8, [])
        self.assertEqual(binding, axis(index=4))
        self.assertEqual(learner.aliases, [axis(index=4), 7])

    def test_button_first_axis_later_still_prefers_axis(self):
        learner = BindingLearner([-1], [0], [])
        self.assertIsNone(learner.sample([-.8], [1], []))
        self.assertIsNone(learner.sample([1], [1], []))
        self.assertEqual(learner.sample([-1], [0], []), axis())
        self.assertEqual(learner.aliases, [axis(), 0])

    def test_hat_with_button_alias_returns_button(self):
        learner = BindingLearner([0], [0], [(0, 0)])
        self.assertIsNone(learner.sample([.2], [1], [(0, -1)]))
        self.assertIsNone(learner.sample([0], [0], [(0, -1)]))
        self.assertEqual(learner.sample([0], [0], [(0, 0)]), 0)
        self.assertEqual(learner.aliases, [0, hat(y=-1)])

    def test_hat_diagonal_rejected_and_cardinal_learned_on_release(self):
        learner = BindingLearner([], [], [(0, 0)])
        self.assertIsNone(learner.sample([], [], [(1, 1)]))
        self.assertIsNone(learner.sample([], [], [(1, 0)]))
        self.assertEqual(learner.sample([], [], [(0, 0)]), hat(x=1, y=0))

    def test_axis_learning_records_rest_and_direction_on_release(self):
        for rest, direction in [(-1.0, 1), (1.0, -1), (0.0, 1), (0.0, -1)]:
            with self.subTest(rest=rest, direction=direction):
                learner = BindingLearner([rest], [], [])
                self.assertIsNone(learner.sample([rest + direction * .2], [], []))
                self.assertIsNone(learner.sample([rest + direction * .7], [], []))
                self.assertEqual(learner.sample([rest], [], []),
                                 axis(rest=rest, direction=direction))

    def test_learning_keeps_only_largest_axis_and_ignores_noise(self):
        learner = BindingLearner([0, -1, 1], [], [])
        self.assertIsNone(learner.sample([.1, -.95, .9], [], []))
        self.assertIsNone(learner.sample([.6, .8, .99], [], []))
        self.assertEqual(learner.sample([0, -1, 1], [], []), axis(index=1))
        self.assertEqual(learner.aliases, [axis(index=1)])

    def test_nonfinite_axis_cannot_be_learned(self):
        learner = BindingLearner([0, math.nan], [], [])
        self.assertIsNone(learner.sample([math.inf, 1], [], []))

    def test_active_axis_invalid_or_missing_does_not_count_as_release(self):
        learner = BindingLearner([-1], [0], [])
        self.assertIsNone(learner.sample([1], [1], []))
        self.assertIsNone(learner.sample([math.nan], [0], []))
        self.assertIsNone(learner.sample([], [0], []))
        self.assertEqual(learner.sample([-1], [0], []), axis())

    def test_active_button_missing_or_invalid_does_not_count_as_release(self):
        learner = BindingLearner([], [0], [])
        self.assertIsNone(learner.sample([], [1], []))
        self.assertIsNone(learner.sample([], [None], []))
        self.assertIsNone(learner.sample([], [], []))
        self.assertEqual(learner.sample([], [0], []), 0)

    def test_initial_button_must_release_before_new_gesture(self):
        learner = BindingLearner([-1], [1], [(0, 0)])
        self.assertIsNone(learner.sample([1], [1], [(0, 1)]))
        self.assertIsNone(learner.sample([-1], [0], [(0, 0)]))
        self.assertIsNone(learner.sample([-1], [0], [(0, 0)]))
        self.assertIsNone(learner.sample([-1], [1], [(0, 0)]))
        self.assertEqual(learner.sample([-1], [0], [(0, 0)]), 0)

    def test_initial_hat_release_refreshes_axis_rest(self):
        learner = BindingLearner([0], [0], [(1, 0)])
        self.assertIsNone(learner.sample([-1], [1], [(1, 0)]))
        self.assertIsNone(learner.sample([-1], [0], [(0, 0)]))
        self.assertIsNone(learner.sample([0], [0], [(0, 0)]))
        self.assertEqual(learner.sample([-1], [0], [(0, 0)]), axis(rest=-1))


if __name__ == "__main__":
    unittest.main()
