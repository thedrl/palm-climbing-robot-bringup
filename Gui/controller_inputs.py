"""Controller bindings independent of pygame, Tk, and motor commands.

Legacy button numbers remain integers on disk.  Axis and directional-pad
bindings carry their rest/direction or hat direction explicitly.
"""

from collections.abc import Mapping
import math


ACTIONS = (
    "actuator_select_0", "actuator_select_1", "actuator_select_2",
    "actuator_select_3", "actuator_pulse_fwd", "actuator_pulse_rev",
    "wheel_fwd", "wheel_rev", "speed_increase", "speed_decrease", "stop",
    "actuator_pair_02", "actuator_pair_13",
)

LABELS = {
    "actuator_select_0": "Select actuator 0",
    "actuator_select_1": "Select actuator 1",
    "actuator_select_2": "Select actuator 2",
    "actuator_select_3": "Select actuator 3",
    "actuator_pulse_fwd": "Actuator forward / extend",
    "actuator_pulse_rev": "Actuator reverse / retract",
    "wheel_fwd": "Wheels forward",
    "wheel_rev": "Wheels reverse",
    "speed_increase": "Increase wheel speed",
    "speed_decrease": "Decrease wheel speed",
    "stop": "Stop",
    "actuator_pair_02": "Select actuators 0 + 2",
    "actuator_pair_13": "Select actuators 1 + 3",
}

AXIS_PRESS_DELTA = 0.55
AXIS_RELEASE_DELTA = 0.30


def _index(value):
    return value if type(value) is int and value >= 0 else None


def _number(value):
    if type(value) not in (int, float):
        return None
    try:
        return float(value) if math.isfinite(value) else None
    except (OverflowError, TypeError, ValueError):
        return None


def _axis_value(value):
    value = _number(value)
    return value if value is not None and -1.0 <= value <= 1.0 else None


def _hat(value):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return None
    if any(type(component) is not int or component not in (-1, 0, 1)
           for component in value):
        return None
    return tuple(value)


def _button_pressed(value):
    return value is True or (type(value) is int and value == 1)


def _sequence(value):
    return value if isinstance(value, (list, tuple)) else ()


def normalize_binding(binding):
    """Return an integer button or canonical axis/hat dict, else None."""
    if _index(binding) is not None:
        return binding
    if not isinstance(binding, Mapping):
        return None
    index = _index(binding.get("index"))
    if index is None:
        return None
    kind = binding.get("kind")
    if kind == "button":
        return index
    if kind == "hat":
        direction = _hat((binding.get("x"), binding.get("y")))
        if direction is None or abs(direction[0]) + abs(direction[1]) != 1:
            return None
        return {"kind": "hat", "index": index, "x": direction[0], "y": direction[1]}
    if kind == "axis":
        rest = _axis_value(binding.get("rest"))
        direction = binding.get("direction")
        if rest is None or type(direction) is not int or direction not in (-1, 1):
            return None
        return {"kind": "axis", "index": index, "rest": rest, "direction": direction}
    return None


def get_binding(mapping, action):
    """Read both existing button fields and richer input bindings."""
    if not isinstance(mapping, Mapping) or action not in ACTIONS:
        return None
    if action.startswith("actuator_select_"):
        bindings = _sequence(mapping.get("actuator_select_buttons"))
        index = int(action.rsplit("_", 1)[1])
        value = bindings[index] if index < len(bindings) else None
    else:
        value = mapping.get(action + "_button")
    return normalize_binding(value)


def set_binding(mapping, action, binding):
    """Mutate a mapping; invalid bindings clear the selected action."""
    if action not in ACTIONS:
        raise ValueError("Unknown controller action: " + str(action))
    binding = normalize_binding(binding)
    if action.startswith("actuator_select_"):
        previous = _sequence(mapping.get("actuator_select_buttons"))
        bindings = [normalize_binding(previous[index]) if index < len(previous) else None
                    for index in range(4)]
        bindings[int(action.rsplit("_", 1)[1])] = binding
        mapping["actuator_select_buttons"] = bindings
    else:
        mapping[action + "_button"] = binding


def format_binding(binding):
    binding = normalize_binding(binding)
    if binding is None:
        return "Not mapped"
    if type(binding) is int:
        return "Button " + str(binding)
    if binding["kind"] == "hat":
        direction = {(0, 1): "Up", (0, -1): "Down", (-1, 0): "Left", (1, 0): "Right"}
        return f"D-pad {binding['index']} {direction[(binding['x'], binding['y'])]}"
    sign = "+" if binding["direction"] > 0 else "−"
    return f"Axis {binding['index']} {sign} (rest {binding['rest']:.2f})"


def evaluate(mapping, axes, buttons, hats, previous=None):
    """Evaluate an input snapshot, with hysteresis only for axis bindings."""
    axes, buttons, hats = map(_sequence, (axes, buttons, hats))
    previous = previous if isinstance(previous, Mapping) else {}
    result = {}
    for action in ACTIONS:
        binding = get_binding(mapping, action)
        active = False
        if type(binding) is int:
            active = binding < len(buttons) and _button_pressed(buttons[binding])
        elif binding is not None:
            index = binding["index"]
            if binding["kind"] == "hat" and index < len(hats):
                active = _hat(hats[index]) == (binding["x"], binding["y"])
            elif binding["kind"] == "axis" and index < len(axes):
                value = _axis_value(axes[index])
                if value is not None:
                    delta = (value - binding["rest"]) * binding["direction"]
                    active = (delta > AXIS_RELEASE_DELTA if previous.get(action) is True
                              else delta >= AXIS_PRESS_DELTA)
        result[action] = bool(active)
    return result


def _physical_key(binding):
    if type(binding) is int:
        return ("button", binding)
    if binding["kind"] == "hat":
        return ("hat", binding["index"], binding["x"], binding["y"])
    return ("axis", binding["index"], binding["direction"])


def _alias_groups(mapping):
    groups = []
    for group in _sequence(mapping.get("input_alias_groups")):
        valid = {}
        for value in _sequence(group):
            binding = normalize_binding(value)
            if binding is not None:
                valid.setdefault(_physical_key(binding), binding)
        if len(valid) > 1:
            groups.append(valid)
    return groups


def register_aliases(mapping, aliases):
    """Remember all inputs emitted by one learned press/release gesture.

    Merge overlapping groups, so aliases remain equivalent after later learning.
    Axis rest does not distinguish physical inputs; index and direction do.
    """
    groups = _alias_groups(mapping)
    new = {}
    for value in _sequence(aliases):
        binding = normalize_binding(value)
        if binding is not None:
            new.setdefault(_physical_key(binding), binding)
    if len(new) > 1:
        groups.append(new)
    merged = []
    for group in groups:
        remaining = []
        for previous in merged:
            if group.keys() & previous.keys():
                group = {**previous, **group}
            else:
                remaining.append(previous)
        merged = remaining + [group]
    mapping["input_alias_groups"] = [list(group.values()) for group in merged]


def conflicts(mapping):
    """Describe duplicate inputs, including learned button/axis aliases."""
    mapping = mapping if isinstance(mapping, Mapping) else {}
    parent = {}

    def find(key):
        parent.setdefault(key, key)
        while parent[key] != key:
            parent[key] = parent[parent[key]]
            key = parent[key]
        return key

    for group in _alias_groups(mapping):
        keys = list(group)
        for key in keys[1:]:
            parent[find(key)] = find(keys[0])
    physical = {}
    for action in ACTIONS:
        binding = get_binding(mapping, action)
        if binding is not None:
            physical.setdefault(find(_physical_key(binding)), []).append((action, binding))
    return [f"{', '.join(LABELS[action] for action, _ in entries)} share an input "
            f"({' / '.join(dict.fromkeys(format_binding(binding) for _, binding in entries))})."
            for entries in physical.values() if len(entries) > 1]


def available_errors(mapping, axes, buttons, hats):
    """Describe valid mappings unavailable on the connected controller."""
    counts = {"axis": len(_sequence(axes)), "button": len(_sequence(buttons)),
              "hat": len(_sequence(hats))}
    errors = []
    for action in ACTIONS:
        binding = get_binding(mapping, action)
        if binding is None:
            continue
        kind, index = ("button", binding) if type(binding) is int else (binding["kind"], binding["index"])
        if index >= counts[kind]:
            errors.append(f"{LABELS[action]}: {format_binding(binding)} is unavailable on this controller.")
    return errors


class BindingLearner:
    """Capture one complete press and release, including input aliases.

    Initially held digital buttons and hats must be released first. Axis rest
    is recorded on creation, or refreshed when that release completes. The
    caller must tell users to release triggers before clicking Learn, then
    fully press and release one control. ``sample`` returns only after release.
    ``aliases`` then contains every button/hat and the largest moving axis
    from that gesture. Analog axes take precedence as the learned binding.
    """

    def __init__(self, axes, buttons, hats):
        self._rest = tuple(_axis_value(value) for value in _sequence(axes))
        self._buttons = tuple(_button_pressed(value) for value in _sequence(buttons))
        self._hats = tuple(_hat(value) for value in _sequence(hats))
        self._waiting_release = any(self._buttons) or any(hat != (0, 0) for hat in self._hats)
        self._captured = False
        self._gesture_buttons = {}
        self._gesture_hats = {}
        self._axis_peak = None
        self.aliases = []

    def sample(self, axes, buttons, hats):
        if self._captured:
            return None
        axes = tuple(_axis_value(value) for value in _sequence(axes))
        raw_buttons = _sequence(buttons)
        buttons = tuple(_button_pressed(value) for value in raw_buttons)
        hats = tuple(_hat(value) for value in _sequence(hats))
        if self._waiting_release:
            if not any(buttons) and all(hat == (0, 0) for hat in hats):
                self._rest = axes
                self._waiting_release = False
            self._buttons, self._hats = buttons, hats
            return None

        for index, (before, now) in enumerate(zip(self._buttons, buttons)):
            if now and not before:
                self._gesture_buttons[index] = index
        for index, (before, now) in enumerate(zip(self._hats, hats)):
            if now is not None and now != before and abs(now[0]) + abs(now[1]) == 1:
                binding = {"kind": "hat", "index": index, "x": now[0], "y": now[1]}
                self._gesture_hats[_physical_key(binding)] = binding
        for index, (rest, now) in enumerate(zip(self._rest, axes)):
            if rest is not None and now is not None:
                delta = now - rest
                if (abs(delta) >= AXIS_PRESS_DELTA and
                        (self._axis_peak is None or abs(delta) > self._axis_peak[0])):
                    self._axis_peak = (abs(delta), index, rest, 1 if delta > 0 else -1)
        self._buttons, self._hats = buttons, hats
        if not (self._gesture_buttons or self._gesture_hats or self._axis_peak):
            return None

        # Missing or invalid input cannot masquerade as a release.
        for index in self._gesture_buttons:
            if (index >= len(raw_buttons) or
                    not (raw_buttons[index] is False or
                         (type(raw_buttons[index]) is int and raw_buttons[index] == 0))):
                return None
        for binding in self._gesture_hats.values():
            index = binding["index"]
            if index >= len(hats) or hats[index] != (0, 0):
                return None
        axis_binding = None
        if self._axis_peak is not None:
            _, index, rest, direction = self._axis_peak
            if (index >= len(axes) or axes[index] is None or
                    abs(axes[index] - rest) > AXIS_RELEASE_DELTA):
                return None
            axis_binding = {"kind": "axis", "index": index, "rest": rest, "direction": direction}
        self.aliases = ([axis_binding] if axis_binding is not None else [])
        self.aliases += list(self._gesture_buttons.values()) + list(self._gesture_hats.values())
        self._captured = True
        return self.aliases[0]
