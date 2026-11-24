# Observations from the present supervised test session

The user authorized necessary testing and confirmed independent support,
clear wheel/actuator travel, hands clear, presence at the cutoff, and both
GUIs closed. They continued responding during the session.

- Arm 0 brief repeated low-current wheel test: user did not see rotation,
  but heard the motor turn on. Encoder feedback showed a small negative
  movement for a positive command; host guard stopped the test.
- Arm 3 actuator, manual retract at power 40 for approximately 0.75 s:
  user said, "okay that worked. for actuator 3. it just moved back a little ways."
  They subsequently said, "i think actuator 3 is good."
  Fresh diagnostics confirmed zero PWM/current after stop. FWD/OUT limit
  changed from pressed to open. Encoder remained -1, so this confirms manual
  movement and stopping, not reliable encoder-based positioning.
- Stop further actuator 3 motion tests; focus on wheels 0 and 3.

Evidence: /home/frc-orin/V18-FarmBot21SEP2026-ActuatorRecovery/evidence/act3-manual-retract-20260922T081120Z/result.json
