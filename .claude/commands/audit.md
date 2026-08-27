Explore $ARGUMENTS (or the custom controller module, control
allocation, and actuator interface as a whole if no argument is
given). Do not make any code changes.

Report:
1. Actual current interfaces at the boundary in question — uORB
   topics, message types, function signatures, whatever's concretely
   there.
2. Anything that contradicts, or isn't yet covered by, the relevant
   spec (specs/system.md, specs/controller.md, specs/allocation.md).
3. Which contract-table row(s) in specs/system.md this exploration
   speaks to.

Append the findings to specs/findings.md as a new dated entry.

Do not edit specs/system.md, specs/controller.md, or
specs/allocation.md directly. Findings go in findings.md only —
whether and how to fold a finding into the actual contract is a
separate, deliberate decision, not something to do automatically as
part of exploring.
