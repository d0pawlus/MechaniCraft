# Backlog po fazie 1 (odłożone uwagi z review)

Uwagi drobne, które review uznały za niewymagające poprawki przed scaleniem. Do rozważenia w kolejnych fazach.

- Task 1: minor (deferred): no validation of negative/NaN durations — needs validating constructor when SAM import feeds the crate (Task 6 should reject)
- Task 1: minor (deferred): eval doc says "last segment" velocity but code uses last positive-duration segment
- Task 1: minor (deferred): interior grid point dropped if rounding reaches t_end (extreme steps only); negative durations unvalidated
- Task 2: minor (deferred): missing tests for duplicate/UnknownAngle/UnknownInputElement/zero-length slider/accepted slider+hinge inputs/validation-order precedence
- Task 2: minor (deferred): InputOnNonDrivable message hardcodes "is a beam"; NaN coords pass zero-length check; duplicate inputs on one element accepted; glob re-export in lib.rs; O(n²) lookups
- Task 5: minor (deferred): indent counted over all Unicode whitespace bytes; SamError::Unsupported unused until Task 6
- Task 3: minor (deferred): L→0 non-finite grad/conv undocumented (solver must detect); wrap() lacks fast path (1e-15 noise near 0); role tests thin (slider free/driven, fixed hinge, input-over-fixed); conv FD tolerance absolute
- Task 4: minor (deferred): acceleration target ë≠0 never exercised (CLINEAR only in phase 1 — known gap); coarse-step branch test likely non-discriminating for this Grashof crank-rocker; solver indexes pub Mechanism fields without validation (panics on hand-built malformed Mechanism); unreachable trailing Err(Singular)
- Task 4: minor (deferred): stencil comment overstates truncation error
- Task 6: minor (deferred): loose error tests (key/line not asserted; KIND≠1, NSTEPS=0, non-finite MOTION, missing ELEMGROUPS untested)
- Task 7: minor (deferred): check sort is byte-wise case-sensitive OsStr order; no check test with spaces in names
- Task 6b: minor (deferred): no multi-segment ordering test; CRLF-in-literal replace strings in nonzero_axis test; AXIS i64 vs KIND u32
- Final fix wave: re-review clean (8/8 addressed). Out-of-scope minors: hand-built InputLaw step count unbounded (only import caps); validate() doesn't check coordinate kind per id
- Final review #9: flaga ELONGATION w .SAM nie jest sprawdzana względem wejść
- Final review #10: udostępnić składanie jakobianu (Problem) dla kinetostatyki w fazie 2
