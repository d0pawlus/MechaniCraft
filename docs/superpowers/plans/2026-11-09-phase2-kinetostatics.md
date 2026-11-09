# Faza 2: kinetostatyka, plan implementacji

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Dla każdego stanu trajektorii kinematycznej policzyć siły: moment lub siłę napędową każdego wejścia, moc napędu, reakcje na współrzędnych zablokowanych i siły wewnętrzne członów (siła normalna, momenty węzłowe). Obciążenia: masy węzłowe, masy i momenty bezwładności członów, grawitacja i siły zewnętrzne w węzłach, wczytywane także z `.SAM`.

**Architecture:** Analiza quasi-statyczna bez całkowania. W stanie `(q, q̇, q̈)` rozwiązujemy `Jᵀσ = (f_app − f_in)_free`, gdzie `J` to kwadratowy jakobian modów sztywnych i napędzanych po współrzędnych swobodnych, czyli ten sam, którego używa solver kinematyki. Składanie jakobianu wydzielamy z `mc-core/src/solver.rs` do publicznego `ModeSystem` (backlog #10). Reakcje: `r = Dᵀσ + f_in − f_app` na współrzędnych zablokowanych, gdzie `D` to pełny jakobian (z kolumnami zablokowanymi). Nowy crate `mc-kineto` zawiera model obciążeń, wektory sił uogólnionych i solver. `mc-samfile` dostaje `import_project`, a `mcraft` opcję `--forces`.

**Tech Stack:** Rust 1.99 (edition 2024), `nalgebra` 0.33 (LU), `thiserror` 2, `clap` 4, `anyhow` 1 (tylko CLI), `approx` 0.5 (testy), `assert_cmd` 2 i `tempfile` 3 (testy CLI).

**Spec:** `docs/superpowers/specs/2026-11-08-mechanismcraft-design.md` (rozdział 2: obciążenia w `.SAM`, rozdział 4: faza 2, rozdział 7: decyzje projektowe fazy 2)

## Global Constraints

- Clean-room jak w fazie 1. Pliki przykładów SAM i projekty pochodne (np. przykład z dodaną masą) **nie trafiają do repozytorium**. Fragmenty `.SAM` w testach pisze się ręcznie, z wymyślonymi liczbami. Nie kopiujemy liczb z przykładów do testów ani fixture'ów (wolno sprawdzać liczności, znaki i to, czy coś istnieje).
- Testy na przykładach czytają `SAM_EXAMPLES_DIR` (domyślnie `C:\Users\Public\Documents\Sam\Examples`) i pomijają się z komunikatem, gdy katalogu nie ma.
- Biblioteki nie panikują na danych wejściowych (także na ręcznie złożonym `Mechanism`, `Loads` i `Trajectory`). Błędy to typy z `thiserror`. Nieobsługiwana treść `.SAM` → `SamError::Unsupported { what, line }`, nigdy ciche pominięcie.
- API kinematyki pozostaje zgodne wstecz: `analyze`, `State`, `Trajectory`, `SolveError`, `import_sam` i wszystkie istniejące testy przechodzą bez zmian. Kolejność błędów `analyze` (walidacja → `NoInputs` → `InputGridMismatch` → `NotSquare`) bez zmian.
- Jednostki SI. Kąty w radianach. Grawitacja `g = VALUE·(cos ANGLE, sin ANGLE)`.
- Konwencje znaków (rozdział 7 specyfikacji): `σ` modu długości to siła normalna (+ rozciąganie). Napęd: `driving = −σ` modu napędzanego, `power = driving · v(t)`. Reakcja to siła lub moment wywierany **przez podporę na mechanizm**.
- Kompilacja i testy bez ostrzeżeń (`RUSTFLAGS=-D warnings`, Linux i Windows). CI używa `--locked`, więc każdy commit zmieniający zależności zawiera zaktualizowany `Cargo.lock`.
- Commity z jawną listą ścieżek (`git commit -- <ścieżki>`), bo w worktree mogą pracować równolegle inni agenci. Przy `index.lock` odczekać i ponowić. Wiadomość kończy pusta linia i `Co-Authored-By: Claude <model> <noreply@anthropic.com>`.
- Tolerancje: testy statyczne 1e-9 (bezwzględnie), bilans mocy 1e-6 względnie (względem `max(1, max_k |P_k|)`), bilans sił 1e-9 względnie (względem `max(1, max |składnik|)`), wzór zamknięty 1e-8 względnie.

## Review Focus

1. **Konwencje znaków.** Moment napędowy, siła normalna i reakcje łatwo dostać z przeciwnym znakiem, a wynik wygląda wiarygodnie. Testy w Task 4: `static_horizontal_crank_holding_torque` (moment trzymający masę na poziomej korbie jest dodatni, reakcja pionowa dodatnia) i `static_actuator_pushes_mass` (siłownik pchający masę w górę ma dodatnią siłę napędową).
2. **Człon konwekcyjny w bezwładności obrotowej.** `α̈ = ∇α·q̈ + conv_α`. Pominięcie `conv_α` daje błędny moment przy niezerowym `d·ḋ` (człon o zmiennej długości albo łącznik). Testy: `rotational_inertia_includes_convective_term` (Task 3) i `slider_crank_power_balance` (Task 4).
3. **Reakcje z obciętego jakobianu.** Jakobian solvera kinematyki nie ma kolumn współrzędnych zablokowanych, a reakcje ich wymagają (`D` pełny). Test w Task 4: `global_force_balance_slider_crank` (suma reakcji + siły czynne = suma `m·a` w x i w y, a reakcja na kącie podstawy napędu = moment napędowy).
4. **Ciche pomijanie obciążeń przy imporcie.** Moment zewnętrzny, siła poza węzłem, argument elementowy, `J1`/`J2` albo suwak z `XL ≠ 0` muszą dać `Unsupported` z linią, a nie zniknąć. Testy w Task 5: `torque_block_is_unsupported`, `forceblock_outside_node_is_unsupported`, `force_arg_element_angle_is_unsupported`, `hinge_inertia_is_unsupported`.
5. **Siła zależna od stanu.** Siła z tablicy o argumencie `Y` węzła musi być liczona z bieżącego `q`, nie z położenia początkowego, z interpolacją liniową i stałą wartością poza zakresem. Testy: `force_table_follows_node_y` (Task 3) i `force_table_interpolates_and_clamps` (Task 2).

---

## Struktura plików

```
crates/mc-core/src/system.rs            ModeSystem, Equation, Assembly (wydzielone z solver.rs)
crates/mc-core/src/solver.rs            analyze() na ModeSystem
crates/mc-core/src/modes.rs             + line_angle()
crates/mc-kineto/Cargo.toml
crates/mc-kineto/src/lib.rs             re-eksporty
crates/mc-kineto/src/loads.rs           Loads, NodeMass, ElementInertia, Gravity, NodalForce, ForceArg, ForceTable, LoadError
crates/mc-kineto/src/forces.rs          applied_forces(), inertia_forces()
crates/mc-kineto/src/solve.rs           kinetostatics(), ForceState, ForceTrajectory, StressLabel, KinetoError
crates/mc-kineto/tests/analytic.rs      testy statyczne, bilans mocy, bilans sił, wzór zamknięty
crates/mc-samfile/src/import.rs         + import_project(), Project, odczyt obciążeń
crates/mc-samfile/tests/examples.rs     + testy importu obciążeń z przykładów
crates/mc-samfile/tests/kinetostatics.rs  kinetostatyka 10 przykładów, bilans mocy, hak wyroczni
fixtures/sam-exports/README.md          jak przygotować eksport z SAM
crates/mc-cli/src/main.rs               analyze --forces, check z kinetostatyką
```

Zależności między zadaniami: T1 i T2 niezależne. T3 ← T1, T2. T4 ← T3. T5 ← T2 (równolegle z T3 i T4). T6 ← T4, T5. T7 ← T4, T5 (równolegle z T6).

---

### Task 1 (T1): Wspólne składanie jakobianu w `mc-core`

**Files:**
- Create: `crates/mc-core/src/system.rs`
- Modify: `crates/mc-core/src/solver.rs`, `crates/mc-core/src/modes.rs`, `crates/mc-core/src/lib.rs`, `docs/backlog-phase1.md`

**Interfaces:**
- Consumes: `Mechanism`, `element_modes`, `mode_roles`, `SolveError`
- Produces (`system.rs`, re-eksport w `lib.rs`: `pub use system::{Assembly, Equation, ModeSystem};`, `pub use modes::line_angle;`):
  - `#[derive(Debug, Clone, Copy, PartialEq, Eq)] pub struct Equation { pub element: usize, pub mode: usize, pub input: Option<usize> }`: mod sztywny (`input: None`, cel 0) albo napędzany przez `mech.inputs[i]`.
  - `#[derive(Debug, Clone, PartialEq)] pub struct Assembly { pub value: DVector<f64>, pub jacobian: DMatrix<f64>, pub conv: DVector<f64> }`: `jacobian` ma wymiar `equations × free`.
  - `#[derive(Debug, Clone)] pub struct ModeSystem<'a>` (pola prywatne: `mech`, `equations`, `column: Vec<Option<usize>>`, `free: Vec<usize>`)
  - `impl<'a> ModeSystem<'a>`:
    - `pub fn new(mech: &'a Mechanism) -> Result<Self, SolveError>`: `mech.validate()` → `InvalidModel`, równania z `mode_roles` (pomija `Free`, kolejność ról), kolumny współrzędnych niezablokowanych rosnąco po `CoordId`, `NotSquare` gdy liczby się różnią. Nie wymaga wejść.
    - `pub fn mechanism(&self) -> &'a Mechanism`
    - `pub fn equations(&self) -> &[Equation]`
    - `pub fn free(&self) -> &[usize]`: indeksy `CoordId` współrzędnych swobodnych w kolejności kolumn.
    - `pub fn targets(&self, t: f64) -> [DVector<f64>; 3]`: `(u, v, a)` każdego równania (0 dla sztywnych).
    - `pub fn assemble(&self, q: &[f64], qd: &[f64]) -> Assembly`
    - `pub fn full_jacobian(&self, q: &[f64]) -> DMatrix<f64>`: `equations × mech.coords.len()`, z kolumnami zablokowanymi; `full_jacobian(q)` w kolumnach `free()` jest równy `assemble(q, 0).jacobian`.
  - `modes.rs`: `pub fn line_angle(mech: &Mechanism, el: &Element, q: &[f64], qd: &[f64]) -> Option<ModeEval>`: dla `Beam` i `Slider` bezwzględny kąt prostej `α = atan2(d)` (`value`), `grad` po czterech współrzędnych węzłów (`∂α/∂q_x = −d_y/L²`, `∂α/∂q_y = d_x/L²`, przeciwnie dla `p`), `conv = −2(d×ḋ)(d·ḋ)/L⁴`. `None` dla `Hinge` i `RotMotor`. Implementacja przez istniejący `Span` (bez duplikowania wzorów).
- `solver.rs`: `analyze` po walidacji, `NoInputs` i `InputGridMismatch` buduje `ModeSystem::new` i używa go w Newtonie oraz w prędkościach i przyspieszeniach. Prywatne `Problem`, `Equation`, `Assembly` znikają z `solver.rs`. Publiczne API `analyze`, `State`, `Trajectory`, `SolveError` bez zmian.

- [ ] **Step 1: Napisać testy, które na razie nie przechodzą** (`system.rs`, `#[cfg(test)]`, mechanizm korbowodu jak w `model.rs::builds_slider_crank`: 11 współrzędnych, 7 swobodnych):
  - `mode_system_counts`: `equations().len() == 7`, `free().len() == 7`, równanie napędu ma `input == Some(0)`, pozostałe `None`.
  - `full_jacobian_restricted_to_free_equals_assemble`: przy `q = value0` i `q` zaburzonym o 0.01 na każdej swobodnej współrzędnej: `full_jacobian(q).select_columns(free())` == `assemble(q, zeros).jacobian` (równość dokładna).
  - `full_jacobian_includes_fixed_columns`: przy `q = value0` wymiar `7 × 11`, a wpis w wierszu modu długości belki 1 i kolumnie `NodeX(1)` (zablokowanej) wynosi `-1.0` (1e-15), bo belka 1 leży wzdłuż osi x.
  - `mode_system_rejects_not_square`: korbowód bez blokady y węzła 3 → `Err(SolveError::NotSquare { equations: 7, unknowns: 8 })`.
  - `mode_system_rejects_invalid_model`: `CoordId(999)` w belce → `Err(SolveError::InvalidModel(ModelError::InvalidCoordId { element: 1 }))`.
  - w `modes.rs`: `line_angle_matches_bending_mode`: dla każdej belki i suwaka przy zaburzonym `q`, `q̇`: gradient `line_angle` po współrzędnych węzłów == minus gradient modu 1 po tych samych współrzędnych (1e-15), `conv == −conv` modu 1 (1e-15), `value == atan2(d)`. Dla `Hinge` i `RotMotor` wynik to `None`.
- [ ] **Step 2:** `cargo test -p mc-core`. Oczekiwane: FAIL (brak `ModeSystem`, `line_angle`).
- [ ] **Step 3:** Implementacja: przenieść `Equation`, `Assembly`, `targets`, `assemble` z `solver.rs` do `system.rs`, dodać `full_jacobian` (to samo składanie bez filtrowania kolumn), przepiąć `newton`, `solve`, `scatter`, `solve_step` na `ModeSystem`.
- [ ] **Step 4:** `cargo test -p mc-core`. Oczekiwane: PASS, w tym wszystkie testy fazy 1 (`tests/analytic.rs`, `solver.rs`) bez zmian.
- [ ] **Step 5:** W `docs/backlog-phase1.md` dopisać do pozycji „Final review #10” na końcu linii: ` — zrobione w fazie 2 (ModeSystem)`.
- [ ] **Step 6:** `git commit -m "refactor(core): expose ModeSystem assembly and line angle for kinetostatics" -- crates/mc-core docs/backlog-phase1.md`

### Task 2 (T2): Crate `mc-kineto` i model obciążeń

**Files:**
- Create: `crates/mc-kineto/Cargo.toml`, `crates/mc-kineto/src/lib.rs`, `crates/mc-kineto/src/loads.rs`
- Modify: `Cargo.lock` (nowy członek workspace)

**Interfaces:**
- `Cargo.toml`: `name = "mc-kineto"`, pola `edition/rust-version/license` z workspace. `[dependencies]`: `mc-core = { path = "../mc-core" }`, `nalgebra`, `thiserror` (workspace). `[dev-dependencies]`: `mc-motion = { path = "../mc-motion" }`, `approx` (workspace).
- `lib.rs`: doc-komentarz crate'a, `pub mod loads;` i jawne re-eksporty `pub use loads::{ElementInertia, ForceArg, ForceTable, Gravity, LoadError, Loads, NodalForce, NodeMass};` (bez `*`).
- `loads.rs`:
  - `#[derive(Debug, Clone, Copy, PartialEq)] pub struct NodeMass { pub node: u32, pub mass: f64 }`
  - `#[derive(Debug, Clone, Copy, PartialEq)] pub struct ElementInertia { pub element: u32, pub mass: f64, pub cog: f64, pub inertia: f64 }`: `cog` = ułamek długości od węzła 1 (`XL`), `inertia` = moment bezwładności względem środka masy.
  - `#[derive(Debug, Clone, Copy, PartialEq)] pub struct Gravity { pub value: f64, pub angle: f64 }` z `pub fn vector(&self) -> [f64; 2]` = `value·(cos angle, sin angle)`.
  - `#[derive(Debug, Clone, Copy, PartialEq, Eq)] pub enum ForceArg { Time, NodeX(u32), NodeY(u32) }`
  - `#[derive(Debug, Clone, PartialEq)] pub struct ForceTable { rows: Vec<[f64; 3]> }` (wiersz: argument, wartość, kąt bezwzględny w rad):
    - `pub fn new(rows: Vec<[f64; 3]>) -> Result<Self, LoadError>`: pusta → `EmptyForceTable`, wpis nieskończony albo argument nie rośnie ściśle → `BadForceTableRow { row }` (indeks od 0, pierwszy zły wiersz).
    - `pub fn rows(&self) -> &[[f64; 3]]`
    - `pub fn eval(&self, arg: f64) -> (f64, f64)`: `(wartość, kąt)` interpolowane liniowo między sąsiednimi wierszami; dla `arg` ≤ pierwszego lub ≥ ostatniego argumentu wartości skrajnego wiersza; jeden wiersz = stała.
  - `#[derive(Debug, Clone, PartialEq)] pub struct NodalForce { pub node: u32, pub arg: ForceArg, pub table: ForceTable }`: siła `wartość·(cos kąt, sin kąt)` w węźle.
  - `#[derive(Debug, Clone, Default, PartialEq)] pub struct Loads { pub node_masses: Vec<NodeMass>, pub element_inertias: Vec<ElementInertia>, pub gravity: Option<Gravity>, pub forces: Vec<NodalForce> }`
  - `impl Loads { pub fn validate(&self, mech: &Mechanism) -> Result<(), LoadError>; pub fn is_empty(&self) -> bool }`. Kolejność kontroli: masy węzłowe, bezwładności członów, grawitacja, siły.
  - `#[derive(Debug, Clone, PartialEq, thiserror::Error)] pub enum LoadError`:
    - `UnknownNode(u32)`: `"load refers to unknown node {0}"`
    - `UnknownElement(u32)`: `"inertia refers to unknown element {0}"`
    - `NotABeamOrSlider(u32)`: `"element {0} cannot carry mass: only beams and sliders can"`
    - `DuplicateElementInertia(u32)`: `"duplicate inertia for element {0}"`
    - `InvalidValue { what: String, value: f64 }`: `"{what} must be finite and non-negative, got {value}"` (masa węzła, masa i `J` członu)
    - `CogOutOfRange { element: u32, cog: f64 }`: `"centre of gravity of element {element} must lie in [0, 1], got {cog}"`
    - `SliderCog(u32)`: `"slider {0}: a centre of gravity away from node 1 is not supported"`
    - `NonFiniteGravity`: `"gravity must be finite"`
    - `EmptyForceTable`: `"force table is empty"`
    - `BadForceTableRow { row: usize }`: `"force table row {row} is not finite or its argument does not increase"`

- [ ] **Step 1: Napisać testy, które na razie nie przechodzą** (`loads.rs`, `#[cfg(test)]`, korbowód z budowniczego jak w fazie 1: węzły 1–3, belki 1 i 2, `RotMotor` 3):
  - `force_table_interpolates_and_clamps`: tablica `[[0.0, 10.0, 0.0], [1.0, 30.0, 1.0], [3.0, 70.0, 2.0]]` → `eval(0.5) == (20.0, 0.5)`, `eval(2.0) == (50.0, 1.5)`, `eval(-1.0) == (10.0, 0.0)`, `eval(5.0) == (70.0, 2.0)`, `eval(1.0) == (30.0, 1.0)` (1e-12).
  - `force_table_single_row_is_constant`: `[[2.0, 5.0, -0.5]]` → `eval(-10.0) == eval(10.0) == (5.0, -0.5)`.
  - `force_table_rejects_bad_rows`: `vec![]` → `EmptyForceTable`; `[[1.0, 0.0, 0.0], [1.0, 1.0, 0.0]]` → `BadForceTableRow { row: 1 }`; `[[0.0, f64::NAN, 0.0]]` → `BadForceTableRow { row: 0 }`.
  - `gravity_vector_points_down`: `Gravity { value: 9.81, angle: -FRAC_PI_2 }.vector()` ≈ `[0.0, -9.81]` (1e-12).
  - `validate_accepts_slider_crank_loads`: masa 25 w węźle 3, bezwładności belek 1 (`1.0, 0.5, 0.01`) i 2 (`2.0, 0.3, 0.05`), grawitacja, siła `NodeY(2)` w węźle 3 → `Ok(())`.
  - `validate_rejects_bad_loads` (po jednym przypadku): węzeł 99 → `UnknownNode(99)`; element 42 → `UnknownElement(42)`; bezwładność na `RotMotor` 3 → `NotABeamOrSlider(3)`; dwa wpisy dla belki 1 → `DuplicateElementInertia(1)`; masa węzła `-1.0` → `InvalidValue { .. }`; `cog: 1.5` → `CogOutOfRange { element: 1, cog: 1.5 }`; grawitacja `value: f64::INFINITY` → `NonFiniteGravity`; siła z argumentem `NodeX(77)` → `UnknownNode(77)`.
  - `validate_rejects_slider_cog`: mechanizm z suwakiem nr 5 i `ElementInertia { element: 5, mass: 1.0, cog: 0.5, inertia: 0.0 }` → `SliderCog(5)`; z `cog: 0.0` → `Ok(())`.
- [ ] **Step 2:** `cargo test -p mc-kineto`. Oczekiwane: FAIL.
- [ ] **Step 3:** Implementacja.
- [ ] **Step 4:** `cargo test -p mc-kineto` i `cargo build --workspace --all-targets --locked` (po `cargo build` aktualizującym `Cargo.lock`). Oczekiwane: PASS.
- [ ] **Step 5:** `git commit -m "feat(kineto): load model with masses, gravity and tabulated nodal forces" -- crates/mc-kineto Cargo.lock`

### Task 3 (T3): Siły uogólnione (czynne i bezwładności)

**Files:**
- Create: `crates/mc-kineto/src/forces.rs`
- Modify: `crates/mc-kineto/src/lib.rs` (`pub mod forces; pub use forces::{applied_forces, inertia_forces};`)

**Interfaces:**
- Consumes: `Loads`, `Mechanism::{node, elements}`, `ElementKind`, `mc_core::line_angle`
- Produces:
  - `pub fn applied_forces(mech: &Mechanism, loads: &Loads, t: f64, q: &[f64]) -> Vec<f64>`: grawitacja i siły zewnętrzne jako wektor uogólniony długości `mech.coords.len()`.
  - `pub fn inertia_forces(mech: &Mechanism, loads: &Loads, q: &[f64], qd: &[f64], qdd: &[f64]) -> Vec<f64>`: siły bezwładności `+∂T/∂q̇` pochodne (wektor do odjęcia), długość `mech.coords.len()`.
  - Obie zakładają `loads.validate(mech)` i długości wektorów równe `coords.len()`. Odwołania, których nie da się rozwiązać, są pomijane bez paniki (zgłasza je `validate`, a `kinetostatics` sprawdza długości). Doc-komentarze mówią to wprost.

Wzory (`p`, `n` = węzły 1 i 2 członu, `s = cog`, `g = gravity.vector()` albo `[0, 0]`):
```
masa węzłowa m:     f_in[x] += m·ẍ,  f_in[y] += m·ÿ;          f_app[x,y] += m·g
masa członu m, s:   r̈_c = (1−s)·p̈ + s·n̈  (składowo)
                    f_in[p] += m(1−s)·r̈_c,  f_in[n] += m·s·r̈_c
                    f_app[p] += m(1−s)·g,   f_app[n] += m·s·g
bezwładność J:      la = line_angle(mech, el, q, q̇);  α̈ = Σ_k la.grad_k·q̈_k + la.conv
                    f_in[k] += J·α̈·la.grad_k   dla każdego (k, grad_k)
siła w węźle:       a = t | q[x_węzła] | q[y_węzła] (wg ForceArg);  (F, φ) = table.eval(a)
                    f_app[x] += F·cos φ,  f_app[y] += F·sin φ
```

- [ ] **Step 1: Napisać testy, które na razie nie przechodzą** (`forces.rs`, `#[cfg(test)]`; mechanizm testowy: węzły 1 `(0,0)` i 2 `(1,0)` swobodne, kąty 1001, 1002, belka 1 `(1, 2, 1001, 1002)`; kolejność współrzędnych: x1, y1, x2, y2, 1001, 1002):
  - `node_mass_inertia_and_weight`: masa 3 w węźle 2, `q̈` z `ẍ2 = 2`, `ÿ2 = -1`, grawitacja `(10, −π/2)` → `f_in[x2] == 6.0`, `f_in[y2] == -3.0`, `f_app[y2] == -30.0`, pozostałe 0 (1e-12).
  - `beam_mass_lumps_by_cog`: `ElementInertia { element: 1, mass: 4.0, cog: 0.25, inertia: 0.0 }`, `ẍ1 = 1`, `ẍ2 = 3` → `f_in[x1] == 4.5`, `f_in[x2] == 1.5`; grawitacja `(10, −π/2)` → `f_app[y1] == -30.0`, `f_app[y2] == -10.0` (1e-12).
  - `rotational_inertia_uses_angular_acceleration`: `inertia: 0.5`, `mass: 0.0`, `q̇ = 0`, `ÿ2 = 2` → `f_in[y2] == 1.0`, `f_in[y1] == -1.0`, x-owe 0 (1e-12).
  - `rotational_inertia_includes_convective_term`: `inertia: 0.5`, `ẋ2 = 1`, `ẏ2 = 1`, `q̈ = 0` (wtedy `conv = −2`, `α̈ = −2`) → `f_in[y2] == -1.0`, `f_in[y1] == 1.0` (1e-12).
  - `force_table_follows_node_y`: siła w węźle 2, `ForceArg::NodeY(2)`, tablica `[[0.0, 0.0, 0.0], [1.0, 100.0, 0.0]]`; przy `q = value0` (`y2 = 0`) `f_app[x2] == 0.0`; przy `y2 = 0.25` `f_app[x2] == 25.0` (1e-12).
  - `time_force_uses_t`: `ForceArg::Time`, tablica `[[0.0, 0.0, FRAC_PI_2], [2.0, 8.0, FRAC_PI_2]]`, `t = 0.5` → `f_app[y2] == 2.0`, `f_app[x2]` ≈ 0 (1e-12).
- [ ] **Step 2:** `cargo test -p mc-kineto forces`. Oczekiwane: FAIL.
- [ ] **Step 3:** Implementacja według wzorów.
- [ ] **Step 4:** `cargo test -p mc-kineto`. Oczekiwane: PASS.
- [ ] **Step 5:** `git commit -m "feat(kineto): generalized applied and inertia forces" -- crates/mc-kineto/src`

### Task 4 (T4): Solver kinetostatyki

**Files:**
- Create: `crates/mc-kineto/src/solve.rs`, `crates/mc-kineto/tests/analytic.rs`
- Modify: `crates/mc-kineto/src/lib.rs` (`pub mod solve; pub use solve::{ForceState, ForceTrajectory, KinetoError, StressLabel, kinetostatics};`)

**Interfaces:**
- Consumes: `ModeSystem`, `Trajectory`, `applied_forces`, `inertia_forces`, `Loads::validate`
- Produces:
  - `#[derive(Debug, Clone, Copy, PartialEq, Eq)] pub struct StressLabel { pub element_nr: u32, pub mode: usize, pub input: Option<usize> }`: jedno na równanie `ModeSystem`, w tej samej kolejności.
  - `#[derive(Debug, Clone, PartialEq)] pub struct ForceState { pub t: f64, pub sigma: Vec<f64>, pub driving: Vec<f64>, pub power: Vec<f64>, pub reactions: Vec<f64> }`: `sigma` wg `StressLabel`; `driving` i `power` wg indeksu `mech.inputs`; `reactions` wg `CoordId` (dokładnie 0.0 na swobodnych).
  - `#[derive(Debug, Clone, PartialEq)] pub struct ForceTrajectory { pub stresses: Vec<StressLabel>, pub states: Vec<ForceState>, pub failure: Option<KinetoError> }`
  - `impl ForceTrajectory { pub fn stress(&self, step: usize, element_nr: u32, mode: usize) -> Option<f64> }`: `None`, gdy krok nie istnieje albo mod jest swobodny (brak równania).
  - `#[derive(Debug, Clone, PartialEq, thiserror::Error)] pub enum KinetoError { #[error("invalid mechanism: {0}")] Setup(SolveError), #[error("invalid loads: {0}")] Loads(LoadError), #[error("state {step} does not match the mechanism")] StateLength { step: usize }, #[error("singular equilibrium system at step {step} (t = {t})")] Singular { step: usize, t: f64 } }`
  - `pub fn kinetostatics(mech: &Mechanism, loads: &Loads, traj: &Trajectory) -> Result<ForceTrajectory, KinetoError>`

Algorytm: `ModeSystem::new(mech)` → `Setup`; `loads.validate(mech)` → `Loads`. Dla stanu `k` (stany `traj.states`, niezależnie od `traj.failure`): długości `q`, `qd`, `qdd` ≠ `coords.len()` → `Err(StateLength { step: k })` (sprawdzane dla wszystkich stanów przed liczeniem). `D = full_jacobian(q)`, `J = D` w kolumnach `free()`, `rhs = (f_app − f_in)` w `free()`. `σ = (Jᵀ).lu().solve(rhs)`; brak rozwiązania albo wartości nieskończone → `failure = Some(Singular { step, t })` i koniec (stany 0..k−1 zachowane). `reactions = Dᵀσ + f_in − f_app`, zerowane na swobodnych. `driving[i] = −σ[wiersz z input == Some(i)]`, `power[i] = driving[i] · inputs[i].law.eval(t).1`.

- [ ] **Step 1: Napisać testy, które na razie nie przechodzą** w `tests/analytic.rs`. Mechanizm statyczny („korba”): węzeł 1 `(0,0)` zablokowany xy, węzeł 2 swobodny, kąty 1001, 1002 swobodne, 1005 zablokowany, `beam(1, 1, 2, 1001, 1002)`, `rot_motor(2, 1005, 1001, false)`, `input(2, Linear { duration: 1.0, steps: 4, displacement: 0.0 })`, grawitacja `(9.81, −π/2)`. Wszystkie 5 stanów sprawdzane, tolerancja 1e-9:
  - `static_horizontal_crank_holding_torque`: węzeł 2 `(0.5, 0)`, masa 2 w węźle 2 → `driving[0] == 9.81`, `power[0] == 0.0`, `stress(k, 1, 0) == 0.0` (N), `stress(k, 1, 1) == 9.81`, `stress(k, 1, 2) == 0.0`, reakcje `NodeX(1) == 0.0`, `NodeY(1) == 19.62`, `Angle(1005) == 9.81`.
  - `static_vertical_crank_compression`: węzeł 2 `(0, 0.5)`, masa 2 w węźle 2 → `stress(k, 1, 0) == -19.62`, `driving[0] == 0.0`, `NodeY(1) == 19.62`.
  - `static_beam_mass_halves_torque`: węzeł 2 `(0.5, 0)`, zamiast masy węzłowej `ElementInertia { element: 1, mass: 2.0, cog: 0.5, inertia: 0.0 }` → `driving[0] == 4.905`, `NodeY(1) == 19.62`.
  - `static_actuator_pushes_mass`: węzeł 1 `(0,0)` zablokowany, węzeł 2 `(0, 0.5)` z zablokowanym x, kąty 1001, 1002 swobodne, `beam(1, 1, 2, 1001, 1002)` z wejściem `Linear { 1.0, 4, 0.0 }`, masa 2 w węźle 2, grawitacja `(9.81, −π/2)` → `driving[0] == 19.62`, `stress(k, 1, 0) == -19.62`.
  - Mechanizm dynamiczny („korbowód”): geometria, numeracja i napęd jak `slider_crank(0.5, law(1.0, 36), true)` z `crates/mc-core/tests/analytic.rs` (r = 0.1, l = 0.35, e = 0.05, belki 1 i 2, `rot_motor(3, 1005, 1001, false)`, 2π w 1 s); funkcję pomocniczą skopiować do testu. Obciążenia „pełne”: korba `ElementInertia { 1, 1.0, 0.5, 0.01 }`, łącznik `ElementInertia { 2, 2.0, 0.3, 0.05 }`, masa 25 w węźle 3, grawitacja `(9.81, −π/2)`, siła w węźle 3 `ForceArg::Time`, tablica `[[0.0, 10.0, 0.0], [1.0, 10.0, 0.0]]`.
  - `slider_crank_power_balance`: obciążenia „pełne”. W teście, niezależnie od `mc-kineto`, z `State` liczymy `dT/dt = Σ m ṙ·r̈ + Σ J α̇ α̈` (masy węzłowe, środki mas członów `r_c = (1−s)p + s·n`, `α̇ = (d×ḋ)/L²`, `α̈ = (d×d̈)/L² − 2(d×ḋ)(d·ḋ)/L⁴`), `dV/dt = −Σ m g·ṙ_c`, `P_ext = F·ṙ_3`. Dla każdego z 37 stanów `|power[0] − (dT/dt + dV/dt − P_ext)| ≤ 1e-6 · max(1, max_k |power_k|)`. Dodatkowo `failure.is_none()` i `states.len() == 37`.
  - `slider_mass_only_matches_closed_form`: tylko masa 25 w węźle 3, bez grawitacji → `driving[0] = 25·ẍ3·ẋ3/ω`, `ω = 2π`, tolerancja `1e-8 · max_k |driving_k|`.
  - `global_force_balance_slider_crank`: obciążenia „pełne”. Dla każdego stanu: `R_x(1) + Σ F_app,x = Σ m·a_x` i `R_y(1) + R_y(3) + Σ F_app,y = Σ m·a_y` (sumy po masach węzłowych i środkach mas członów, siły czynne: ciężary i siła zewnętrzna), tolerancja względna 1e-9; oraz `reactions[Angle(1005)] == driving[0]` (1e-9).
  - `kinetostatics_rejects_bad_loads`: masa w węźle 99 → `Err(KinetoError::Loads(LoadError::UnknownNode(99)))`.
  - `short_state_is_error_not_panic`: trajektoria korbowodu, w której `states[2].qdd` skrócono o 1 → `Err(KinetoError::StateLength { step: 2 })`.
  - `partial_trajectory_is_processed`: trajektoria korbowodu obcięta do 10 stanów z ustawionym `failure` → 10 stanów sił, `failure.is_none()`.
- [ ] **Step 2:** `cargo test -p mc-kineto --test analytic`. Oczekiwane: FAIL.
- [ ] **Step 3:** Implementacja `kinetostatics`.
- [ ] **Step 4:** `cargo test --workspace`. Oczekiwane: PASS. Jeśli test statyczny ma przeciwny znak, poprawić implementację, nie test (znaki są ustalone w specyfikacji, rozdział 7).
- [ ] **Step 5:** `git commit -m "feat(kineto): kinetostatic solver for driving forces, stresses and reactions" -- crates/mc-kineto/src crates/mc-kineto/tests`

### Task 5 (T5): Import obciążeń z `.SAM`

**Files:**
- Modify: `crates/mc-samfile/Cargo.toml` (`mc-kineto = { path = "../mc-kineto" }`), `crates/mc-samfile/src/import.rs`, `crates/mc-samfile/src/lib.rs` (`pub use import::{ImportError, Project, import_project, import_sam};`), `crates/mc-samfile/tests/examples.rs`, `Cargo.lock`, `docs/superpowers/specs/2026-11-08-mechanismcraft-design.md` (tylko gdy odkryjesz nowe znaczenie pola: dopisać w rozdziale 2)

**Interfaces:**
- Consumes: `parse_tree`, `SamNode`, `mc_kineto::{Loads, NodeMass, ElementInertia, Gravity, NodalForce, ForceArg, ForceTable, LoadError}`
- Produces:
  - `#[derive(Debug, Clone, PartialEq)] pub struct Project { pub mechanism: Mechanism, pub loads: Loads }`
  - `pub fn import_project(bytes: &[u8]) -> Result<Project, ImportError>`: mechanizm jak `import_sam`, potem obciążenia, potem `loads.validate(&mechanism)`.
  - Nowy wariant `ImportError::Loads(#[from] LoadError)` (`#[error(transparent)]`). Uwaga: `LoadError` nie jest `Eq`, więc z `ImportError` trzeba usunąć `derive(Eq)` (sprawdzić, czy nic w workspace go nie wymaga).
  - `import_sam` bez zmian (nie czyta obciążeń).

Mapowanie (rozdział 2 specyfikacji):
- `DOFLIST/NODE/MASS/MASSVALUE` → `NodeMass { node: NR, mass }`; wartość 0 pomijana; `MASS` pod `ANGLE` → `Unsupported { what: "MASS on ANGLE", line }`.
- `BEAM`, `SLIDER`: `M`, `XL`, `J` (brak klucza = 0) → `ElementInertia`, gdy `M ≠ 0` albo `J ≠ 0`. `SLIDER` z `M ≠ 0` i `XL ≠ 0` → `Unsupported { what: "SLIDER centre of gravity XL", line: linia XL }`.
- `HINGE`, `ROTMOTOR`: `J1` albo `J2` ≠ 0 → `Unsupported { what: "rotational inertia J1 on ROTMOTOR" (albo J2, HINGE), line }`.
- `GRAVITY`: brak bloku albo `VALUE == 0` → `None`; inaczej `Gravity { value: VALUE, angle: ANGLE }` (wartości skończone, inaczej `BadNumber`).
- `DOFLIST/NODE/FORCEBLOCK`: `DISABLED:1` → pomijany (świadomie, zgodnie z semantyką SAM); `EDNR` ≠ `NR` węzła → `Unsupported { what: "FORCEBLOCK EDNR {e} in NODE {nr}" }`; `ISTORQUE:1` → `Unsupported { what: "external torque" }`; `FORCEANYARG` wymagany (`Missing`); niepuste `FNAME` → `Unsupported { what: "force file" }`; wewnętrzne `ARGTYPE:2` z `ARGPROP` `X`/`Y` → `ForceArg::NodeX/NodeY(EDNR wewnętrzny)`; każda inna para → `Unsupported { what: "force argument ARGTYPE {a} ARGPROP {p}" }`; `TABLE` → wiersze po 3 liczby (inaczej `BadNumber { key: "TABLE", line: linia TABLE + 1 + indeks wiersza }`), potem `ForceTable::new` (błąd → `ImportError::Loads`).
- `FORCEBLOCK` w dowolnym innym miejscu drzewa (przeszukać całe drzewo rekurencyjnie) → `Unsupported { what: "FORCEBLOCK outside a node", line }`.

- [ ] **Step 1: Napisać testy, które na razie nie przechodzą.** W `import.rs` na ręcznym fragmencie z istniejącego `slider_crank_text`, z dopisanymi blokami (liczby wymyślone):
  - `imports_node_mass_and_gravity`: `MASS:` / `MASSVALUE:7.5` w węźle 3 i `GRAVITY:` z `VALUE:9.5`, `ANGLE:-1.5` → `node_masses == [NodeMass { node: 3, mass: 7.5 }]`, `gravity == Some(Gravity { value: 9.5, angle: -1.5 })`.
  - `imports_beam_inertia`: belka 1 z `M:3`, `XL:0.4`, `J:0.02` → `element_inertias == [ElementInertia { element: 1, mass: 3.0, cog: 0.4, inertia: 0.02 }]` (belka 2 bez kluczy nie występuje).
  - `imports_node_force_table`: `FORCEBLOCK` w węźle 3 (`EDNR:3`, `ISTORQUE:0`, `ARGTYPE:2`, `DISABLED:0`, `FORCEANYARG` z `FNAME:`, `ARGTYPE:2`, `EDNR:2`, `ARGPROP:X`, `TABLE` z wierszami `0 1 0` i `2 5 0.5`) → `forces[0].node == 3`, `arg == ForceArg::NodeX(2)`, `table.rows() == [[0.0, 1.0, 0.0], [2.0, 5.0, 0.5]]`.
  - `disabled_force_is_skipped`: to samo z `DISABLED:1` → `forces.is_empty()`.
  - `torque_block_is_unsupported`: `ISTORQUE:1` → `Unsupported` z `what` zawierającym `torque` i linią `FORCEBLOCK`.
  - `force_arg_element_angle_is_unsupported`: wewnętrzne `ARGTYPE:1`, `ARGPROP:A1` → `Unsupported` z `what == "force argument ARGTYPE 1 ARGPROP A1"`.
  - `forceblock_outside_node_is_unsupported`: `FORCEBLOCK` wewnątrz bloku `BEAM` → `Unsupported` z `what == "FORCEBLOCK outside a node"` i linią bloku.
  - `hinge_inertia_is_unsupported`: `ROTMOTOR` z `J1:0.5` → `Unsupported` z `what` zawierającym `J1`.
  - `slider_cog_is_unsupported`: `SLIDER` z `M:1`, `XL:0.5` → `Unsupported` z `what` zawierającym `XL`.
  - `negative_mass_is_a_load_error`: `MASSVALUE:-1` → `ImportError::Loads(LoadError::InvalidValue { .. })`.
  - `import_sam_ignores_loads`: fragment z `ISTORQUE:1` → `import_sam` zwraca `Ok` (kinematyka bez zmian).
  - W `tests/examples.rs` (gdy jest `SAM_EXAMPLES_DIR`): `phase1_examples_import_loads`: `import_project` zwraca `Ok` dla tych samych 10 przykładów co `phase1_examples_analyze`; `trfr_has_node_masses_and_gravity`: `TRFR.SAM` ma `node_masses.len() == 2` i `gravity.is_some()` z `value > 0`; `scs_force_depends_on_node_y`: `SCS.sam` ma dokładnie jedną siłę z argumentem `ForceArg::NodeY(_)`.
- [ ] **Step 2:** `cargo test -p mc-samfile`. Oczekiwane: FAIL.
- [ ] **Step 3:** Implementacja. Jeśli któryś z 10 przykładów zgłasza `Unsupported` w obciążeniach, sprawdź plik (tylko do odczytu) i według `superpowers:systematic-debugging` ustal znaczenie pola. Nowe ustalenia dopisz do rozdziału 2 specyfikacji własnymi słowami.
- [ ] **Step 4:** `cargo test -p mc-samfile` i `cargo build --workspace --all-targets --locked`. Oczekiwane: PASS.
- [ ] **Step 5:** `git commit -m "feat(samfile): import masses, gravity and nodal forces" -- crates/mc-samfile Cargo.lock docs/superpowers/specs/2026-11-08-mechanismcraft-design.md`

### Task 6 (T6): Kinetostatyka na przykładach SAM i hak wyroczni

**Files:**
- Create: `crates/mc-samfile/tests/kinetostatics.rs`, `fixtures/sam-exports/README.md`

**Interfaces:**
- Consumes: `import_project`, `mc_core::analyze`, `mc_kineto::{kinetostatics, applied_forces, inertia_forces}`
- Produces: testy integracyjne; katalog `fixtures/sam-exports/` z `README.md` (po polsku): eksport z SAM 8.5 (Results → Export, ASCII) dla nieskróconych przykładów z obciążeniami (`TRFR`, `BED`, `DYAD`, `SCS`) zapisany jako `fixtures/sam-exports/<NAZWA>.txt`, gdzie `<NAZWA>` to nazwa pliku przykładu bez rozszerzenia; wybrane wielkości: moment napędowy, moc, reakcje węzłów podporowych, siła normalna. Do repozytorium trafiają tylko eksporty wyników, nigdy pliki `.SAM`.

- [ ] **Step 1: Napisać testy** (funkcja `examples_dir()` jak w `tests/examples.rs`):
  - `phase1_examples_kinetostatics`: dla `SLCR.SAM, 6BAR.SAM, CRRK.SAM, ORTHO.SAM, DYAD.sam, SCS.sam, TRFR.SAM, BED.SAM, QKRT.SAM, DXFDEMO (detached).SAM`: `import_project` → `analyze` (bez `failure`) → `kinetostatics` daje `Ok`, `failure.is_none()`, tyle stanów co trajektoria, wszystkie liczby (`sigma`, `driving`, `power`, `reactions`) skończone. Błędy zbierane do listy i raportowane razem.
  - `examples_power_balance`: dla tych samych plików i każdego stanu: `Σ_i power_i` = `q̇ · (inertia_forces − applied_forces)` z tolerancją `1e-6 · max(1, max |składnik iloczynu|)`.
  - `examples_with_loads_have_nonzero_driving`: dla `TRFR.SAM` i `BED.SAM` maksimum `|driving[0]|` po stanach jest `> 0` (obciążenia faktycznie działają).
  - `sam_export_oracle_hook`: katalog `concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/sam-exports")`. Gdy nie ma w nim plików `*.txt` → `eprintln!("skipping: no SAM exports in …")` i powrót. Gdy są → `panic!` z komunikatem, że porównanie nie jest jeszcze zaimplementowane i wskazaniem `fixtures/sam-exports/README.md` (to sygnał dla następnej fazy, żeby dopasować parser do rzeczywistego formatu eksportu).
- [ ] **Step 2:** `cargo test -p mc-samfile --test kinetostatics`. Oczekiwane: PASS (przy obecnych przykładach) i komunikat o pominięciu haka. Jeśli któryś przykład nie przechodzi, zdiagnozować (`superpowers:systematic-debugging`) i poprawić przyczynę w `mc-kineto` albo w imporcie (osobny commit `fix(...)` z jawnymi ścieżkami).
- [ ] **Step 3:** `git commit -m "test(samfile): kinetostatics on SAM examples and export oracle hook" -- crates/mc-samfile/tests/kinetostatics.rs fixtures/sam-exports/README.md`

### Task 7 (T7): CLI `--forces` i `check` z kinetostatyką

**Files:**
- Modify: `crates/mc-cli/Cargo.toml` (`mc-kineto = { path = "../mc-kineto" }`), `crates/mc-cli/src/main.rs`, `crates/mc-cli/tests/cli.rs`, `Cargo.lock`, `README.md`

**Interfaces:**
- Consumes: `import_sam`, `import_project`, `analyze`, `kinetostatics`, `ForceTrajectory`
- Produces:
  - `mcraft analyze <PLIK> [-o CSV] [--nodes ...] [--forces]`. Bez `--forces` zachowanie i nagłówek bez zmian (`import_sam`). Z `--forces`: `import_project`, `analyze`, `kinetostatics`; po kolumnach węzłów dochodzą, w tej kolejności:
    1. dla każdego wejścia w kolejności `mech.inputs`: `DRV<nr elementu>`, `PWR<nr elementu>`;
    2. dla każdej współrzędnej zablokowanej: najpierw węzły rosnąco po NR (`RX<nr>` przed `RY<nr>`), potem kąty rosnąco po NR (`RA<nr>`);
    3. dla każdego elementu `Beam` rosnąco po NR: `N<nr>` (siła normalna = `σ` modu 0).
  - Wiersze: `min(stany kinematyki, stany sił)`, format `{:.12e}`. Kod wyjścia 2, gdy jest `failure` kinematyki albo sił (komunikat na stderr), 1 przy błędzie importu lub `KinetoError` zwróconym jako `Err`.
  - `mcraft check`: `classify` używa `import_project` → `analyze` → `kinetostatics`. `OK` tylko, gdy wszystko przechodzi (szczegół bez zmian: `"{n} states"`). `Unsupported` → `UNSUPPORTED`, każdy inny błąd (także `KinetoError` i `failure` sił) → `FAILED` z prefiksem `kinetostatics: ` dla błędów sił.
  - `README.md`: stan „faza 2, kinetostatyka” (obciążenia, wyniki), przykład `cargo run -p mc-cli -- analyze projekt.sam --forces`, crate `mc-kineto` w tabeli, link do planu fazy 2.

- [ ] **Step 1: Napisać testy, które na razie nie przechodzą** (`tests/cli.rs`; `SLIDER_CRANK` z dopisanym `MASS:` / `MASSVALUE:3` w węźle 3 i blokiem `GRAVITY:` z `VALUE:9.81`, `ANGLE:-1.5707963267948966` jako nowa stała `SLIDER_CRANK_LOADED`):
  - `analyze_forces_adds_columns`: `analyze <plik> --forces -o out.csv` → kod 0, 38 linii, nagłówek kończy się na `,DRV3,PWR3,RX1,RY1,RY3,RA1005,N1,N2`, każda komórka danych parsuje się jako skończony `f64`.
  - `analyze_without_forces_is_unchanged`: ten sam plik bez `--forces` → nagłówek nie zawiera `DRV`, kończy się na `,AY3`.
  - `check_runs_kinetostatics`: katalog z plikiem zawierającym `FORCEBLOCK` z `ISTORQUE:1` w węźle 3 → stdout zawiera `UNSUPPORTED` i `torque`, kod 0.
  - Istniejące testy `analyze_writes_csv` i `check_reports_unsupported_without_failing` przechodzą bez zmian.
- [ ] **Step 2:** `cargo test -p mc-cli`. Oczekiwane: FAIL.
- [ ] **Step 3:** Implementacja i aktualizacja `README.md`.
- [ ] **Step 4:** `cargo test --workspace --locked`. Oczekiwane: PASS. Potem ręcznie: `cargo run -p mc-cli -- check "C:\Users\Public\Documents\Sam\Examples"`. Oczekiwane: co najmniej 10 × `OK`, 0 × `FAILED`. `cargo run -p mc-cli -- analyze "C:\Users\Public\Documents\Sam\Examples\TRFR.SAM" --forces` → kod 0, kolumna `DRV` z niezerowymi wartościami.
- [ ] **Step 5:** `git commit -m "feat(cli): analyze --forces and kinetostatics in check" -- crates/mc-cli Cargo.lock README.md`

---

## Kryteria ukończenia fazy

```
cd X:\MechanismCraft\.worktrees\phase2
RUSTFLAGS="-D warnings" cargo build --workspace --all-targets --locked     # bez ostrzeżeń
RUSTFLAGS="-D warnings" cargo test --workspace --locked                    # wszystkie testy PASS
cargo run -p mc-cli -- check "C:\Users\Public\Documents\Sam\Examples"      # >= 10 OK, 0 FAILED, kod 0
cargo run -p mc-cli -- analyze "C:\Users\Public\Documents\Sam\Examples\TRFR.SAM" --forces   # kod 0, kolumny DRV/PWR/R*/N*
```
Kluczowe testy: `mc-kineto` `slider_crank_power_balance` (moment napędowy korbowodu zgodny z zasadą mocy wirtualnych), `static_horizontal_crank_holding_torque`, `global_force_balance_slider_crank`; `mc-samfile` `phase1_examples_kinetostatics`, `examples_power_balance`; `sam_export_oracle_hook` pomija się do czasu dostarczenia eksportów.

## Po fazie 2 (poza tym planem)
- **Wyrocznia SAM:** użytkownik eksportuje wyniki sił dla `TRFR`, `BED`, `DYAD`, `SCS` do `fixtures/sam-exports/`. Wtedy hak `sam_export_oracle_hook` zamienia się w porównanie (format eksportu, konwencje znaków reakcji i momentu).
- Siły w łożyskach elementów (Fbx, Fby), momenty zewnętrzne, `J1`/`J2`, suwak z `XL ≠ 0`: po ustaleniu znaczenia pól.
