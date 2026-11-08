# Faza 1: rdzeń kinematyki, plan implementacji

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Biblioteka i CLI w Ruście, które wczytują płaskie mechanizmy z plików SAM (`.SAM`, podzbiór) i liczą położenia, prędkości oraz przyspieszenia wszystkich współrzędnych metodą elementów skończonych dla kinematyki.

**Architecture:** Model składa się ze współrzędnych (x i y węzłów oraz kąty węzłowe elementów) i elementów. Każdy element udostępnia mody deformacji o roli sztywnej, swobodnej albo napędzanej. Solver Newtona-Raphsona spełnia równania `e_i(q) = cel_i(t)` dla niezablokowanych współrzędnych. Prędkości i przyspieszenia wynikają z tego samego jakobianu. Import `.SAM` buduje drzewo z wcięć linii, a z drzewa model.

**Tech Stack:** Rust 1.99 (edition 2024), `nalgebra` 0.33 (DMatrix, LU), `thiserror` 2, `clap` 4 (derive), `anyhow` 1 (tylko CLI), `approx` 0.5 (testy).

**Spec:** `docs/superpowers/specs/2026-11-08-mechanismcraft-design.md` (rozdziały 1, 2 i faza 1 z tabeli w rozdziale 4)

## Global Constraints

- Zasada clean-room: żadnego kodu ani zasobów z `Sam85.exe` lub `SamDLL32.dll`. Pliki przykładów SAM **nie trafiają do repozytorium**. Testy integracyjne czytają je z `SAM_EXAMPLES_DIR` (domyślnie `C:\Users\Public\Documents\Sam\Examples`) i pomijają się z komunikatem, gdy katalogu nie ma.
- Jednostki wewnętrzne: SI (m, rad, s). Współrzędne w `.SAM` traktujemy jako SI.
- Biblioteki nie panikują na danych wejściowych. Każdy błąd jest typem z `thiserror`.
- Tolerancje: Newton do `|r|∞ < 1e-12`, maksymalnie 50 iteracji. Testy analityczne: położenie 1e-9, prędkość 1e-8, przyspieszenie 1e-6.
- Faza 1 obsługuje elementy `BEAM`, `SLIDER`, `HINGE` i `ROTMOTOR`, wejścia `KIND:1` (deformacja elementu) oraz segmenty `CLINEAR`. Każda inna treść kończy się błędem `Unsupported` z numerem linii.

## Review Focus

1. **Pełny obrót korby.** Kąty węzłowe przekraczają ±π, a `atan2` przeskakuje. Mody zginania muszą zawijać różnicę do (-π, π], a współrzędne kątowe pozostają ciągłe. Test w Task 4: `slider_crank_full_revolution_matches_analytic`.
2. **Położenie martwe lub poza zasięgiem.** Napęd wypycha mechanizm poza zakres montażu. Oczekiwany wynik to błąd z numerem kroku i zachowana częściowa trajektoria, bez paniki. Test w Task 4: `unreachable_input_reports_step_and_keeps_partial`.
3. **Przeskok gałęzi przy grubym kroku.** Czworobok z 12 krokami na obrót nie może zmienić konfiguracji (znak iloczynu wektorowego łącznik × wahacz stały). Test w Task 4: `four_bar_coarse_steps_keep_branch`.
4. **Nieobsługiwana treść `.SAM`.** `GEAR`, `CSINQ`, `KIND:2` lub binarny plik SAM 3/4 dają czytelny błąd z typem i linią, a nie ciche pominięcie. Testy w Task 6.
5. **Kodowanie pliku.** Notatki projektu mogą zawierać bajty Windows-1252 (np. polskie znaki). Wczytanie nie może się na nich wyłożyć. Test w Task 5: `parses_non_utf8_bytes_lossily`.

---

## Struktura plików

```
Cargo.toml                       workspace
crates/mc-motion/src/lib.rs      InputLaw, Segment, siatka czasu
crates/mc-core/src/lib.rs        re-eksporty
crates/mc-core/src/model.rs      CoordId, Coord, Element, Mechanism, MechanismBuilder, ModelError
crates/mc-core/src/modes.rs      ModeRole, ModeEval, element_modes()
crates/mc-core/src/solver.rs     State, Trajectory, SolveError, analyze()
crates/mc-core/tests/analytic.rs testy korbowodu i czworoboku
crates/mc-samfile/src/tree.rs    SamTree (parser wcięć)
crates/mc-samfile/src/import.rs  import_sam() -> Mechanism
crates/mc-samfile/tests/examples.rs  testy na przykładach SAM_EXAMPLES_DIR
crates/mc-cli/src/main.rs        `mcraft analyze`, `mcraft check`
```

---

### Task 1: Workspace i prawo ruchu liniowego

**Files:**
- Create: `Cargo.toml`, `.gitignore`, `crates/mc-motion/Cargo.toml`, `crates/mc-motion/src/lib.rs`

**Interfaces:**
- Produces:
  - `pub enum Segment { Linear { duration: f64, steps: u32, displacement: f64 } }`
  - `pub struct InputLaw { pub segments: Vec<Segment> }`
  - `impl InputLaw { pub fn eval(&self, t: f64) -> (f64, f64, f64) /* u, v, a */; pub fn time_grid(&self) -> Vec<f64>; pub fn duration(&self) -> f64 }`

- [ ] **Step 1:** `git init`, utworzyć workspace (`members = ["crates/*"]`, `resolver = "3"`) i `.gitignore` z `/target`.
- [ ] **Step 2: Napisać testy, które na razie nie przechodzą** w `crates/mc-motion/src/lib.rs` (`#[cfg(test)]`):
  - `linear_eval`: `Linear{duration:1.0, steps:36, displacement:2π}` → `eval(0.5) == (π, 2π, 0.0)`.
  - `segments_chain`: dwa segmenty `(1.0, 10, 1.0)` i `(2.0, 10, -1.0)` → `eval(1.0).0 == 1.0`, `eval(3.0).0 == 0.0`, `eval(2.0).1 == -0.5`.
  - `time_grid_counts`: siatka z tych dwóch segmentów ma 21 punktów, zaczyna się od `0.0`, kończy na `3.0`, a punkt 10 to `1.0`.
  - `eval_clamps_beyond_end`: `eval(5.0)` zwraca pozycję końcową i `v = a = 0`.
- [ ] **Step 3:** `cargo test -p mc-motion`. Oczekiwane: FAIL (brak typów).
- [ ] **Step 4:** Implementacja. Segmenty występują po sobie, a każdy zaczyna z pozycji, na której skończył się poprzedni.
- [ ] **Step 5:** `cargo test -p mc-motion`. Oczekiwane: PASS.
- [ ] **Step 6:** `git commit -m "feat(motion): linear input law and time grid"`

### Task 2: Model mechanizmu i builder

**Files:**
- Create: `crates/mc-core/Cargo.toml`, `crates/mc-core/src/lib.rs`, `crates/mc-core/src/model.rs`

**Interfaces:**
- Consumes: `mc_motion::InputLaw`
- Produces:
  - `pub struct CoordId(pub usize)`
  - `pub enum CoordLabel { NodeX(u32), NodeY(u32), Angle(u32) }`
  - `pub struct Coord { pub label: CoordLabel, pub value0: f64, pub fixed: bool }`
  - `pub enum ElementKind { Beam { p: [CoordId; 2], q: [CoordId; 2], a: CoordId, b: CoordId }, Slider { p: [CoordId; 2], q: [CoordId; 2], a: CoordId, b: CoordId }, Hinge { a: CoordId, b: CoordId, fixed: bool }, RotMotor { a: CoordId, b: CoordId, fixed: bool } }`
  - `pub struct Element { pub nr: u32, pub kind: ElementKind }`
  - `pub struct Input { pub element_nr: u32, pub law: InputLaw }`
  - `pub struct Mechanism { pub coords: Vec<Coord>, pub elements: Vec<Element>, pub inputs: Vec<Input> }` z `fn node(&self, nr: u32) -> Option<(CoordId, CoordId)>` i `fn angle(&self, nr: u32) -> Option<CoordId>`
  - `pub struct MechanismBuilder` z metodami `node(nr, x, y, fix_x: bool, fix_y: bool)`, `angle(nr, a0, fixed)`, `beam(nr, n1, n2, a1, a2)`, `slider(nr, n1, n2, a1, a2)`, `hinge(nr, a1, a2, fixed)`, `rot_motor(nr, a1, a2, fixed)`, `input(element_nr, law)` (każda zwraca `&mut Self`) oraz `build(&self) -> Result<Mechanism, ModelError>`
  - `pub enum ModelError { DuplicateNode(u32), DuplicateAngle(u32), DuplicateElement(u32), UnknownNode { element: u32, node: u32 }, UnknownAngle { element: u32, angle: u32 }, UnknownInputElement(u32), ZeroLengthBeam(u32), InputOnNonDrivable(u32) }`

- [ ] **Step 1: Napisać testy, które na razie nie przechodzą** (`model.rs`, `#[cfg(test)]`):
  - `builds_slider_crank`: węzły 1 `(0,0,fix xy)`, 2 `(0.1,0)`, 3 `(0.4,0, fix y)`; kąty 1001–1004 swobodne, 1005 zablokowany; belki 1 `(1,2,1001,1002)` i 2 `(2,3,1003,1004)`; `rot_motor(3, 1005, 1001, false)`; `input(3, …)`. Oczekiwane: `coords.len() == 11`, liczba niezablokowanych == 7.
  - `rejects_unknown_node`: belka do węzła 9 → `UnknownNode{element:1,node:9}`.
  - `rejects_zero_length_beam`: oba węzły w tym samym punkcie → `ZeroLengthBeam(1)`.
  - `rejects_input_on_beam_in_phase1`: wejście na element typu `Beam` → `InputOnNonDrivable(1)` (faza 1 napędza tylko `RotMotor`, `Hinge` i `Slider`).
- [ ] **Step 2:** `cargo test -p mc-core model`. Oczekiwane: FAIL.
- [ ] **Step 3:** Implementacja.
- [ ] **Step 4:** `cargo test -p mc-core model`. Oczekiwane: PASS.
- [ ] **Step 5:** `git commit -m "feat(core): mechanism model and builder"`

### Task 3: Mody deformacji elementów

**Files:**
- Create: `crates/mc-core/src/modes.rs`

**Interfaces:**
- Consumes: `Mechanism`, `ElementKind`, `CoordId`
- Produces:
  - `pub enum ModeRole { Rigid, Free, Driven(usize /* indeks w mech.inputs */) }`
  - `pub struct ModeEval { pub value: f64, pub grad: Vec<(CoordId, f64)>, pub conv: f64 }`, gdzie `conv = q̇ᵀ (∂²e/∂q²) q̇`
  - `pub fn element_modes(mech: &Mechanism, el: &Element, q: &[f64], qd: &[f64]) -> Vec<ModeEval>`
  - `pub fn mode_roles(mech: &Mechanism) -> Vec<(usize /* element idx */, usize /* mode idx */, ModeRole)>`

Definicje modów (każdy mierzony względem konfiguracji początkowej, więc w chwili 0 wszystkie są równe zeru). Oznaczenia: `d = q_xy − p_xy`, `L = |d|`, `α = atan2(d)`, indeks 0 to wartość początkowa, `wrap` sprowadza do (−π, π]:

| Element | Mod 0 | Mod 1 | Mod 2 |
|---|---|---|---|
| Beam | `L − L0`: Rigid (siłownik liniowy dopiero w fazie 5) | `wrap((φa−φa0) − (α−α0))`: Rigid | `wrap((φb−φb0) − (α−α0))`: Rigid |
| Slider | `L − L0`: Free albo Driven | jak w Beam: Rigid | jak w Beam: Rigid |
| Hinge, RotMotor | `(φb−φb0) − (φa−φa0)`: Rigid gdy `fixed`, Driven gdy ma wejście, inaczej Free | – | – |

Człony konwekcyjne (ḋ = q̇_q − q̇_p):
```
conv(L) = (|ḋ|² − (d·ḋ)²/L²) / L
conv(α) = −2 (d×ḋ)(d·ḋ) / L⁴        // d×ḋ = dx·ḋy − dy·ḋx
conv(mod zginania) = −conv(α);  conv(Hinge/RotMotor) = 0
```

- [ ] **Step 1: Napisać testy, które na razie nie przechodzą:**
  - `modes_zero_at_initial`: dla mechanizmu z Task 2 przy `q = value0` każdy mod ma `value == 0`.
  - `gradients_match_finite_difference`: losowe zaburzenie `q` (ziarno stałe, amplituda 0.05), gradient każdego modu porównany z różnicą centralną (h=1e-7), tolerancja 1e-6.
  - `conv_matches_second_difference`: `conv ≈ (e(q+hq̇) − 2e(q) + e(q−hq̇))/h²` przy h=1e-4, tolerancja 1e-4.
  - `bending_mode_wraps`: belka obrócona o `α−α0 = 3.0` i `φa−φa0 = 3.0 − 2π` daje mod 1 ≈ 0.
  - `roles_for_slider_crank`: 7 ról. Mod 0 RotMotor to `Driven(0)`, a sześć modów belek to `Rigid`.
- [ ] **Step 2:** `cargo test -p mc-core modes`. Oczekiwane: FAIL.
- [ ] **Step 3:** Implementacja według tabeli.
- [ ] **Step 4:** `cargo test -p mc-core modes`. Oczekiwane: PASS.
- [ ] **Step 5:** `git commit -m "feat(core): element deformation modes"`

### Task 4: Solver kinematyki

**Files:**
- Create: `crates/mc-core/src/solver.rs`, `crates/mc-core/tests/analytic.rs`

**Interfaces:**
- Consumes: `element_modes`, `mode_roles`, `InputLaw::{eval, time_grid}`
- Produces:
  - `pub struct State { pub t: f64, pub q: Vec<f64>, pub qd: Vec<f64>, pub qdd: Vec<f64> }`
  - `pub struct Trajectory { pub states: Vec<State>, pub failure: Option<SolveError> }`
  - `pub enum SolveError { NotSquare { equations: usize, unknowns: usize }, NoInputs, InputGridMismatch, Singular { step: usize, t: f64 }, NoConvergence { step: usize, t: f64, residual: f64 } }`
  - `pub fn analyze(mech: &Mechanism) -> Result<Trajectory, SolveError>`. Błędy strukturalne (`NotSquare`, `NoInputs`, `InputGridMismatch`) zwracamy jako `Err`. Błąd w kroku k ustawia `failure` i zwraca stany 0..k−1.

Algorytm na krok k (siatka czasu z `inputs[0]`, wszystkie wejścia muszą mieć identyczną siatkę, inaczej `InputGridMismatch`):
1. Predyktor: `q = q_prev + q̇_prev·Δt + ½ q̈_prev·Δt²` (dla k=0: `q = value0`).
2. Newton po współrzędnych swobodnych: `r_i = e_i(q) − cel_i`, gdzie `cel = 0` dla Rigid i `u(t)` dla Driven. `J = ∂e/∂q_free`, rozkład LU z `nalgebra`. Nieodwracalne `J` → `Singular`, a po 50 iteracjach → `NoConvergence`.
3. Prędkości: `J q̇_free = ė_cel` (Driven: `v(t)`, Rigid: 0).
4. Przyspieszenia: `J q̈_free = ë_cel − conv(q, q̇)`.

- [ ] **Step 1: Napisać testy, które na razie nie przechodzą** w `tests/analytic.rs`:
  - `slider_crank_full_revolution_matches_analytic`: korba r=0.1, łącznik l=0.35, mimośród e=0.05 (węzeł 1 `(0,0.05)`, suwak na y=0), napęd `Linear{1.0, 36, 2π}`. Dla każdego kroku `x3 = r cosθ + sqrt(l² − (0.05 + r sinθ)²)`, `θ = θ0 + u(t)`. Pochodne liczone analitycznie i porównane z tolerancjami z Global Constraints. Dodatkowo wszystkie 37 stanów istnieją i `failure.is_none()`.
  - `four_bar_lengths_preserved_and_velocity_consistent`: czworobok (grunt 0.4, korba 0.1, łącznik 0.35, wahacz 0.3), 72 kroki. Długości stałe do 1e-10, a `q̇` zgodne z różnicą centralną położeń do 1e-4.
  - `four_bar_coarse_steps_keep_branch`: ten sam czworobok, 12 kroków. Znak `(łącznik × wahacz)` taki sam we wszystkich stanach.
  - `unreachable_input_reports_step_and_keeps_partial`: czworobok z korbą 0.3 i łącznikiem 0.1 (podwójny wahacz) napędzany o 2π. Oczekiwane: `failure` to `Singular` albo `NoConvergence` z `step > 0`, a `states.len() == step`.
  - `under_constrained_is_not_square`: korbowód bez blokady y węzła 3 → `Err(NotSquare{equations:7, unknowns:8})`.
- [ ] **Step 2:** `cargo test -p mc-core --test analytic`. Oczekiwane: FAIL.
- [ ] **Step 3:** Implementacja `analyze`.
- [ ] **Step 4:** `cargo test -p mc-core`. Oczekiwane: wszystko PASS.
- [ ] **Step 5:** `git commit -m "feat(core): Newton kinematic solver with velocity and acceleration"`

### Task 5: Parser drzewa `.SAM`

**Files:**
- Create: `crates/mc-samfile/Cargo.toml`, `crates/mc-samfile/src/lib.rs`, `crates/mc-samfile/src/tree.rs`

**Interfaces:**
- Produces:
  - `pub struct SamNode { pub key: String, pub value: String, pub line: usize, pub children: Vec<SamNode>, pub rows: Vec<String> }`
  - `pub struct SamTree { pub format_version: u32, pub root: Vec<SamNode> }`
  - `pub fn parse_tree(bytes: &[u8]) -> Result<SamTree, SamError>`
  - `impl SamNode { pub fn get(&self, key: &str) -> Option<&SamNode>; pub fn all<'a>(&'a self, key: &'a str) -> impl Iterator<Item=&'a SamNode>; pub fn num<T: FromStr>(&self, key: &str) -> Result<T, SamError> }`
  - `pub enum SamError { NotSamAscii, Missing { key: String, line: usize }, BadNumber { key: String, line: usize }, Unsupported { what: String, line: usize } }`

Reguły: tekst przez `String::from_utf8_lossy`. Linia 1 musi być `SAMASCII` (inaczej `NotSamAscii`), linia 2 to wersja formatu. Kolejne linie: `ENDOBJ` pomijamy. Linia z `:` daje węzeł `key:value` (podział na pierwszym `:`). Linia bez `:` trafia do `rows` rodzica. Rodzicem jest najbliższa wcześniejsza linia z dwukropkiem o mniejszym wcięciu (liczba spacji wiodących). `ENDSAM` kończy parsowanie.

- [ ] **Step 1: Napisać testy, które na razie nie przechodzą:**
  - `parses_nesting_by_indent`: fragment z `DOFLIST:` / ` NODE:` / `  NR:1` / `  X:-0.05` / ` ANGLE:` / `  NR:1001` → `root[0].all("NODE").count()==1`, `num::<f64>("X") == -0.05`.
  - `rows_attach_to_parent`: `     OPTIMIZATIONS:3` i trzy linie liczb z wcięciem 6 → `rows.len()==3`.
  - `rejects_binary`: bajty `[0x00, 0x01]` → `NotSamAscii`.
  - `parses_non_utf8_bytes_lossily`: `PROJECTNOTES:` z bajtem `0xB3` (ł w cp1250) nie zwraca błędu.
  - `missing_key_reports_line`: `num::<u32>("NR")` na węźle bez `NR` → `Missing{key:"NR", line:<linia węzła>}`.
- [ ] **Step 2:** `cargo test -p mc-samfile tree`. Oczekiwane: FAIL.
- [ ] **Step 3:** Implementacja (stos `(wcięcie, ścieżka)`).
- [ ] **Step 4:** `cargo test -p mc-samfile tree`. Oczekiwane: PASS.
- [ ] **Step 5:** `git commit -m "feat(samfile): indentation tree parser"`

### Task 6: Import `.SAM` do modelu

**Files:**
- Create: `crates/mc-samfile/src/import.rs`, `crates/mc-samfile/tests/examples.rs`

**Interfaces:**
- Consumes: `parse_tree`, `MechanismBuilder`, `InputLaw`, `Segment::Linear`
- Produces: `pub fn import_sam(bytes: &[u8]) -> Result<Mechanism, ImportError>`, gdzie `pub enum ImportError { Sam(SamError), Model(ModelError) }`

Mapowanie:
- `DOFLIST/NODE`: `NR`, `X`, `Y`, `FIXAXIS` (bit 1 to x, bit 2 to y).
- `DOFLIST/ANGLE`: `NR`, `A`, `FIXAXIS` (bit 4 oznacza zablokowany).
- `ELEMGROUPS/ELEMGRP/ELEMLIST/<TYP>`: `BEAM` i `SLIDER` mają `NR`, `N1`, `N2`, `A1`, `A2`; `HINGE` i `ROTMOTOR` mają `NR`, `A1`, `A2`, `FIXED`. Każdy inny typ → `Unsupported{what:"element GEAR", line}`.
- `INPUT/MOTIONBLOCK`: `EDNR` to element, `KIND` musi być 1 (inaczej `Unsupported{what:"input KIND 2"}`). Dzieci `CLINEAR` → `Segment::Linear{duration: CTIME, steps: NSTEPS, displacement: MOTION}`. Inne węzły `C*` poza `CURVELIST` → `Unsupported{what:"motion CSINQ"}`.

- [ ] **Step 1: Napisać testy, które na razie nie przechodzą:**
  - W `import.rs`: `imports_minimal_slider_crank` na własnym, ręcznie napisanym fragmencie `.SAM` w formacie z rozdziału 2 specyfikacji (3 węzły, 5 kątów, 2× `BEAM`, `ROTMOTOR`, `MOTIONBLOCK` z `CLINEAR NSTEPS:36 CTIME:1 MOTION:6.28318512`). Oczekiwane: 3 elementy, 1 wejście, `analyze` daje 37 stanów.
  - `gear_is_unsupported`: fragment z `GEAR:` → `Unsupported` z `what` zawierającym `GEAR` i poprawną linią.
  - `csinq_is_unsupported`: analogicznie dla `CSINQ:`.
  - W `tests/examples.rs`, gdy istnieje `SAM_EXAMPLES_DIR`: `slcr_matches_analytic`. `SLCR.SAM` importuje się, a `x3(t)` zgadza się z wzorem korbowodu z Task 4 dla geometrii odczytanej z pliku (r = |n2−n1|, l = |n3−n2|, e = y1−y3) z tolerancją 1e-9.
  - `phase1_examples_analyze`: dla `SLCR, 6BAR, CRRK, ORTHO, DYAD, SCS, TRFR, BED, QKRT, DXFDEMO (detached)` import i `analyze` zwracają `failure.is_none()`.
- [ ] **Step 2:** `cargo test -p mc-samfile`. Oczekiwane: FAIL.
- [ ] **Step 3:** Implementacja.
- [ ] **Step 4:** `cargo test -p mc-samfile`. Oczekiwane: PASS. Jeśli któryś przykład nie przejdzie, analizuj go według `superpowers:systematic-debugging` (najczęściej chodzi o znaczenie pola, którego nie znamy). Odkryte znaczenie dopisz do rozdziału 2 specyfikacji.
- [ ] **Step 5:** `git commit -m "feat(samfile): import SAM projects into mechanism model"`

### Task 7: CLI `mcraft`

**Files:**
- Create: `crates/mc-cli/Cargo.toml` (`[[bin]] name = "mcraft"`), `crates/mc-cli/src/main.rs`, `crates/mc-cli/tests/cli.rs`

**Interfaces:**
- Consumes: `import_sam`, `analyze`, `Trajectory`
- Produces:
  - `mcraft analyze <PLIK> [-o <CSV>] [--nodes 3,2]`: CSV z nagłówkiem `step,t,X3,Y3,VX3,VY3,AX3,AY3,...` (domyślnie wszystkie węzły, rosnąco wg NR), separator `,`, liczby w formacie `{:.12e}`, bez `-o` na stdout. Kod wyjścia 0, gdy trajektoria jest pełna, 2 przy `failure` (częściowy CSV jest zapisany, błąd idzie na stderr), 1 przy błędzie importu.
  - `mcraft check <KATALOG>`: dla każdego `*.sam`/`*.SAM` jedna linia `OK|UNSUPPORTED|FAILED  <nazwa>  <szczegół>`, na końcu podsumowanie. Kod wyjścia 1 tylko przy `FAILED`.

- [ ] **Step 1: Napisać testy, które na razie nie przechodzą** (`assert_cmd` 2 + `tempfile` 3 jako dev-deps):
  - `analyze_writes_csv`: minimalny fragment z Task 6 zapisany do pliku tymczasowego o nazwie ze spacją i nawiasami (`"demo (1).sam"`) → CSV ma 38 linii, a nagłówek zaczyna się od `step,t,X1,Y1`.
  - `check_reports_unsupported_without_failing`: katalog z plikiem z `GEAR:` → stdout zawiera `UNSUPPORTED`, kod wyjścia 0.
- [ ] **Step 2:** `cargo test -p mc-cli`. Oczekiwane: FAIL.
- [ ] **Step 3:** Implementacja (`clap` derive, `anyhow` w `main`).
- [ ] **Step 4:** `cargo test --workspace`. Oczekiwane: PASS. Potem ręcznie:
  `cargo run -p mc-cli -- check "C:\Users\Public\Documents\Sam\Examples"`. Oczekiwane: co najmniej 10 × `OK`, 0 × `FAILED`.
- [ ] **Step 5:** `git commit -m "feat(cli): mcraft analyze and check commands"`

---

## Po fazie 1 (poza tym planem)
- **Wyrocznia SAM:** użytkownik eksportuje z SAM 8.5 (Results → Select węzeł 3 → X, Vx, Ax → Export) przebieg dla `SLCR.SAM` i `CRRK.SAM` do `fixtures/sam/`. Pierwszy test fazy 2 dopasuje parser do rzeczywistego formatu eksportu.
- Następny plan: `2026-xx-xx-phase2-kinetostatics.md`.
