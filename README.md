# MechaniCraft

Współczesne, wieloplatformowe narzędzie w Ruście do analizy i (docelowo) syntezy oraz optymalizacji **mechanizmów płaskich**. Kinematykę liczy metodą elementów skończonych (van der Werff, Klein Breteler).

> Projekt clean-room: nie zawiera kodu, zasobów ani plików przykładowych innych programów. Import plików `.SAM` (SAM 8.5) służy wyłącznie interoperacyjności.

## Stan: faza 2, kinetostatyka
- Elementy: belka (także jako siłownik liniowy), suwak, przegub (`HINGE`), napęd obrotowy (`ROTMOTOR`), podpory x/y/kąt.
- Ruch wejściowy: liniowy (wiele segmentów).
- Wyniki: położenia, prędkości i przyspieszenia wszystkich współrzędnych (Newton-Raphson).
- Obciążenia: masy węzłowe, masy i momenty bezwładności członów, grawitacja, siły zewnętrzne w węzłach (także tablicowe, zależne od położenia węzła), wczytywane również z `.SAM`.
- Kinetostatyka (analiza quasi-statyczna): moment lub siła napędowa i moc każdego wejścia, reakcje na współrzędnych zablokowanych, siły normalne belek.
- Import `.SAM` (podzbiór); nieobsługiwana treść daje czytelny błąd z numerem linii.

## Użycie
```bash
cargo test --workspace
cargo run -p mc-cli -- analyze projekt.sam -o wyniki.csv
cargo run -p mc-cli -- analyze projekt.sam --nodes 3
cargo run -p mc-cli -- analyze projekt.sam --forces
cargo run -p mc-cli -- check katalog_z_projektami
```
Z `--forces` po kolumnach węzłów dochodzą: `DRV<nr>`, `PWR<nr>` (siła lub moment napędowy i moc każdego wejścia), `RX<nr>`, `RY<nr>`, `RA<nr>` (reakcje na współrzędnych zablokowanych: węzły rosnąco, potem kąty) i `N<nr>` (siła normalna belki, dodatnia przy rozciąganiu). `check` sprawdza import, kinematykę i kinetostatykę.

Kody wyjścia `analyze`: `0` pełny wynik, `2` wynik częściowy kinematyki lub sił (CSV zapisany, błąd na stderr), `1` błąd importu lub argumentów.

## Struktura
| Crate | Rola |
|---|---|
| `mc-motion` | prawa ruchu wejść, siatka czasu |
| `mc-core` | model mechanizmu, mody deformacji elementów, solver kinematyki |
| `mc-kineto` | model obciążeń, siły uogólnione, solver kinetostatyki |
| `mc-samfile` | parser i import plików SAM ASCII |
| `mc-cli` | narzędzie `mcraft` |

Testy na przykładach SAM czytają je z `SAM_EXAMPLES_DIR` (domyślnie `C:\Users\Public\Documents\Sam\Examples`) i pomijają się, gdy katalogu nie ma.

## Dokumentacja
- [Projekt i mapa drogowa](docs/superpowers/specs/2026-11-08-mechanismcraft-design.md)
- [Plan fazy 1](docs/superpowers/plans/2026-11-08-phase1-kinematic-core.md)
- [Plan fazy 2](docs/superpowers/plans/2026-11-09-phase2-kinetostatics.md)
