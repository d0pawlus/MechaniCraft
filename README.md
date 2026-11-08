# MechaniCraft

Współczesne, wieloplatformowe narzędzie w Ruście do analizy i (docelowo) syntezy oraz optymalizacji **mechanizmów płaskich**. Kinematykę liczy metodą elementów skończonych (van der Werff, Klein Breteler).

> Projekt clean-room: nie zawiera kodu, zasobów ani plików przykładowych innych programów. Import plików `.SAM` (SAM 8.5) służy wyłącznie interoperacyjności.

## Stan: faza 1, rdzeń kinematyki
- Elementy: belka (także jako siłownik liniowy), suwak, przegub (`HINGE`), napęd obrotowy (`ROTMOTOR`), podpory x/y/kąt.
- Ruch wejściowy: liniowy (wiele segmentów).
- Wyniki: położenia, prędkości i przyspieszenia wszystkich współrzędnych (Newton-Raphson).
- Import `.SAM` (podzbiór); nieobsługiwana treść daje czytelny błąd z numerem linii.

## Użycie
```bash
cargo test --workspace
cargo run -p mc-cli -- analyze projekt.sam -o wyniki.csv
cargo run -p mc-cli -- analyze projekt.sam --nodes 3
cargo run -p mc-cli -- check katalog_z_projektami
```
Kody wyjścia `analyze`: `0` pełny wynik, `2` wynik częściowy (CSV zapisany, błąd na stderr), `1` błąd importu lub argumentów.

## Struktura
| Crate | Rola |
|---|---|
| `mc-motion` | prawa ruchu wejść, siatka czasu |
| `mc-core` | model mechanizmu, mody deformacji elementów, solver kinematyki |
| `mc-samfile` | parser i import plików SAM ASCII |
| `mc-cli` | narzędzie `mcraft` |

Testy na przykładach SAM czytają je z `SAM_EXAMPLES_DIR` (domyślnie `C:\Users\Public\Documents\Sam\Examples`) i pomijają się, gdy katalogu nie ma.

## Dokumentacja
- [Projekt i mapa drogowa](docs/superpowers/specs/2026-11-08-mechanismcraft-design.md)
- [Plan fazy 1](docs/superpowers/plans/2026-11-08-phase1-kinematic-core.md)
