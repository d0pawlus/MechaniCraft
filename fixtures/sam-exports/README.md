# Eksporty wyników z SAM (wyrocznia)

Ten katalog przechowuje eksporty **wyników** z SAM 8.5, służące jako wyrocznia
dla kinetostatyki. Nigdy nie wolno umieszczać tu plików `.SAM` ani tekstów
pomocy SAM.

## Jak przygotować eksport

1. Otworzyć w SAM 8.5 nieskrócony przykład z obciążeniami: `TRFR`, `BED`,
   `DYAD` lub `SCS`.
2. Uruchomić analizę, a następnie `Results -> Export` w formacie ASCII.
3. Wybrać wielkości: moment (siła) napędowy, moc, reakcje węzłów podporowych,
   siła normalna.
4. Zapisać plik jako `fixtures/sam-exports/<NAZWA>.txt`, gdzie `<NAZWA>` to
   nazwa pliku przykładu bez rozszerzenia (np. `TRFR.txt`).

## Stan

Test `sam_export_oracle_hook` w `crates/mc-samfile/tests/kinetostatics.rs`
pomija się, gdy nie ma tu plików `*.txt`. Gdy się pojawią, test celowo
kończy się błędem: porównanie nie jest jeszcze zaimplementowane i parser
trzeba dopasować do rzeczywistego formatu eksportu (zadanie następnej fazy).
