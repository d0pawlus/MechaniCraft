# Faza 3: podgląd (mc-view + mc-app)

**Cel:** otworzyć projekt `.SAM`, przeliczyć kinematykę i kinetostatykę, animować mechanizm, rysować wykresy wyników i tor wybranego węzła. Aplikacja działa na desktopie i w przeglądarce (WASM).

**Gotowe, gdy:** `SLCR.SAM` (sprawdzane na Windows) i wbudowany korbowód otwierają się z animacją i wykresem x(t); `cargo build -p mc-app --target wasm32-unknown-unknown` przechodzi w CI.

## Decyzje
- **Logika poza GUI.** Nowy crate `mc-view` (bez egui) trzyma wszystko, co da się testować bez okna: wczytanie dokumentu, geometria sceny dla klatki, odtwarzanie (czas, klatka, pętla), kanały wykresów i tor węzła. `mc-app` to cienka powłoka eframe.
- **Biblioteki:** `eframe 0.37` (backend glow), `egui_plot 0.38`, `rfd 0.18` (okno wyboru pliku na desktopie i w przeglądarce). Wersje sprawdzone w crates.io 2026-11-19.
- **Wczytywanie:** `import_project(&[u8])` przyjmuje bajty, więc ta sama ścieżka działa w WASM (rfd zwraca bajty) i przy przeciąganiu pliku na okno. Wbudowany przykład (własny korbowód z masą i grawitacją, `fixtures/demo/slider-crank.sam`) otwiera się na starcie, żeby wersja WEB miała co pokazać bez pliku.
- **Błędy:** błąd importu lub ustawienia analizy to komunikat w oknie, a nie panika. Wynik częściowy (awaria w kroku n) jest pokazywany do kroku n z ostrzeżeniem.
- **WASM:** binarka `mcraft-view` ma dwa `main`: desktopowy (`run_native`) i przeglądarkowy (`WebRunner` na płótnie `mc_canvas`, uruchamiany przez `spawn_local`). `index.html` jest dla `trunk`. CI buduje cel `wasm32-unknown-unknown`.
- **Obciążenia nieobsługiwane:** gdy `import_project` odrzuca same obciążenia, podgląd wczytuje mechanizm przez `import_sam` i pokazuje go bez sił, z ostrzeżeniem.
- **Poza zakresem:** edycja mechanizmu (faza 4), punkty sprzężone na członach (tor tylko dla węzłów), eksport wykresów, utrwalanie ustawień.

## Zadania
1. `mc-view::document` – `load(name, bytes) -> Result<Document, LoadError>`: import, kinematyka, kinetostatyka; ostrzeżenia dla wyników częściowych.
2. `mc-view::scene` – węzły i człony (belka, suwak) z mechanizmu; położenia w danym stanie; prostokąt obejmujący wszystkie stany.
3. `mc-view::playback` – czas, odtwarzanie, prędkość, pętla; numer klatki dla czasu.
4. `mc-view::channels` – lista kanałów (x, y, kąt i ich pochodne; siła/moment napędowy i moc wejść) z etykietami; próbki `(t, wartość)`; tor węzła.
5. `mc-app` – okno: panel boczny (plik, odtwarzanie, wybór kanału i węzła toru), widok mechanizmu (skalowanie do prostokąta, podpory, tor), wykres z pionową linią bieżącego czasu.
6. WASM: punkt wejścia w `main.rs`, `index.html`, budowanie w CI; README.
