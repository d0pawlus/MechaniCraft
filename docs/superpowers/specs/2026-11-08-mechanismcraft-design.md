# MechaniCraft — projekt i mapa drogowa

**Data:** 2026-11-08
**Cel:** współczesny, wieloplatformowy program w Ruście do syntezy, analizy (ruch i siły) i optymalizacji mechanizmów płaskich, funkcjonalnie zbliżony do SAM 8.5 (ARTAS) i potrafiący wczytywać jego projekty.

## 1. Zasady (clean-room)

- **Nie dekompilujemy `Sam85.exe` ani `SamDLL32.dll`.** Nie kopiujemy kodu, zasobów, ikon, tekstów pomocy ani nazwy produktu.
- Źródła wiedzy: (a) publiczna literatura metody elementów skończonych dla mechanizmów (niżej), (b) opis funkcji z pomocy SAM, przepisany własnymi słowami, (c) zachowanie programu obserwowane z zewnątrz (eksporty wyników jako dane referencyjne), (d) analiza własnych plików `.SAM` wyłącznie w celu interoperacyjności (import).
- Własna nazwa, własny interfejs, własny format natywny (`.mcraft`, JSON/RON). `.SAM` jest tylko formatem importu.

## 2. Co ustaliliśmy o SAM 8.5

- 32-bitowy program w Embarcadero C++Builder (VCL), solver w `SamDLL32.dll`.
- Jądro obliczeniowe to **metoda elementów skończonych dla kinematyki** (van der Werff 1977, Klein Breteler 1987, Schwab i Meijaard 1999). Wszystkie mechanizmy (otwarte, zamknięte, wielopętlowe, planetarne) są modelowane jednakowo: węzły, elementy i ich mody deformacji.
- Elementy: belka, suwak, suwak krzywoliniowy, koło zębate, pas, zębatka z kołem, czujnik, sprężyna, tłumik, tarcie (każde w wersji obrotowej i liniowej), sprężyna nieliniowa, sprężyna gazowa. W pliku występują też `HINGE` (sprzężenie względnego kąta, `FIXED:1` oznacza sztywne) i `ROTMOTOR` (napęd względnego obrotu).
- Do 10 niezależnych wejść. Prawa ruchu: liniowe, sinusoidalne, wielomian 5. stopnia, trapezowe, splajn sześcienny, plik ruchu (ASCII `t u v a` w SI).
- Wyniki: położenia, przemieszczenia, prędkości i przyspieszenia węzłów i kątów; kinetostatyka (moment lub siła napędowa, reakcje podpór, siły w łożyskach, siły wewnętrzne, moc); wyniki użytkownika przez parser wzorów.
- Optymalizacja: eksploracja Monte Carlo i algorytm ewolucyjny, potem lokalnie simplex lub algorytm ewolucyjny; ograniczenia jako kary, funkcja odległości do wykrywania kolizji.
- Kreatory: mechanizm 4-członowy, synteza 3 położeń, synteza funkcji kąta, mechanizmy prostowodowe (Watt, Roberts, Czebyszew, Hoecken, Evans).
- Wejście i wyjście: import i eksport DXF, eksport wyników do ASCII, krzywe (tor, hodograf, centroidy, ewoluta).

### Format `.SAM` (od wersji 5.x tekstowy)
```
SAMASCII
802                        <- wersja formatu
BUILDNRREAD:8.0.1.161
 ELEMGROUPS: / ELEMGRP: / ELEMLIST: / <TYP>: ... ENDOBJ
DOFLIST:  NODE:{NR,FIXAXIS,X,Y,...}  ANGLE:{NR,FIXAXIS,A}
INPUT:    MOTIONBLOCK:{EDNR,AXIS,KIND, CURVELIST: CLINEAR|CSINQ|CMOTIONFILE ...}
GRAVITY:, UNITS:, PROJECTOPTIONS:, GRAPHOPTIONS:, OPTIMIZATION:, ...
ENDSAM
```
- `FIXAXIS` to maska bitowa: 1 = x, 2 = y, 4 = kąt.
- Kąty mają numery od `FIRSTANGLENR+1` (zwykle 1001).
- Każdy element ma własne kątowe stopnie swobody w węzłach (`A1`, `A2`). Przegub to brak wspólnego kąta, a sztywne połączenie to `HINGE` z `FIXED:1`.
- 22 przykłady leżą w `C:\Users\Public\Documents\Sam\Examples`.
- Ustalone przy imporcie (Task 6): nagłówek `CREATORTYPE_PRO` (token bez dwukropka na wcięciu 0) zamyka bloki, więc `ELEMGROUPS`, `DOFLIST`, `INPUT` są korzeniami drzewa. `FIXAXIS` węzła: 1 = x, 2 = y; kąta: 4. W `MOTIONBLOCK` krzywe (`CLINEAR`, `CSINQ`, `CMOTIONFILE`) leżą obok `CURVELIST`. Wejście `KIND:1` działa na element: `ROTMOTOR` to obrót względny, a `BEAM` z `ELONGATION:1` to wydłużenie belki (np. `BED.SAM`, wymaga wsparcia w `mc-core`).
- Blok `LINETYPE<n>:` (styl rysowania) bywa zapisany ze stałym wcięciem 1 wewnątrz bloku elementu (`SCS.sam`) i ma własny `ENDOBJ`. Parser pomija go w całości.

- Obciążenia (ustalone przy planowaniu fazy 2, z przykładów i pomocy): masa skupiona w węźle to blok `MASS:` z kluczem `MASSVALUE` wewnątrz `DOFLIST/NODE`. `BEAM` (i `SLIDER`) mają `M` (masa), `XL` (położenie środka masy jako ułamek długości od `N1`, 0–1) i `J` (moment bezwładności względem środka masy). `HINGE` i `ROTMOTOR` mają `J1`, `J2` o nieustalonym znaczeniu. Blok `GRAVITY:` ma `VALUE` (m/s²) i `ANGLE` (kierunek wektora w radianach, −π/2 to w dół); `POSX`, `POSY` to tylko położenie symbolu na rysunku. Brak bloku oznacza brak grawitacji.
- Siła zewnętrzna to `FORCEBLOCK:` wewnątrz `DOFLIST/NODE` (`EDNR` = numer węzła, `ISTORQUE`, `DISABLED`). Definicję niesie dziecko `FORCEANYARG:`: `FNAME` (plik, zwykle pusty), `ARGTYPE` i `EDNR` (2 = węzeł, 1 = element), `ARGPROP` (np. `X`, `Y`, `A1`) oraz `TABLE:` z wierszami `argument wartość kąt` (kąt bezwzględny w radianach). Zewnętrzne `ARGTYPE` bloku bywa różne od wewnętrznego, więc czytamy tylko wewnętrzne.

## 3. Architektura docelowa

```
crates/
  mc-core      model (węzły, współrzędne, elementy), mody deformacji, solver kinematyki
  mc-motion    prawa ruchu i harmonogram wejść
  mc-kineto    kinetostatyka (faza 2)
  mc-samfile   import .SAM
  mc-io        format natywny, eksport CSV/DXF
  mc-opt       optymalizacja (faza 6)
  mc-cli       narzędzie wiersza poleceń (testy, wsad)
  mc-app       interfejs graficzny (egui, desktop i WASM)
```
Jądro (`mc-core`, `mc-motion`, `mc-kineto`) nie zależy od interfejsu ani od plików, więc da się je testować bez okien.

## 4. Fazy

| Faza | Zakres | Gotowe, gdy |
|---|---|---|
| **1. Rdzeń kinematyki** | belka, suwak, `HINGE`, `ROTMOTOR`, podpory; ruch `CLINEAR`; położenie, prędkość, przyspieszenie; import `.SAM` (podzbiór); CLI → CSV | 10 przykładów SAM liczy się bez błędu; korbowód zgodny z rozwiązaniem analitycznym (1e-9) |
| 2. Kinetostatyka | masy, momenty bezwładności, grawitacja, siły zewnętrzne; moment napędowy, reakcje, siły wewnętrzne, moc | moment napędowy korbowodu zgodny z zasadą mocy wirtualnych; zgodność z eksportem SAM |
| 3. Podgląd | `mc-app`: wczytanie, animacja, wykresy wyników, tor punktu | otwarcie `SLCR.SAM` i animacja z wykresem x(t) |
| 4. Edytor | rysowanie członów, przyciąganie, podpory, właściwości, cofnij i ponów, format natywny | zbudowanie mechanizmu 4-członowego od zera w interfejsie |
| 5. Elementy | koło zębate, pas, zębatka, suwak krzywoliniowy, sprężyna, tłumik, tarcie, sprężyna gazowa i nieliniowa; pozostałe prawa ruchu i plik ruchu | wszystkie 22 przykłady SAM wczytują się i liczą |
| 6. Optymalizacja | parser wzorów, cele (szczyt, RMS, odchylenie od toru), ograniczenia przez kary, algorytm ewolucyjny i Nelder–Mead | odtworzenie optymalizacji toru z przykładu |
| 7. Synteza i dodatki | kreatory (4-członowy, 3 położenia, prostowody), DXF, centroidy i hodografy | kreator generuje mechanizm Watta |

Każda faza ma osobny plan implementacji w `docs/superpowers/plans/`.

## 5. Weryfikacja

1. **Analitycznie:** zamknięte wzory (korbowód, czworobok przegubowy) w testach jednostkowych.
2. **Wyrocznia SAM:** dla wybranych przykładów użytkownik eksportuje z SAM (Results → Export) przebiegi do `fixtures/sam/<nazwa>.txt`. Testy porównują je z tolerancją 1e-6 względem zakresu.
3. **Spójność:** prędkości i przyspieszenia sprawdzane różnicami skończonymi położeń.

## 6. Literatura (metoda)
- K. van der Werff, *Kinematic and dynamic analysis of mechanisms, a finite element approach*, praca doktorska, Delft 1977.
- A. J. Klein Breteler, *Kinematic optimization of mechanisms, a finite element approach*, praca doktorska, Delft 1987.
- A. L. Schwab, J. P. Meijaard, *The belt, gear, bearing and hinge as special finite elements…*, IFToMM 1999.
- J. B. Jonker, *A finite element dynamic analysis of spatial mechanisms with flexible links*, 1989 (dalsze rozwinięcie).

## 7. Kinetostatyka (faza 2): decyzje projektowe

- **Metoda.** Analiza quasi-statyczna na gotowej trajektorii kinematycznej, bez całkowania w czasie. W każdym stanie rozwiązujemy równanie mocy wirtualnych `Dᵀσ = f_zew + f_graw − f_bezwł` na współrzędnych swobodnych, gdzie `D = ∂e/∂q` to ten sam jakobian modów co w solverze kinematyki (wspólny kod w `mc-core`, `ModeSystem`). Niewiadome to naprężenia uogólnione `σ` modów sztywnych i napędzanych. Mody swobodne nie przenoszą obciążenia (`σ = 0`), bo sprężyny, tłumiki i tarcie przychodzą w fazie 5.
- **Reakcje.** Po wyznaczeniu `σ` reakcja na współrzędnej zablokowanej `k` to `r_k = (Dᵀσ)_k + f_bezwł,k − f_zew,k − f_graw,k`, czyli siła (albo moment) wywierana przez podporę na mechanizm.
- **Znaki.** `σ` modu długości belki to siła normalna (dodatnia przy rozciąganiu, jak w SAM). Napęd: siła lub moment napędowy to `−σ` modu napędzanego (moment, który napęd przykłada do kąta `A2` względem `A1`), a moc napędu to `−σ · ė`. Zgodność znaków z SAM sprawdzi wyrocznia eksportu.
- **Bezwładność.** Masa skupiona w węźle działa tylko translacyjnie. Masa członu leży w punkcie `(1−XL)·p + XL·q`, a moment bezwładności `J` działa na kąt prostej członu `α = atan2(q − p)`, z członem konwekcyjnym w `α̈`. Suwak z masą poza węzłem `N1` (`XL ≠ 0`) oraz `J1`, `J2` przegubów i napędów są na razie nieobsługiwane (czytelny błąd).
- **Siły zewnętrzne.** Tablica `argument → (wartość, kąt)` interpolowana liniowo, poza zakresem stała (wartości skrajne). Argument: czas albo współrzędna `X`/`Y` węzła w bieżącym stanie. Momenty zewnętrzne (`ISTORQUE:1`), siły z pliku i argumenty elementowe są nieobsługiwane.
- **Crate.** `mc-kineto` zależy od `mc-core`. `mc-samfile` dostaje `import_project` (mechanizm i obciążenia), a `import_sam` pozostaje bez zmian.
- **Wyrocznia.** Eksporty wyników z SAM trafiają do `fixtures/sam-exports/` (tylko eksporty, nigdy pliki projektów). Test porównujący pomija się, gdy katalog nie ma eksportów.

