# MechanismCraft — projekt i mapa drogowa

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
