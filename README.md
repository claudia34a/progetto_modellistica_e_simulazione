# Progetto di Modellistica e Simulazione

Autori: Claudia Agosti, Greta Brognoli, Matteo Cropelli  
Corso: Modellistica e Simulazione  
Anno Accademico: 2021/2022

## Descrizione del progetto

Il progetto è suddiviso in tre esercizi principali, ciascuno focalizzato su un diverso ambito della modellistica e dell'analisi dei sistemi dinamici:

---

### Esercizio 1 - Modellizzazione di un quadricottero

In questo esercizio è stato modellato un drone (quadricottero) tramite la definizione delle sue **variabili di stato**, **ingressi**, **uscite**, e le relative equazioni di:

- cinematica rotazionale e traslazionale
- dinamica rotazionale e traslazionale

Sono state definite le condizioni di equilibrio e analizzata la **stabilità locale** del sistema tramite linearizzazione.  
Il punto di equilibrio considerato corrisponde a una configurazione statica in cui il drone è sospeso nel vuoto, con input costante per contrastare la forza di gravità.

---

### Esercizio 2 - Analisi di serie storiche

Obiettivo: analizzare una serie storica relativa alle emissioni di CO₂ e temperatura globale.

Attività svolte:
- Pre-processing dei dati forniti (serie WorldTemperature e co-emissions-per-capita).
- Suddivisione in dati di identificazione (righe dispari) e validazione (righe pari).
- Costruzione di modelli autoregressivi con parte esogena, variando l’ordine fino a 3.
- Valutazione della bontà del modello tramite metriche MAE e NMAE.
- Previsione della temperatura fino all’anno 2100 con il miglior modello selezionato.

Osservazioni:
- I modelli presentano errori elevati, attribuibili alla limitata quantità di dati e a eventi climatici non previsti.

---

### Esercizio 3 - Studio di un sistema dinamico non lineare

Dato un sistema non lineare con ingressi e uscite, è stata svolta un’analisi completa in tre parti:

#### (a) Analisi del sistema e progettazione del controllo

- Calcolo dei **punti di equilibrio** per `u = 0` e analisi di stabilità tramite autovalori della jacobiana.
- Valutazione di quale uscita (tra `y₁ = 5x₂` e `y₂ = 2x₁`) sia più adatta alla **linearizzazione ingresso-uscita**. Si è selezionata `y₂` per il grado relativo 2.
- Progettazione del **controllo linearizzante**, ottenendo l’espressione dell’ingresso `u_lin` in funzione degli stati e di un nuovo ingresso `v`.
- Definizione di un controllo in **retroazione di stato**, con autovalori `[a₁, 2a₁]`, tali da raggiungere l’equilibrio in `T = 2s`. Si è ricavato il guadagno `K = [12.5, 7.5]`.

#### (b) Linearizzazione del sistema

- Linearizzazione del sistema attorno al punto di equilibrio `[0, 0]`.
- Calcolo delle matrici del sistema linearizzato:
  - A = \[\[0, 0\]; \[1, -4\]\]
  - B = \[\[0\]; \[3\]\]
  - C (uscite): \[0, 5\] per `y₁`, \[2, 0\] per `y₂`

#### (c) Simulazione

- Simulazione del sistema in due versioni:
  - Linearizzazione classica
  - Linearizzazione ingresso-uscita

Con condizione iniziale `x₀ = [3; 5]`, si è verificato che solo `y₂_lin` converge a zero, confermando la corretta selezione dell’uscita per la linearizzazione e la validità del controllo progettato.


## Note finali

Il progetto ha permesso di applicare in modo concreto le tecniche di modellazione dinamica, analisi di sistemi non lineari e progettazione di controlli, unendo teoria e simulazione tramite strumenti MATLAB.
