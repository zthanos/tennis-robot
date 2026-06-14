# Perimeter Survey PoC

Στόχος: να τρέχει το perimeter court survey από console, με ένα Docker command,
χωρίς να εξαρτάται από το web control panel. Το PoC χρησιμοποιεί το υπάρχον
file-backed command bus:

```text
scripts/run_perimeter_survey_poc.py
  -> runtime/robot_command.json: mode=map_court
  -> controller_node + lidar_survey_v2
  -> runtime/robot_status.json
  -> runtime/perimeter_survey_poc_summary.json
  -> runtime/court_boundary.json
```

## Εκτέλεση

Πρώτα ξεκίνα το Gazebo stack:

```bash
docker compose --profile gazebo up gazebo
```

Σε δεύτερο terminal, τρέξε το PoC:

```bash
docker compose --profile gazebo exec gazebo bash -lc \
  'python3 /workspace/scripts/run_perimeter_survey_poc.py --timeout-s 900'
```

Το script ξεκινά `map_court`, τυπώνει progress ανά survey state, και στο τέλος
στέλνει `idle` για cleanup.

## Αποτελέσματα

Κύρια αρχεία:

```text
runtime/perimeter_survey_poc_summary.json
runtime/court_boundary.json
runtime/survey_trace.jsonl
runtime/robot_status.json
```

Το PoC θεωρείται επιτυχημένο όταν το summary έχει:

```json
{
  "outcome": "success",
  "survey_complete": true
}
```

Αν πάρει `partial`, `timeout`, `stale_status`, ή `no_status`, κράτα το
`runtime/survey_trace.jsonl` για replay/debug.

## Χρήσιμες παραλλαγές

Πιο μεγάλο timeout:

```bash
docker compose --profile gazebo exec gazebo bash -lc \
  'python3 /workspace/scripts/run_perimeter_survey_poc.py --timeout-s 1200'
```

Να μην στείλει `idle` στο τέλος:

```bash
docker compose --profile gazebo exec gazebo bash -lc \
  'python3 /workspace/scripts/run_perimeter_survey_poc.py --no-stop-on-exit'
```

## Απόφαση για main survey

Αν αυτό το PoC σταθεροποιηθεί, το main survey μπορεί να χρησιμοποιήσει την ίδια
λογική σαν mission contract:

```text
start perimeter survey
wait for court_boundary.json
validate 4 sides / net / obstacle notes
hand boundary geometry to collection/search planner
```
