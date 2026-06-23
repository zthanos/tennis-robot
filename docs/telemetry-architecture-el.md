# Telemetry Architecture — Pi edge control + containerized dashboard (EL)

> Σκοπός: να κρατήσουμε **όλον τον έλεγχο στο Pi** (real-time, safety-critical) και
> να σπρώχνουμε το telemetry — status, map, sensor views — προς ένα **containerized
> web dashboard** με λογική IoT. Το dashboard **παρατηρεί** και στέλνει μόνο
> **supervisory** εντολές· δεν κλείνει ποτέ βρόχο ελέγχου πάνω από το δίκτυο.

Αυτό το σημείωμα τεκμηριώνει την αρχιτεκτονική-στόχο, τους μη-διαπραγματεύσιμους
κανόνες, τις επιλογές transport, και ένα συγκεκριμένο migration path από το
σημερινό `scripts/control_panel.py`.

---

## 1. Αρχή σχεδιασμού: edge control, offloaded observation

Δύο διακριτές περιοχές, με σαφές σύνορο:

| Περιοχή | Τι τρέχει | Πού | Απαιτήσεις |
| --- | --- | --- | --- |
| **Control plane** | ROS 2 graph: survey/collection FSM, slam_toolbox, motion, perception | **Pi (edge)** | real-time, ντετερμινιστικό, safety-critical |
| **Observation plane** | dashboard UI, sensor views, ιστορικό, alerts | **Container (οπουδήποτε)** | non-real-time, ανεκτικό σε καθυστέρηση/απώλεια |

Ο κανόνας: ό,τι έχει deadline ή αφορά ασφάλεια μένει στο Pi. Ό,τι είναι «να το
βλέπω/να το ελέγχω εποπτικά» φεύγει στο container. Αυτή είναι η καθιερωμένη
robotics + IoT edge τοπολογία.

---

## 2. Τι έχουμε ήδη (και είναι σωστό)

Το σημερινό setup είναι ήδη ~80% αυτής της αρχιτεκτονικής:

- **File-based IPC ως πηγή αλήθειας στο Pi.** `RobotCommandStore` /
  `RobotStatusStore` / `RobotSensorStore` (`scripts/control_panel.py:27`,
  `tennis_robot.control_bus`). Το ρομπότ **δεν εξαρτάται** από το αν τρέχει το UI:
  αν πέσει το dashboard, ο control loop συνεχίζει. Αυτό το decoupling είναι το πιο
  σημαντικό σημείο και είναι ήδη πετυχημένο.
- **Command store ως σύνορο εποπτικού ελέγχου.** Οι εντολές του UI (start/stop
  survey κ.λπ.) γράφονται στο command store και το Pi είναι η αρχή που τις
  εκτελεί ή τις απορρίπτει (`CourtSurveyLaunchManager`, `control_panel.py:40`).
- **Telemetry-friendly snapshots.** Το survey γράφει throttled, **sampled**
  telemetry: `runtime/court_survey_live.json` στα **5 Hz**, με **έως 1500 σημεία**
  (`MAP_SAMPLE_MAX`) χάρτη — όχι raw scans. Στέλνουμε **παράγωγα**, όχι ωμά
  δεδομένα. Αυτό είναι ακριβώς το σωστό για IoT bandwidth.
- **Web server που διαβάζει snapshots.** `ThreadingHTTPServer` που σερβίρει το
  `scripts/control_panel/` UI και διαβάζει τα stores (`control_panel.py:18`).

Δηλαδή το «τι» υπάρχει· αυτό που μένει είναι το **«πώς το βγάζουμε έξω από το Pi»**
(transport) και το **containerization**.

---

## 3. Τοπολογία-στόχος

```
            ┌──────────────────────── Raspberry Pi 5 (EDGE) ────────────────────────┐
            │                                                                        │
            │   ROS 2 graph (control plane — real-time, μένει ΕΔΩ)                   │
            │   ┌───────────────┐  ┌──────────────┐  ┌───────────────────────────┐  │
            │   │ slam_toolbox  │  │ survey/coll. │  │ perception_node           │  │
            │   │ (map→base_tf) │  │ FSM + motion │  │ (classical CV, OAK-D depth)│  │
            │   └──────┬────────┘  └──────┬───────┘  └────────────┬──────────────┘  │
            │          │                  │                        │                 │
            │          ▼                  ▼                        ▼                 │
            │     ┌──────────────────────────────────────────────────────┐         │
            │     │  File-based IPC (πηγή αλήθειας)                        │         │
            │     │  command / status / sensor stores                     │         │
            │     │  runtime/*.json  (5 Hz, sampled, παράγωγα)            │         │
            │     └───────────────┬───────────────────────┬──────────────┘         │
            │                     │ publish (push)         │ supervisory cmd (pull)  │
            │            ┌────────▼─────────┐              │                         │
            │            │ telemetry bridge │◄─────────────┘                         │
            │            │ (MQTT / WS / FG) │   commands ΠΟΤΕ real-time loop         │
            │            └────────┬─────────┘                                        │
            └─────────────────────┼──────────────────────────────────────────────────┘
                                  │  LAN ή internet (συμπιεσμένο, throttled)
                                  ▼
            ┌──────────────────────────────────────────────────────────────────────┐
            │  Containerized dashboard (OBSERVATION plane — οπουδήποτε)              │
            │  Docker:  web UI  +  (προαιρετικά) MQTT broker  +  time-series store   │
            │  • sensor views (JPEG/H.264 preview, χαμηλό fps)                       │
            │  • map / occupancy points, status, timing, alerts                      │
            │  • supervisory controls: start/stop, go-to-vantage (idempotent)        │
            └──────────────────────────────────────────────────────────────────────┘
```

---

## 4. Μη-διαπραγματεύσιμοι κανόνες

1. **Ο control loop μένει 100% στο Pi.** Καμία real-time απόφαση κίνησης/ασφάλειας
   δεν εξαρτάται από το δίκτυο ή τον browser.
2. **Web → robot μόνο supervisory & idempotent.** Επιτρεπτά: `start_survey`,
   `stop`, `go_to_vantage(i)`, `return_home`. Απαγορευμένα: συνεχές teleop /
   κλείσιμο βρόχου πάνω από WAN. Η εντολή περνά από το command store· **το Pi είναι
   η αρχή που μπορεί να την απορρίψει**.
3. **Fail-safe σε απώλεια δικτύου/dashboard.** Αν χαθεί το link ή πέσει το
   container, το ρομπότ συνεχίζει ή σταματά με ασφάλεια — ποτέ undefined behavior.
   (Το file-based IPC ήδη το εγγυάται: ο robot δεν περιμένει το UI.)
4. **Telemetry = push, throttled, παράγωγα.** Στέλνουμε επεξεργασμένα/δειγματισμένα
   δεδομένα (map points, status), όχι ωμά scans/frames. Διατηρούμε το throttle των
   5 Hz ως αφετηρία.
5. **Ποτέ raw frames/depth πάνω από δίκτυο.** Δες §6.
6. **Supervisory εντολές με ack.** Κάθε εντολή έχει id· το Pi γράφει αποτέλεσμα
   (accepted/rejected/done) στο status, το UI το επιβεβαιώνει.

---

## 5. Επιλογές transport

Τρεις καθαρές διαδρομές για να βγει το telemetry από το Pi. Δεν αποκλείονται
μεταξύ τους — συχνά συνδυάζονται (π.χ. MQTT για scalars + Foxglove για sensor views).

### 5α. Εξέλιξε το υπάρχον (χαμηλότερο effort)
Το containerized web app διαβάζει τα ίδια `runtime/*.json` και τα σπρώχνει στον
browser με **WebSocket ή SSE** αντί για HTTP polling.
- ✅ Κρατάς όλο τον υπάρχοντα κώδικα· απλώς Docker + push αντί poll.
- ✅ Ιδανικό για status, timing, map/occupancy points.
- ⚠️ Δικός σου ο μηχανισμός για image preview (δες §6).

### 5β. MQTT — η «κανονική» IoT λογική
Pi = publisher σε broker (Mosquitto, containerized)· dashboard = subscriber.
- ✅ Πολλαπλοί clients, εύκολο remote/internet access, retained messages, QoS.
- ✅ Ελαφρύ για scalar/JSON telemetry· φυσικό για alerts.
- ✅ Επεκτείνεται πέρα από ROS (π.χ. Home Assistant, Grafana, cloud).
- ⚠️ Λιγότερο κατάλληλο για image streams — βάλε τα frames σε ξεχωριστό
  μονοπάτι (δες §6), όχι σε MQTT payload.

### 5γ. Foxglove / rosbridge — αφού είμαστε ήδη ROS 2
`foxglove_bridge` στο Pi → Foxglove (web/desktop) δίνει **σχεδόν δωρεάν** sensor
views: `/camera`, `/camera/depth`, `/scan`, TF, map, occupancy grid.
- ✅ Μηδέν custom UI κώδικας· φτιαγμένο για robot telemetry.
- ✅ Άριστο για debugging «τι βλέπει το ρομπότ» σε πραγματικό χρόνο.
- ⚠️ Λιγότερος έλεγχος στο «προϊοντικό» look· πιο εργαλείο-μηχανικού.
- 💡 Εναλλακτικά `rosbridge_suite` (WebSocket προς ROS topics) για δικό σου UI.

### Σύγκριση
| Κριτήριο | 5α WS/SSE | 5β MQTT | 5γ Foxglove |
| --- | --- | --- | --- |
| Effort | Χαμηλό | Μεσαίο | Πολύ χαμηλό (για sensor views) |
| Scalars/status | ✅ | ✅✅ | ✅ |
| Sensor views | χειροκίνητα | χειροκίνητα | ✅✅ |
| Remote/internet | μέτριο | ✅✅ | μέτριο |
| Custom «προϊόν» UI | ✅✅ | ✅ | ⚠️ |
| Multi-client | μέτριο | ✅✅ | ✅ |

---

## 6. Sensor views: το πραγματικό πρόβλημα είναι το bandwidth

Τα camera/depth frames είναι βαριά. Κανόνες:

- **Ποτέ raw frames ή raw depth arrays.** Το `/camera/depth` είναι `32FC1`
  (`perception_node.py:94`) — τεράστιο. Μην το στέλνεις ωμό.
- **RGB → JPEG** για preview· **H.264** αν θες πραγματικό stream.
- **Depth → colormap preview** (8-bit, downsampled), όχι ο πίνακας float.
- **Map/occupancy → παράγωγα σημεία** (όπως ήδη τα 1500 σημεία του survey), όχι raw scans.
- **Throttle** το image preview σε χαμηλό fps (π.χ. 2–5 fps για επόπτευση· το
  ανθρώπινο μάτι δεν χρειάζεται 30 fps για dashboard).
- **Ξεχωριστό μονοπάτι για βαριά δεδομένα.** Scalars/status από το ένα κανάλι
  (WS/MQTT)· image preview από δεύτερο (MJPEG endpoint ή compressed topic). Έτσι
  ένα αργό frame δεν μπλοκάρει το status.

> Σημείωση compute: η συμπίεση (JPEG/H.264) προσθέτει φόρτο στη CPU του Pi. Σε
> χαμηλό fps είναι αμελητέα· αν ανέβει το fps, προτίμησε hardware encoder ή
> κράτα το preview μικρό (π.χ. 480p).

---

## 7. Πού τρέχει το container

- **LAN-only (γήπεδο, laptop στο πλάι):** το container μπορεί να τρέξει και στο
  ίδιο το Pi (το dashboard server είναι ελαφρύ) ή στο laptop. Απλό, χαμηλό latency.
- **Remote / internet:** βάλε broker + dashboard σε μικρό server ή cloud· το Pi
  κάνει μόνο **outbound publish**. Εδώ λάμπει το MQTT (retained state, reconnect,
  QoS). Πρόσεξε auth/TLS αν βγαίνει στο internet.
- Ταιριάζει με το υπάρχον workflow: το repo ήδη χρησιμοποιεί `docker compose`
  εκτενώς, οπότε ένα `dashboard` service είναι φυσική προσθήκη.

---

## 8. Migration path από το σημερινό `control_panel.py`

Σταδιακά, χωρίς να σπάσει τίποτα — κάθε βήμα είναι ανεξάρτητα χρήσιμο:

1. **Containerize ως έχει.** Βάλε το `control_panel.py` + `control_panel/` σε
   Docker service (`dashboard`) στο `docker-compose`. Mount το `runtime/` read-only.
   Αποτέλεσμα: ίδια λειτουργία, αλλά portable/απομονωμένο.
2. **Polling → push.** Αντικατέστησε το HTTP polling με **SSE ή WebSocket** για
   status/map. Λιγότερο latency, λιγότερος φόρτος. (Κράτα το REST για supervisory
   commands.)
3. **Image preview path.** Πρόσθεσε ξεχωριστό **MJPEG/compressed preview** endpoint
   για το «τι βλέπει η κάμερα» σε 2–5 fps, αποσυνδεδεμένο από το status κανάλι.
4. **(Προαιρετικά) MQTT για remote.** Πρόσθεσε publisher στο Pi που σπρώχνει τα
   ίδια snapshots σε Mosquitto broker· το dashboard γίνεται subscriber. Ξεκλειδώνει
   internet access + multi-client + alerts.
5. **(Προαιρετικά) Foxglove για deep debugging.** `foxglove_bridge` στο Pi όταν
   θες live sensor views χωρίς να γράψεις UI.

Σε κάθε βήμα: **το command store παραμένει το σύνορο** και ο control loop στο Pi
δεν αγγίζεται.

---

## 9. Σχετικά αρχεία

- `scripts/control_panel.py` — σημερινός web console (HTTP + file stores).
- `scripts/control_panel/app.js` — UI client.
- `tennis_robot.control_bus` — `RobotCommandStore` / `RobotStatusStore` / `RobotSensorStore`.
- `ros2_ws/src/tennis_robot/tennis_robot/court_survey_v2_node.py` — γράφει
  `runtime/court_survey_live.json` (5 Hz, sampled telemetry).
- `ros2_ws/src/tennis_robot/tennis_robot/perception_node.py` — classical CV + OAK-D depth.
- `runtime/robot_command.json`, `runtime/robot_status.json` — IPC αρχεία.

---

## 10. TL;DR

Η αρχιτεκτονική που περιγράφεις είναι σωστή και είσαι ήδη στο 80%. Κράτα τον
έλεγχο στο Pi, κράτα το file-based IPC ως πηγή αλήθειας (το decoupling είναι
χρυσός), και βγάλε το telemetry έξω σε containerized dashboard με push transport.
Για scalars/map → WS/SSE ή MQTT· για sensor views → συμπιεσμένο preview σε χαμηλό
fps (ή Foxglove για debug). Το web app **μόνο παρατηρεί + στέλνει supervisory
εντολές**, ποτέ real-time loop πάνω από δίκτυο.
