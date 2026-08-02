# Σταθεροποίηση δικτύου PC–Pi

## Scope freeze

Η εργασία δικτύου προηγείται κάθε νέου collection test. Μέχρι να περάσει το
network gate δεν αλλάζουν:

- ο ρυθμός, τα δείγματα ή το περιεχόμενο του `/scan`,
- perception, timestamps ή TF semantics,
- planner, route executor, follower ή intake,
- η πληροφορία που παρουσιάζει το τελικό UI.

Αλλάζει μόνο η παρατηρησιμότητα και, σε επόμενα checkpoints, ο τρόπος μεταφοράς
της ίδιας πληροφορίας.

## Read-only probe

Το `scripts/network/network_probe.py` διαβάζει counters από
`/sys/class/net/<interface>/statistics` και `/proc/net/snmp`. Χωρίς
`--http-url` δεν ανοίγει socket και δεν προκαλεί network traffic.

Παράδειγμα offline smoke:

```bash
python3 scripts/network/network_probe.py \
  --label local-offline-smoke \
  --interface lo \
  --duration-s 2 \
  --interval-s 0.5 \
  --output /tmp/tennis-network-smoke.json
```

Σε πραγματικό qualification window, το probe τρέχει ταυτόχρονα σε PC και Pi
πάνω στο interface που συνδέεται στο satellite. Το `--capture-ros-graph` είναι
προαιρετικό και χρησιμοποιείται μόνο με ενεργό ROS stack. Καταγράφει metadata
του graph αλλά δεν κάνει subscribe σε payload topics:

```bash
ROS_DOMAIN_ID=42 python3 scripts/network/network_probe.py \
  --label D-full-stack-ui-closed-pc \
  --interface eth0 \
  --duration-s 120 \
  --capture-ros-graph \
  --ros-topic /scan \
  --ros-topic /tf \
  --ros-topic /telemetry/sensor_snapshot \
  --output runtime/network/D-full-stack-ui-closed-pc.json
```

Για μέτρηση HTTP payload προστίθεται ρητά το endpoint. Αυτό προσομοιώνει τον
browser και άρα δεν χρησιμοποιείται στα UI-closed scenarios:

```bash
python3 scripts/network/network_probe.py \
  --label E-dashboard-pc \
  --interface eth0 \
  --duration-s 120 \
  --http-url http://tennisserver.local:8081/api/diagnostics \
  --output runtime/network/E-dashboard-pc.json
```

## Scenario matrix

Κάθε σενάριο διαρκεί τουλάχιστον 120 s και επαναλαμβάνεται τρεις φορές:

| ID | PC | Pi | Browser |
| --- | --- | --- | --- |
| A | χωρίς ROS | κλειστό | κλειστό |
| B | χωρίς ROS | ανοικτό, χωρίς stack | κλειστό |
| C | simulation μόνο | χωρίς brain | κλειστό |
| D | simulation | brain stack | κλειστό |
| E | simulation | brain stack | Dashboard |
| F | simulation | brain stack | Survey/Collection |
| G | πλήρες mission | brain stack | ενεργό live preview |

Για κάθε run κρατάμε:

- PC και Pi probe artifacts,
- internet speed πριν/κατά/μετά, χωρίς να αυτοματοποιηθεί από το probe,
- ping PC↔Pi και προς gateway,
- αν το satellite έχασε internet,
- ακριβές start/stop time και run/commit id.

Το speed test παραμένει χειροκίνητο ώστε το instrumentation να μην καταναλώνει
internet ή να ξεκινήσει κατά λάθος σε ώρες που το δίκτυο χρειάζεται να παραμένει
διαθέσιμο.

## Comparison

```bash
python3 scripts/network/network_probe_report.py \
  --baseline-label A-baseline-pc \
  --input runtime/network/A-baseline-pc.json \
  --input runtime/network/D-full-stack-ui-closed-pc.json \
  --input runtime/network/E-dashboard-pc.json \
  --output-json runtime/network/comparison.json \
  --output-markdown runtime/network/comparison.md
```

Το report συγκρίνει bytes/s, packets/s, multicast counters, interface drops,
UDP errors και HTTP payload/latency. Δεν αποδίδει αιτία χωρίς packet capture·
η ερμηνεία γίνεται μετά τη scenario matrix.

## Packet classification

Αν τα counters δείξουν packet-rate ή multicast spike, γίνεται σύντομο capture
μόνο σε προγραμματισμένο παράθυρο:

```bash
sudo timeout 60 tcpdump -ni eth0 -w /tmp/tennis-dds.pcap udp
```

Το pcap μπορεί να περιέχει διευθύνσεις/ονόματα του LAN, δεν γίνεται commit και
παραμένει εκτός repository.

## Network gate

Πριν επιστρέψουμε στο collection:

- internet throughput με πλήρες stack ≥ 90% του median baseline,
- καμία απώλεια internet από το satellite,
- μηδενικά sustained interface drops και UDP buffer errors,
- κανένα `TF_OLD_DATA` storm,
- `/scan` rate και message contents αμετάβλητα,
- TF lookup success ≥ 99.9% σε qualification run,
- 30 min soak, ένα Survey και δύο διαδοχικά Collection χωρίς restart,
- UI πλήρες, με preview traffic μόνο όταν υπάρχει ενεργό σχετικό view.

## Checkpoints

1. Instrumentation και scenario contract — το παρόν checkpoint.
2. Demand-driven preview και versioned/delta HTTP payloads.
3. Unicast DDS isolation ή δύο local domains με explicit allowlist, ανάλογα με
   τα μετρημένα multicast/fan-out αποτελέσματα.
4. Topic-class QoS και restart/recovery qualification.
5. Τελικό network report και άρση του collection freeze.

## Isolated-domain qualification profile

Οι μετρήσεις των scenarios C/D εντόπισαν δύο ανεξάρτητους πολλαπλασιαστές:

- PC Ethernet και Wi-Fi στο ίδιο subnet έκαναν το Fast DDS να χρησιμοποιεί και
  τα δύο interfaces. Με μόνο Ethernet εξαφανίστηκαν multicast/UDP errors στο C.
- Με κοινό domain, κάθε Pi subscriber ήταν ξεχωριστός remote DDS reader. Στο D
  αυτό έφτασε περίπου 2.400 packets/s στο PC και δημιούργησε UDP receive-buffer
  errors στο Pi, παρότι το συνολικό throughput ήταν μόνο περίπου 6 Mbit/s.

Το δοκιμαστικό profile κρατά το PC στο domain 42 και το Pi brain στο domain 43.
Ένα `domain_bridge` στο Pi είναι το μοναδικό endpoint και στα δύο domains και
μεταφέρει μόνο το allowlist
`config/network/pc42_pi43_domain_bridge.yaml`. Raw RGB/depth, Gazebo contacts
και debug markers μένουν στο PC. Το `/scan` δεν αλλάζει rate, περιεχόμενο ή
τύπο· απλώς έχει έναν remote reader πριν γίνει local fan-out στο Pi.

Τα allowlisted endpoints δημιουργούνται σταθερά κατά την εκκίνηση. Το
πειραματικό dynamic `wait/auto-remove` profile γεφύρωσε μόνο το `/clock` στο
Jazzy 0.5.0 και άφησε `/scan`, odometry και TF χωρίς publisher στο domain 43,
οπότε απορρίφθηκε πριν από το D2 measurement.

PC, με Wi-Fi αποσυνδεδεμένο κατά το qualification:

```bash
TENNIS_LAUNCH_BRAIN=false ROS_DOMAIN_ID=42 ./run_native.sh
```

Pi:

```bash
cd ~/tennis-robot
./scripts/network/run_pi_isolated.sh 2>&1 | tee ~/run_pi.log
```

Το legacy κοινό-domain launch παραμένει διαθέσιμο για rollback. Το isolated
profile δεν γίνεται default πριν περάσει το D2 gate: μηδενικά UDP errors/drops,
σταθερό gateway RTT και αισθητή μείωση packet rate σε σχέση με το D.

## Qualification results — 2026-08-01

Το isolated profile πέρασε το UI-closed D2 gate για 120 s χωρίς UDP ή receive
buffer errors και χωρίς packet loss. Η διάμεση κίνηση του PC έπεσε από περίπου
710 KiB/s και 2.395 packets/s TX στο κοινό domain, σε 235 KiB/s και 511
packets/s TX. Το `/scan` παρέμεινε περίπου στα 10.7 Hz και το περιεχόμενό του
δεν τροποποιήθηκε.

Η αρχική προσομοίωση Collection UI με τα πλήρη diagnostics απέτυχε το network
gate λόγω μεγάλου HTTP payload (περίπου 212 KiB diagnostics ανά refresh). Η
gzip συμπίεση μείωσε το payload, αλλά μόνη της δεν αρκούσε: το gateway RTT
παρέμεινε πάνω από 100 ms και εμφανίστηκαν receive-buffer errors στο PC.

Η τελική λύση χρησιμοποιεί και gzip και view-scoped diagnostics. Το Collection
view δεν μεταφέρει survey-only `robot_path`, `map_points` και
`navigation_points`, ενώ το Survey view κρατά τα πλήρη 1.500 LiDAR map points
και χρησιμοποιεί μόνο deterministic display sample 200 σημείων για το path.
Το πλήρες path των 2.000 σημείων παραμένει διαθέσιμο από το audit endpoint
`/api/path`.

Τα δύο 120 s UI gates έδωσαν:

| Gate | HTTP | Payload | Gateway RTT PC / Pi | UDP / Rcvbuf errors | Loss |
| --- | ---: | ---: | ---: | ---: | ---: |
| Collection view | 230/230 | 2.14 MB | 4.53 / 4.18 ms | 0 / 0 | 0% |
| Survey view | 228/228 | 4.00 MB | 8.94 / 5.29 ms | 0 / 0 | 0% |

Στο Survey gate το HTTP p95 ήταν 71.8 ms και το μέγιστο gateway RTT ήταν
70.7 ms στο PC και 22.7 ms στο Pi, χωρίς sustained spike ή packet loss. Τα
interface drops (PC 7, Pi 15) ήταν στο επίπεδο του idle baseline και δεν
αυξήθηκαν ως sustained rate. Τα static network/UI gates επομένως περνούν χωρίς
μείωση LiDAR. Παραμένουν ανοιχτά τα δυναμικά qualification gates: ένα Survey,
δύο διαδοχικά Collection χωρίς restart και το 30 min soak.

### Dynamic qualification findings

Το πρώτο monitored Survey με Ubuntu default UDP receive buffers (212 KiB)
ολοκλήρωσε τη διαδρομή μόνο μερικώς πριν διακοπεί από το safety gate. Παρότι το
gateway είχε 0% loss και περίπου 4–5 ms RTT, μετρήθηκαν 20.141 Pi και 270 PC
`RcvbufErrors`. Το `ss -uapm` έδειξε υπερχείλιση σε DDS participant sockets και
όχι κορεσμό του Ethernet.

Με προσωρινό `net.core.rmem_default=4194304` στο Pi, το επαναληπτικό Survey
ολοκληρώθηκε `9/9`, έκανε save και τερμάτισε `done / OK` σε 134.6 s simulation
time. Τα νέα runtime sockets είχαν μηδενικά drops. Τα υπόλοιπα 1.106 global Pi
errors ανήκαν αποκλειστικά σε δύο παλιούς `ros2-daemon` participants των domains
42/43 που είχαν παραμείνει με 212 KiB buffers. Ο isolated launcher πλέον θέτει
`ROS2CLI_DISABLE_DAEMON=1`, και οι daemons αφαιρέθηκαν από το qualification
graph.

Στο επόμενο 480 s window, που περιείχε δύο διαδοχικά Collection χωρίς restart,
το Pi πέρασε με 0 UDP/Rcvbuf errors, 0% gateway loss και 5.28 ms average RTT.
Το PC είχε επίσης 0% loss και 5.50 ms RTT, αλλά 898 local buffer drops σε Gazebo
discovery και sim-side ROS sockets που εξακολουθούσαν να χρησιμοποιούν το PC
default των 212 KiB. Γι' αυτό το ίδιο 4 MiB profile είναι υποχρεωτικό και στις
δύο μηχανές μέσω
`config/network/99-tennis-robot-udp-buffers.conf`. Τα distributed launchers
κάνουν fail-loud αν λείπει, αντί να επιτρέπουν υποβαθμισμένο qualification.

Τα Collection αποτελέσματα καταγράφονται χωριστά από το network gate:

- run 1: ξεκίνησε, μάζεψε/επιβεβαίωσε 1 στόχο, έκανε δεύτερο scan και τερμάτισε
  σωστά ως `incomplete_targets` με 3 unresolved targets,
- run 2: ξεκίνησε χωρίς restart, αλλά τερμάτισε `aborted_tracking/path_failed`.
  Το backend log κατέγραψε `collection execution context load rejected:
  context_already_consumed`, που είναι πλέον συγκεκριμένο collection lifecycle
  defect για το επόμενο fix και όχι network/startup failure.

Ανοιχτά gates: εγκατάσταση του sysctl profile στο PC, επανάληψη του 480 s
dynamic window με μηδενικά errors και κατόπιν 30 min soak. Το profile έχει ήδη
εγκατασταθεί μόνιμα στο Pi.

### Symmetric isolation qualification — 2026-08-02

Η εγκατάσταση του 4 MiB profile και στο PC μηδένισε τα socket errors, αλλά ένα
full-stack Survey εξακολουθούσε να επηρεάζει τον mesh satellite. Η απομόνωση
έδειξε δύο ακόμη πηγές local traffic που δεν έπρεπε να βρίσκονται στο LAN:

- το Gazebo Transport χρησιμοποιούσε το Ethernet για επικοινωνία μεταξύ
  διεργασιών που εκτελούνται όλες στο ίδιο PC,
- όλοι οι PC ROS participants συμμετείχαν απευθείας στο LAN domain 42, οπότε το
  PC-side discovery/fan-out παρέμενε ενεργό παρά την απομόνωση του Pi.

Το τελικό topology είναι συμμετρικό:

```text
PC Gazebo/ROS domain 41
        |
PC allowlist domain_bridge
        |
LAN domain 42 (μόνο δύο bridge endpoints)
        |
Pi allowlist domain_bridge
        |
Pi brain/Nav2/SLAM domain 43
```

Το Gazebo Transport περιορίζεται ταυτόχρονα στο loopback με
`GZ_IP=127.0.0.1`. Το PC ξεκινά πλέον από
`scripts/network/run_pc_isolated.sh`, ενώ το Pi συνεχίζει με
`scripts/network/run_pi_isolated.sh`. Και τα δύο wrappers απενεργοποιούν τον
ROS CLI daemon και απαιτούν το 4 MiB receive-buffer profile.

Το μεγάλο gateway RTT που μετριόταν μέσα από το φορτωμένο PC/Pi δεν ήταν
αξιόπιστος δείκτης mesh latency: με PC simulation ενεργό, το PC έδειξε 80 ms
average ενώ το idle Pi, την ίδια στιγμή προς το ίδιο gateway, έδειξε 3.52 ms.
Πρόκειται για host scheduling jitter. Η εξωτερική μέτρηση από πραγματικό
browser με όλο το stack και UI ενεργά έδωσε 210–290 Mbit/s, έναντι της παλιάς
κατάρρευσης περίπου στα 20 Mbit/s.

Τα τελικά qualification runs έδωσαν:

| Run | Mission result | PC UDP/Rcvbuf | Pi UDP/Rcvbuf | UI requests | HTTP p95 |
| --- | --- | ---: | ---: | ---: | ---: |
| Q2 Survey, 480 s | `9/9`, `done / OK` | 0 / 0 | 0 / 0 | 890/890 | 96.2 ms |
| Q3 2× Collection, 480 s | 1 retained, δεύτερο run ξεκίνησε | 0 / 0 | 0 / 0 | 914/914 | 58.6 ms |

Το network freeze για λειτουργικές δοκιμές μπορεί να αρθεί: `/scan` παρέμεινε
10–11 Hz, TF και UI ήταν φρέσκα, και τα δύο Collection ξεκίνησαν χωρίς restart.
Παραμένει ως τελικό infrastructure gate μόνο το 30 min soak.

Το επόμενο blocker ανήκει αποκλειστικά στο collection lifecycle. Το πρώτο run
τερμάτισε `incomplete_targets` αφού κράτησε 1 μπάλα και άφησε 2 unresolved. Το
δεύτερο run ξεκίνησε χωρίς restart, αλλά το backend απέρριψε το παλιό execution
context ως `context_already_consumed` και τερμάτισε
`aborted_tracking/path_failed`. Δεν πρέπει να αντιμετωπιστεί με network ή QoS
αλλαγές.
