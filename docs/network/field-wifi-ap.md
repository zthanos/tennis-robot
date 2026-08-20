# Field Wi-Fi access point (Phase 1)

Το Raspberry Pi μπορεί να δημιουργεί αυτόνομο, προστατευμένο Wi-Fi για το
operator console στο γήπεδο. Δεν απαιτούνται router ή Internet. Το Phase 1 είναι
σκόπιμα **dedicated AP mode**· δεν κάνει ακόμη αυτόματη εναλλαγή home Wi-Fi/AP.

## Αρχιτεκτονική και προεπιλογές

- Ubuntu 24.04 NetworkManager, χωρίς `hostapd` ή ξεχωριστό `dnsmasq` service. Το
  Ubuntu `dnsmasq-base` executable χρησιμοποιείται εσωτερικά από το
  NetworkManager για DHCP/DNS σε `ipv4.method=shared` και εγκαθίσταται από το
  provisioning μόνο αν λείπει.
- Profile: `tennis-robot-field-ap`, SSID: `TennisRobot`.
- Pi/router: `10.42.0.1/24`. Το `ipv4.method=shared` του NetworkManager παρέχει
  DHCP και local DNS στους clients.
- 2.4 GHz channel 6 από προεπιλογή για προβλέψιμη συμβατότητα κινητών.
- WPA2-RSN (`wpa-psk`) από προεπιλογή. Προαιρετικά WPA3-SAE (`sae`) αν το
  τηλέφωνο το υποστηρίζει.
- Στο WPA2 profile χρησιμοποιείται AES/CCMP και το PMF απενεργοποιείται ρητά για
  συμβατότητα με το Broadcom `brcmfmac` AP firmware του Pi. Το WPA2 passphrase
  παραμένει υποχρεωτικό.
- Client isolation είναι απενεργοποιημένο: το Broadcom `brcmfmac` firmware του
  Pi απομόνωνε λανθασμένα clients και από το AP/router, με αποτέλεσμα ατέρμονα
  ARP requests και μη προσβάσιμο web console. Το AP προορίζεται για το έμπιστο
  operator device και προστατεύεται με WPA2/AES.
- Το profile είναι persistent, `autoconnect=yes`, και ξεκινά από το
  NetworkManager στο boot. Έχει τη μέγιστη autoconnect priority ώστε στο
  dedicated Phase 1 field mode να κερδίζει άλλα αδρανή Wi-Fi profiles. Δεν
  υπάρχει service που να εξαρτά τη λειτουργία του robot από την παρουσία
  τηλεφώνου.
- Το operator HTTP/API console ακούει σκόπιμα στο `0.0.0.0:8081` από το κύριο
  ROS launch. Το ROS DDS δεν εκτίθεται ως phone API. Το προαιρετικό Foxglove
  WebSocket (`8765`) παραμένει debug service και δεν ξεκινά αυτόματα στο Pi.

## Provisioning στο Pi

Δημιούργησε μοναδικό password εκτός Git:

```bash
sudo install -d -m 0755 /etc/tennis-robot
sudo install -m 0600 config/network/field-wifi.env.example \
  /etc/tennis-robot/field-wifi.env
sudoedit /etc/tennis-robot/field-wifi.env
```

Το `FIELD_WIFI_PASSPHRASE` πρέπει να είναι μοναδικό (8–63 χαρακτήρες). Μετά:

```bash
INSTALL_FIELD_WIFI=true ./scripts/setup_pi.sh
```

Για να ξεκινά και το υπάρχον operator console αυτόματα στο boot:

```bash
INSTALL_FIELD_WIFI=true INSTALL_PI_CONSOLE_SERVICE=true ./scripts/setup_pi.sh
```

Το `tennis-robot-console.service` τρέχει ως ο non-root χρήστης που εκτέλεσε το
provisioning, κάνει bind στο `0.0.0.0:8081` και επανεκκινείται μόνο σε failure.

Για networking-only εγκατάσταση, χωρίς ROS build:

```bash
sudo ./scripts/network/install_field_wifi_ap.sh \
  --config /etc/tennis-robot/field-wifi.env
```

Το script είναι idempotent: ενημερώνει το ονομασμένο profile, δεν δημιουργεί
duplicates και δεν αλλάζει Ethernet ή άλλα profiles. Αν το τρέξεις μέσω SSH που
περνά από το ίδιο Wi-Fi interface, δεν κόβει τη σύνδεση· αφήνει την ενεργοποίηση
για το επόμενο reboot. Με Ethernet SSH μπορεί να ενεργοποιηθεί αμέσως.

Οι διαθέσιμες ρυθμίσεις φαίνονται στο
`config/network/field-wifi.env.example`. Άφησε κενό το interface για ασφαλές
auto-detection. Χρησιμοποίησε `FIELD_WIFI_BAND="a"` μόνο αφού οριστεί σωστά το
regulatory country και έχει επιβεβαιωθεί 5 GHz phone compatibility.

## Χρήση στο γήπεδο

1. Άναψε/reboot το Pi και περίμενε να ξεκινήσει το robot stack.
2. Στο τηλέφωνο επίλεξε `TennisRobot` και βάλε το provisioned password. Η ένδειξη
   «χωρίς Internet» είναι αναμενόμενη· επίλεξε να παραμείνει συνδεδεμένο.
3. Άνοιξε `http://10.42.0.1:8081`.
4. Telemetry/diagnostics/control λειτουργούν από το ίδιο HTTP API. Αν έχει
   ξεκινήσει ξεχωριστά το προαιρετικό Foxglove bridge, σύνδεσέ το στο
   `ws://10.42.0.1:8765`.

Το κλείσιμο browser ή η αποσύνδεση τηλεφώνου αφαιρεί μόνο το operator interface.
Οι autonomous ROS processes και το file-backed command/status bus δεν έχουν
network-client liveness dependency. Οι υπάρχουσες pause/stop semantics δεν
αλλάζουν και το Wi-Fi **δεν είναι emergency stop**.

## Status, recovery και validation

```bash
./scripts/network/field_wifi_status.sh
sudo nmcli connection up tennis-robot-field-ap
sudo nmcli connection down tennis-robot-field-ap
sudo nmcli connection up tennis-robot-field-ap
./scripts/network/validate_field_wifi.sh
nmcli device status
nmcli connection show --active
ip -4 addr
ip route
ss -ltnp | grep -E ':(8081|8765)'
curl --fail http://10.42.0.1:8081/api/status
journalctl -u NetworkManager -b --no-pager
systemctl status tennis-robot-console.service --no-pager
journalctl -u tennis-robot-console.service -b --no-pager
```

Physical acceptance μετά από reboot:

1. Επιβεβαίωσε ότι εμφανίζεται το SSID και ότι το phone/laptop παίρνει DHCP IP.
2. Κάνε ping το `10.42.0.1`, άνοιξε console, δες live telemetry και δοκίμασε μόνο
   ασφαλή/επιτρεπτή control command.
3. Κλείσε mobile data/Internet και επανάλαβε.
4. Αποσύνδεσε το phone και επιβεβαίωσε ότι οι autonomous processes συνεχίζουν
   (`systemctl`/ROS process status και ασφαλής παρατήρηση του robot).
5. Επανασύνδεσε και επιβεβαίωσε ότι monitoring/control ανακάμπτουν.

## Security και περιορισμοί

Το passphrase δεν αποθηκεύεται στο Git: βρίσκεται στο root-only config και στο
root-owned NetworkManager profile. Άλλαξέ το αν κοινοποιηθεί. Το console σήμερα
δεν προσθέτει δικό του login ή TLS, άρα όποιος γνωρίζει το Wi-Fi password έχει
operator-network access· χρησιμοποίησε ισχυρό password και μη μοιράζεσαι το AP.
Firewall rules δεν αλλάζουν αυτόματα, ώστε να μη διαταραχθεί υπάρχουσα πολιτική.
Αν είναι ενεργό UFW, επίτρεψε μόνο τα operator ports στο AP interface σύμφωνα με
την τοπική πολιτική.

Το Phase 2 μπορεί αργότερα να προσθέσει client/AP policy πάνω σε διαφορετικά
NetworkManager profiles. Το Phase 1 δεν προσθέτει fragile fallback daemon και
δεν εμποδίζει αυτή την εξέλιξη.
