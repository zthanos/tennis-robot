# Body concept F — current status

Το F είναι το μοναδικό ενεργό body concept. Τα exact πίσω panels είναι τα
P2-86b του `body-panels-f-rear-v2.scad` και παραμένουν release-candidate,
**όχι ακόμη εγκεκριμένα για εκτύπωση**. Τα πλαϊνά/εμπρός panels του
`body-panels-f.scad` παραμένουν concept σύμφωνα με τον χάρτη P2-86b μέχρι το
basket mockup και το κλείδωμα των interfaces.

Το `current-design-p293.scad` είναι μόνο render composition της τρέχουσας
γεωμετρίας P2-93. Τα PNG στο `renders-current-p293/` είναι τα σημερινά
documentation renders· δεν αποτελούν STL release.

Το `renders-f/` περιέχει **αρχικό concept, πριν το P2-84**. Διατηρείται για
ιστορική σύγκριση και δεν απεικονίζει την τρέχουσα P2-93 αψίδα, το πραγματικό
rear-pod v2/P2-90d ή τα exact P2-86b rear panels.

```bash
OPENSCAD_BIN=/snap/bin/openscad-nightly bash cad/body-panels/render-current-p293.sh
```
