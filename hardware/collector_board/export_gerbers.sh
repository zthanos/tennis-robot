#!/bin/bash
# Run from the collector_board/ directory
# Requires KiCad 7+ installed
kicad-cli pcb export gerbers \
  --output gerber/ \
  --layers F.Cu,B.Cu,F.SilkS,F.Mask,B.Mask,Edge.Cuts \
  collector_board.kicad_pcb

kicad-cli pcb export drill \
  --output gerber/ \
  --format excellon \
  --drill-origin absolute \
  collector_board.kicad_pcb

echo "Gerbers saved to gerber/"
