#!/usr/bin/env bash
# Render one top-view video per species. Run from the folder with the CSVs.
set -e
for s in Anguilla_rostrata Acipenser_brevirostrum Salvelinus_fontinalis \
         Scomber_scombrus Lutjanus_griseus Thunnus_albacares; do
    echo "=== $s ==="
    python -m simulation.run_sim --csv "Midline_*.csv" --species "$s" \
        --arch cascade --video --duration 4 --quiet-clip
done
