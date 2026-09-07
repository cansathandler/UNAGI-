# Render one top-view video per species.
# Run from the folder that holds the Midline_*.csv files.
#
#   powershell -ExecutionPolicy Bypass -File .\make_all_videos.ps1
#
# Each run is ~360 rendered frames at 1920x1200, so budget a few minutes
# per species.  Drop --video to get the tracking numbers only (seconds).

$species = @(
    "Anguilla_rostrata",
    "Acipenser_brevirostrum",
    "Salvelinus_fontinalis",
    "Scomber_scombrus",
    "Lutjanus_griseus",
    "Thunnus_albacares"
)

foreach ($s in $species) {
    Write-Host "`n=== $s ===" -ForegroundColor Cyan
    python -m simulation.run_sim --csv "Midline_*.csv" --species $s `
        --arch cascade --video --duration 4 --quiet-clip
}
