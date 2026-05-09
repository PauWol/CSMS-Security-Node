# Threat Scoring Function Tuning

Simulation and optimization framework for the CSMS Security Node threat assessment scoring function. This directory documents the development iterations, parameter tuning, and validation against real-world sensor data.

## Scoring Function Overview

The threat score combines sensor inputs (PIR, Radar, Photocell) across three environmental phases:

| Phase | Light Range | Threshold | Sensitivity |
|-------|-------------|-----------|-------------|
| **DAY** | ≥ 0.7 | 6.5 | Low |
| **DUSK** | 0.25–0.7 | 5.5 | Medium |
| **NIGHT** | < 0.25 | 4.5 | High |

**Base Formula**: `score = (PIR_BASE_SCORE + radar_bonuses) × phase_weight`

---

## Version History

| Version | Date | Focus | Status |
|---------|------|-------|--------|
| **v0.0.0** | May 2, 2026 | Baseline thresholds | Pre-tuned |
| **v0.0.1** | May 2, 2026 | Threshold refinement | Iteration |
| **v0.0.3** | May 2, 2026 | Noise filtering | ⭐ Production |

### v0.0.0 → v0.0.1: Threshold Adjustment
- `THRESH_DUSK`: 3.5 → 5.5 (+57%) to reduce false positives

### v0.0.1 → v0.0.3: Active Noise Filtering (Current)
Added intelligent pre-filtering before score calculation:
```python
NOISE_CUT_OUT_THRESHOLD = 0.25

if avg_confidence < NOISE_CUT_OUT_THRESHOLD:
    return 0, threshold, phase  # Filter noise automatically
```

**Rationale**: Pre-filtering is more robust than threshold tuning alone; eliminates environmental noise (wind, animals, reflections) while preserving genuine motion sensitivity.

---

## Test Results

### Dataset: 652026-night-cycle.csv
**Real sensor data** collected during night cycle (June 5, 2026). Contains ~1,800 samples of PIR, Radar confidence, photocell readings, and computed scores. **Not yet used for further tuning** — reserved for validation of production builds.

### v0.0.3 Performance Visualizations

#### Confidence vs. Score Mapping
![Confidence vs Score](Weights%200.0.3/conf_vs_score.png)

Shows how radar confidence translates to final threat scores. The clear separation validates noise cutout effectiveness: low-confidence signals (< 0.25 avg) return 0 score regardless of other factors.

#### Score Distribution
![Score Distribution](Weights%200.0.3/score_dist.png)

Histogram of computed scores against the night-cycle dataset. Demonstrates natural clustering and threshold placement relative to score density.

#### Sensor Correlation Matrix
![Correlation Heatmap](Weights%200.0.3/corelation_heatmap.png)

Multi-sensor correlation analysis revealing:
- Strong radar ↔ score correlation (validates scoring logic)
- PIR ↔ Radar independence (justifies dual-sensor approach)
- Phase detection accuracy through light ↔ threshold relationship

---

## Directory Structure

```
Score Function Tuning/
├── README.md                    (this file)
├── Data/
│   └── 652026-night-cycle.csv  (real night-cycle sensor data)
├── Weight <0.0.3/              (iteration history)
│   ├── 0.0.0/weights.txt
│   └── 0.0.1/weights.txt
└── Weights 0.0.3/              (current production)
    ├── scoring.py              (v0.0.3 implementation)
    ├── conf_vs_score.png
    ├── score_dist.png
    └── corelation_heatmap.png
```

**Production Integration**: [lib/threat_assessment.py](../lib/threat_assessment.py) and [lib/constants.py](../lib/constants.py)

---

## Running Simulations

1. Load `Data/652026-night-cycle.csv` into Spyder/Jupyter
2. Import `Weights 0.0.3/scoring.py`
3. Run calculations and generate comparison plots
4. For new versions: copy folder, modify parameters, re-validate

---

## References

- **Platform**: Spyder IDE  
- **Language**: Python 3 (tuning) / MicroPython (device)  
- **Updated**: May 9, 2026
