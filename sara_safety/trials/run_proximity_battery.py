#!/usr/bin/env python3
# Autor: David Capacho Parra
# Descripción: Batería de ensayos de Proximity Stop (paper 2,
# docs/paper2_draft.tex sec 5, Table 2 - "Static human, moving human,
# boundary chatter check"). Corre run_proximity_trial.py repetidamente
# para cada condición, guarda un CSV crudo por ensayo y un resumen
# agregado. Este es el "batch trial runner" que paper_outline.md Phase 5
# pedía, con el alcance real de esta sesión: N ensayos por condición, no
# los 50 de la validación de navegación de la tesis - eso se declara así
# en el CSV/paper, no se exagera.

import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
from run_proximity_trial import run_one_trial  # noqa: E402

RESULTS_DIR = '/tmp/trial_results'
N_REPEATS = 5


def condition_static(distance_m):
    return {'start': (distance_m, 0.0), 'end': (distance_m, 0.0), 'speed': 1.4}


def condition_moving():
    # end=0.35m: inside the re-derived 0.5m zone (was 0.6m, comfortably
    # inside the old 1.2m zone but OUTSIDE the new one - would silently
    # never trigger if left unchanged).
    return {'start': (3.0, 0.0), 'end': (0.35, 0.0), 'speed': 1.4}


def condition_boundary():
    # Re-derived zone_radius_m=0.5m this session (was 1.2m) - see
    # proximity_stop.py's header for the ISO 13855 derivation.
    return {'start': (0.5, 0.0), 'end': (0.5, 0.0), 'speed': 1.4}


def run_condition(name, params, n, duration_s, results_dir, drive=True):
    rows = []
    for i in range(n):
        trial_id = f'{name}_{i}'
        print(f'[{time.strftime("%H:%M:%S")}] running {trial_id}...', flush=True)
        result = run_one_trial(
            trial_id, params['start'], params['end'], params['speed'],
            spawn_wait_s=9.0, drive_duration_s=duration_s, results_dir=results_dir,
            drive=drive)
        result['condition'] = name
        rows.append(result)
        print(f'  -> {result}', flush=True)
    return rows


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    all_rows = []

    # Static human: two distances comfortably inside the re-derived 0.5m
    # zone (was 0.8/1.0m for the old 1.2m zone) - both also comfortably
    # above the 0.3m self-detection filter floor (proximity_stop.py).
    for d in (0.35, 0.45):
        all_rows += run_condition(f'static_{d}m', condition_static(d), N_REPEATS, 8.0, RESULTS_DIR)

    # Moving human: the actor's own back-and-forth walk.
    all_rows += run_condition('moving', condition_moving(), N_REPEATS, 8.0, RESULTS_DIR)

    # Boundary chatter check: actor held static exactly at the 1.2m zone
    # edge, longer observation window, robot NOT driving (isolates
    # detection-boundary stability from motion/braking effects).
    all_rows += run_condition('boundary_chatter', condition_boundary(), N_REPEATS, 12.0,
                               RESULTS_DIR, drive=False)

    csv_path = os.path.join(RESULTS_DIR, 'proximity_stop_battery.csv')
    fieldnames = ['condition', 'trial_id', 'start_xy', 'end_xy', 'triggered', 'stopped',
                  'success', 'response_time_s', 'stop_distance_m', 'overshoot_m',
                  'chatter_events', 'human_gt_samples', 'final_gt_distance_m']
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for row in all_rows:
            w.writerow({k: row.get(k) for k in fieldnames})
    print(f'Wrote {csv_path} ({len(all_rows)} trials)')


if __name__ == '__main__':
    main()
