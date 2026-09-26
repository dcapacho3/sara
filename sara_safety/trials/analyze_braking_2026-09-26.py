import json
import sys

PRE_BRAKE_BUFFER_S = 0.3
POST_BRAKE_S = 2.0


def analyze(log_path, brake_sim_time, entity_name='sara'):
    samples = []
    with open(log_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                continue
            stamp = msg.get('header', {}).get('stamp', {})
            sim_t = float(stamp.get('sec', 0)) + float(stamp.get('nsec', 0)) / 1e9
            if sim_t < brake_sim_time - PRE_BRAKE_BUFFER_S or sim_t > brake_sim_time + POST_BRAKE_S:
                continue
            for p in msg.get('pose', []):
                if p.get('name') == entity_name:
                    x = p.get('position', {}).get('x', 0.0)
                    samples.append((sim_t, x))
                    break

    if len(samples) < 3:
        return None, len(samples)

    samples.sort()
    velocities = []
    for i in range(1, len(samples)):
        dt = samples[i][0] - samples[i - 1][0]
        if dt > 1e-4:
            v = (samples[i][1] - samples[i - 1][1]) / dt
            velocities.append((samples[i][0], v))

    max_decel = 0.0
    for i in range(1, len(velocities)):
        dt = velocities[i][0] - velocities[i - 1][0]
        if dt > 1e-4:
            dv = velocities[i][1] - velocities[i - 1][1]
            decel = -dv / dt
            if decel > max_decel:
                max_decel = decel

    return max_decel, len(samples)


if __name__ == '__main__':
    brake_times = {'braking1_pose.jsonl': 15.53, 'braking2_pose.jsonl': 10.921, 'braking3_pose.jsonl': 10.613}
    for name, bt in brake_times.items():
        md, n = analyze(name, bt)
        print(f'{name}: brake_sim_time={bt} n_samples={n} max_decel_mps2={md}')
