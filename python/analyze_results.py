"""Generate motor-control plots and step-response metrics from a captured CSV."""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def step_metrics(segment):
    """Use observed commanded target and start speed; metrics are estimates at 20 Hz."""
    t = segment['phase_time_s'].to_numpy(dtype=float)
    y = segment['actual_rpm'].to_numpy(dtype=float)
    target = float(segment['target_rpm'].iloc[-1])
    if len(t) < 5 or target <= 0:
        return {'status': 'insufficient data'}
    initial = float(y[0])
    amplitude = target - initial
    if amplitude <= 2:
        return {'status': 'not a positive speed step'}
    low = initial + 0.1 * amplitude
    high = initial + 0.9 * amplitude
    idx10 = np.flatnonzero(y >= low)
    idx90 = np.flatnonzero(y >= high)
    rise = float(t[idx90[0]] - t[idx10[0]]) if len(idx10) and len(idx90) else np.nan
    overshoot = max(0.0, (float(np.max(y)) - target) / amplitude * 100.0)
    band = 0.05 * abs(amplitude)
    outside = np.flatnonzero(np.abs(y - target) > band)
    settling = float(t[outside[-1] + 1]) if len(outside) and outside[-1] + 1 < len(t) else (
        float(t[0]) if not len(outside) else np.nan)
    # Average last 20% of the phase to estimate final speed error.
    tail = y[max(0, int(len(y) * 0.8)):]
    steady_error = float(target - np.mean(tail))
    return dict(status='ok', target_rpm=target, rise_time_s=rise,
                overshoot_pct=overshoot, settling_time_s=settling,
                steady_state_error_rpm=steady_error,
                saturation_pct=float(100 * np.mean(segment['pwm'].to_numpy() >= 255)),
                samples=len(t))


def analyze(csv_path):
    csv_path = Path(csv_path)
    df = pd.read_csv(csv_path)
    if df.empty:
        raise ValueError('No telemetry rows')
    output = csv_path.parent
    df['t_s'] = (df['time_ms'] - df['time_ms'].iloc[0]) / 1000

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 9))
    axes[0].plot(df['t_s'], df['target_rpm'],
             label='Target RPM', linestyle='--')

    axes[0].plot(df['t_s'], df['actual_rpm'],
             label='Measured RPM')
    axes[0].set_ylabel('Speed (RPM)')
    axes[0].legend(loc='best')
    axes[1].plot(df['t_s'], df['pwm'], label='Applied PWM')
    axes[1].plot(df['t_s'], df['pid_raw'], label='Raw PID output')
    axes[1].set_ylabel('PWM command (0–255)')
    axes[1].legend(loc='best')
    axes[2].plot(df['t_s'], df['error_rpm'], label='Control error')
    axes[2].set_ylabel('Error (RPM)')
    axes[2].set_xlabel('Arduino time since logging began (s)')
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    plot_path = output / (csv_path.stem + '.png')
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)

    results = []
    for phase, data in df.groupby('phase', sort=False):
        if phase.startswith('step_'):
            metrics = step_metrics(data)
            metrics['phase'] = phase
            results.append(metrics)
    metrics_path = output / (csv_path.stem + '_metrics.csv')
    pd.DataFrame(results).to_csv(metrics_path, index=False)
    report_path = output / (csv_path.stem + '_report.md')
    with report_path.open('w') as f:
        f.write('# DC motor PID automated validation\n\n')
        f.write(f'Source CSV: `{csv_path.name}`\n\n')
        f.write(f'![Measured response]({plot_path.name})\n\n')
        f.write('## Step metrics\n\n')
        f.write(pd.DataFrame(results).to_markdown(index=False) + '\n\n')
        f.write(
            'Metrics are estimated from encoder measurements sampled '
            'at approximately 20 Hz. Rise time, overshoot, settling time, '
            'and steady-state error characterize the closed-loop response. '
            'The 2% settling metric may be undefined because of encoder '
            'quantization or measurement noise.\n\n'
        )
    print('Saved plot:', plot_path)
    print('Saved metrics:', metrics_path)
    print('Saved report:', report_path)
    print(pd.DataFrame(results).to_string(index=False))
    return results


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('csv_file', type=Path)
    analyze(p.parse_args().csv_file)
