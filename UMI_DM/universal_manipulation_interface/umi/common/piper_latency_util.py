"""Offline delay fitting for measured scalar Piper signals; no hardware access."""
import numpy as np


def validate_series(times, values):
    times, values = np.asarray(times, dtype=float), np.asarray(values, dtype=float)
    if (times.ndim != 1 or values.shape != times.shape or len(times) < 4
            or not np.isfinite(times).all() or not np.isfinite(values).all()
            or np.any(np.diff(times) <= 0)):
        raise ValueError('A signal needs at least four finite, strictly increasing samples')
    return times, values


def estimate_signal_delay(reference_times, reference, feedback_times, feedback, *,
                          min_delay_s=0.0, max_delay_s=.5, step_s=.001,
                          hold_reference=False):
    """Fit feedback(t) ~= gain * reference(t - delay) + offset.

    Every candidate uses the same overlap window. A positive result means
    feedback is later than reference. Commands are zero-order held; physical
    reference measurements are linearly interpolated. No extrapolation occurs.
    """
    rt, rv = validate_series(reference_times, reference)
    ft, fv = validate_series(feedback_times, feedback)
    if (not np.isfinite([min_delay_s, max_delay_s, step_s]).all()
            or max_delay_s <= min_delay_s or step_s <= 0):
        raise ValueError('Invalid delay search interval')
    dt = float(np.median(np.diff(ft)))
    if not hold_reference:
        dt = max(dt, float(np.median(np.diff(rt))))
    start, end = max(ft[0], rt[0] + max_delay_s), min(ft[-1], rt[-1] + min_delay_s)
    grid = np.arange(start, end, dt)
    if len(grid) < 4:
        raise ValueError('Signals have insufficient common time overlap for this search interval')
    y = np.interp(grid, ft, fv)
    y -= y.mean()
    y_norm = np.linalg.norm(y)
    if y_norm <= np.finfo(float).eps * max(1.0, np.linalg.norm(fv)):
        raise ValueError('Feedback has no measurable excitation')
    delays = np.linspace(min_delay_s, max_delay_s,
                         int(np.ceil((max_delay_s - min_delay_s) / step_s)) + 1)
    scores = []
    for delay in delays:
        if hold_reference:
            indices = np.searchsorted(rt, grid - delay, side='right') - 1
            x = rv[indices].copy()
        else:
            x = np.interp(grid - delay, rt, rv)
        x -= x.mean()
        norm = np.linalg.norm(x)
        scores.append(float(x @ y / (norm * y_norm)) if norm > np.finfo(float).eps else None)
    finite = [score for score in scores if score is not None]
    if not finite or np.ptp(finite) <= 32 * np.finfo(float).eps:
        raise ValueError('Delay is unidentifiable from a constant or simple ramp signal')
    best = max((i for i, score in enumerate(scores) if score is not None), key=lambda i: scores[i])
    return {'delay_ms': float(delays[best] * 1000),
            'correlation': scores[best], 'at_search_boundary': best in (0, len(delays) - 1),
            'fit_grid_period_ms': dt * 1000,
            'search_step_ms': float(delays[1] - delays[0]) * 1000,
            'overlap_s': float(end - start),
            'curve': [{'delay_ms': float(d * 1000), 'correlation': s}
                      for d, s in zip(delays, scores)]}


def summarize_ms(values):
    values = np.asarray([v for v in values if v is not None], dtype=float)
    if not len(values):
        return {'count': 0, 'median_ms': None}
    return {'count': int(len(values)), 'median_ms': float(np.median(values)),
            'min_ms': float(values.min()), 'max_ms': float(values.max())}


def gripper_step_responses(targets, feedback_times, feedback_widths):
    """Report 10/50/90% width response times for opening and closing steps.

    Percentages describe displacement toward the commanded width, not motor
    current or first physical motion. Raw sensor zero offset is not changed.
    """
    times, widths = validate_series(feedback_times, feedback_widths)
    rows = []
    for i, target in enumerate(targets):
        begin = target['send_start_monotonic_s']
        end = targets[i + 1]['send_start_monotonic_s'] if i + 1 < len(targets) else times[-1]
        baseline = widths[(times >= begin - .1) & (times < begin)]
        row = {'target_id': target['target_id'], 'target_width_m': target['target_width_m']}
        if not len(baseline):
            rows.append({**row, 'reason': 'no pre-command feedback'})
            continue
        initial = float(np.median(baseline))
        delta = target['target_width_m'] - initial
        if abs(delta) <= np.finfo(float).eps:
            rows.append({**row, 'reason': 'no displacement requested'})
            continue
        mask = (times >= begin) & (times < end)
        t, progress = times[mask], (widths[mask] - initial) / delta
        row.update({'initial_width_m': initial, 'direction': 'opening' if delta > 0 else 'closing'})
        for percentage in (10, 50, 90):
            hits = np.flatnonzero(progress >= percentage / 100)
            row[f't{percentage}_ms'] = float((t[hits[0]] - begin) * 1000) if len(hits) else None
        rows.append(row)
    return rows
