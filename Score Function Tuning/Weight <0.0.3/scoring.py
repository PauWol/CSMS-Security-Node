#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat May  2 14:35:08 2026

@author: wolf
"""

# %% Imports
import random
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# %% Weights and Constants
# --- LIGHT (0.0 – 1.0) ---
LIGHT_DAY        = 0.7
LIGHT_DUSK       = 0.25

# --- TIME WEIGHTS ---
WEIGHT_DAY       = 1.0
WEIGHT_DUSK      = 3.0
WEIGHT_NIGHT     = 2.0

# --- PIR ---
PIR_BASE_SCORE   = 1.5

# --- RADAR CONFIDENCE (0.0 – 1.0) ---
CONF_SCALE       = 6.0
CONF_PEAK_BONUS  = 2.0
PEAK_THRESHOLD   = 0.75

# --- BEHAVIOR ---
BONUS_CONTINUOUS = 2.0
BONUS_STABLE     = 1.5

# --- VARIANCE / STABILITY ---
STABLE_VARIANCE  = 0.02

# --- THRESHOLDS ---
THRESH_DAY       = 6.5
THRESH_DUSK      = 5.5 #3.5
THRESH_NIGHT     = 4.5 #4.5

# --- TIMING ---
RADAR_WINDOW     = 8000      # ms
CONTINUOUS_TIME  = 4000      # ms

# %% Helpers

def get_time_weight(light):
    if light >= LIGHT_DAY:
        return WEIGHT_DAY, THRESH_DAY, "DAY"
    elif light >= LIGHT_DUSK:
        return WEIGHT_DUSK, THRESH_DUSK, "DUSK"
    else:
        return WEIGHT_NIGHT, THRESH_NIGHT, "NIGHT"

# %% Calculate Score

def calculate_score(samples, timestamps, light):

    weight, threshold, phase = get_time_weight(light)

    score = PIR_BASE_SCORE

    if len(samples) == 0:
        return 0, threshold, phase

    avg_conf = np.mean(samples)
    max_conf = np.max(samples)
    variance = np.var(samples)


    # --- radar core ---
    score += avg_conf * CONF_SCALE

    # --- strong peak ---
    if max_conf >= PEAK_THRESHOLD:
        score += CONF_PEAK_BONUS

    # --- continuous motion ---
    duration = timestamps[-1] - timestamps[0]
    if duration >= CONTINUOUS_TIME and avg_conf > 0.3:
        score += BONUS_CONTINUOUS

    # --- stability ---
    if variance < STABLE_VARIANCE and avg_conf > 0.25:
        score += BONUS_STABLE

    score = score * weight

    return score, threshold, phase

# %% Mock Data Generators

def mock_light():
    """Simulate day/dusk/night"""
    return random.choice([
        random.uniform(0.8, 1.0),   # day
        random.uniform(0.3, 0.6),   # dusk
        random.uniform(0.0, 0.15)   # night
    ])

def mock_event(event_type):
    """
    Simulate radar confidence patterns
    """
    samples = []
    timestamps = []

    t = 0
    step = 200  # ms

    for _ in range(int(RADAR_WINDOW / step)):
        if event_type == "noise":
            conf = random.uniform(0.05, 0.2)

        elif event_type == "small_predator":
            conf = random.uniform(0.3, 0.5)

        elif event_type == "large_predator":
            conf = random.uniform(0.6, 0.9)

        else:
            conf = 0.0

        samples.append(conf)
        timestamps.append(t)
        t += step

    return samples, timestamps

# %% Simulation

def run_simulation(n=10):

    event_types = ["noise", "small_predator", "large_predator"]
    results = []

    for i in range(n):

        light = mock_light()
        event_type = random.choice(event_types)

        samples, timestamps = mock_event(event_type)

        score, threshold, phase = calculate_score(samples, timestamps, light)

        avg_conf = np.mean(samples)
        max_conf = np.max(samples)
        variance = np.var(samples)
        duration = timestamps[-1] - timestamps[0]

        triggered = score >= threshold

        print(f"\nEvent {i+1}")
        print(f"Type:        {event_type}")
        print(f"Light:       {light:.2f} ({phase})")
        print(f"Avg Conf:    {avg_conf:.2f}")
        print(f"Max Conf:    {max_conf:.2f}")
        print(f"Variance:    {variance:.4f}")
        print(f"Score:       {score:.2f}")
        print(f"Threshold:   {threshold}")
        print(f"ALERT:       {'YES' if triggered else 'no'}")

        results.append({
            "event_type": event_type,
            "light": light,
            "phase": phase,
            "avg_conf": avg_conf,
            "max_conf": max_conf,
            "variance": variance,
            "duration": duration,
            "score": score,
            "threshold": threshold,
            "triggered": int(triggered)
        })

    return pd.DataFrame(results)

# %% Visualization

def plot_correlation(df):
    corr = df[[
        "avg_conf", "max_conf", "variance",
        "duration", "light", "score", "triggered"
    ]].corr()

    plt.figure()
    plt.imshow(corr)
    plt.xticks(range(len(corr.columns)), corr.columns, rotation=45)
    plt.yticks(range(len(corr.columns)), corr.columns)
    plt.title("Correlation Heatmap")
    plt.colorbar()
    plt.tight_layout()


def plot_scatter(df):
    plt.figure()

    for event in df["event_type"].unique():
        subset = df[df["event_type"] == event]
        plt.scatter(subset["avg_conf"], subset["score"], label=event)

    plt.xlabel("Average Confidence")
    plt.ylabel("Score")
    plt.title("Avg Confidence vs Score")
    plt.legend()


def plot_distribution(df):
    plt.figure()

    for event in df["event_type"].unique():
        subset = df[df["event_type"] == event]
        plt.hist(subset["score"], alpha=0.5, label=event)

    plt.xlabel("Score")
    plt.ylabel("Frequency")
    plt.title("Score Distribution by Event Type")
    plt.legend()


# %% Run

if __name__ == "__main__":

    df = run_simulation(300)

    print(df.head())

    plot_correlation(df)
    plot_scatter(df)
    plot_distribution(df)

    plt.show()
