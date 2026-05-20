#!/usr/bin/env python3

import argparse
import pandas as pd
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description="Plot Lighthouse angle data from CSV.")
    parser.add_argument("csv_file")
    args = parser.parse_args()

    df = pd.read_csv(args.csv_file)

    print("Columns found:")
    print(df.columns.tolist())
    print()
    print(df.head())

    required = {"timestamp_us", "sensor_id", "base_station_id", "sweep_id", "angle_rad"}

    if not required.issubset(set(df.columns)):
        print()
        print("This file does not contain decoded angle columns yet.")
        print("For now it may only contain raw serial lines.")
        return

    for sensor_id in sorted(df["sensor_id"].unique()):
        sub = df[df["sensor_id"] == sensor_id]
        plt.figure()
        plt.plot(sub["timestamp_us"], sub["angle_rad"], marker=".")
        plt.xlabel("timestamp_us")
        plt.ylabel("angle_rad")
        plt.title(f"Sensor {sensor_id}")
        plt.grid(True)

    plt.show()


if __name__ == "__main__":
    main()
