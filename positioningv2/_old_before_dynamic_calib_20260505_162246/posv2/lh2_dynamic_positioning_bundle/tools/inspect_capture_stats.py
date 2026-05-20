#!/usr/bin/env python3
import argparse
import pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument("--obs", required=True)
args = ap.parse_args()

df = pd.read_csv(args.obs)
print("rows:", len(df))
for col in ["sensor_id", "lighthouse_id", "frame_id"]:
    if col in df.columns:
        print(f"{col} unique:", df[col].nunique())
        print(df[col].value_counts().head(20))
print(df.describe(include="all"))
