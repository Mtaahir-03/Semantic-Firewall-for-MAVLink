import pandas as pd
import numpy as np
import glob
import os

def analyze_flight_safety(csv_path):
    df = pd.read_csv(csv_path)
    
    # Safely skip non-telemetry CSVs (like ground truth or firewall logs)
    if 'value' not in df.columns or 'msg_type' not in df.columns:
        return None

    df['value'] = pd.to_numeric(df['value'], errors='coerce')
    df = df.dropna(subset=['t_wall', 'value'])

    # Extract absolute Roll and Rollspeed metrics
    roll = df[(df['msg_type'] == 'ATTITUDE') & (df['field'] == 'roll')]['value'].abs()
    rollspeed = df[(df['msg_type'] == 'ATTITUDE') & (df['field'] == 'rollspeed')]['value'].abs()
    
    # Skip files that don't contain ATTITUDE telemetry
    if roll.empty or rollspeed.empty:
        return None

    metrics = {
        'File': os.path.basename(csv_path),
        'Max_Roll_rad': roll.max(),
        'RMS_Roll_rad': np.sqrt(np.mean(roll**2)),
        'Max_Rollspeed_rad_s': rollspeed.max()
    }
    return metrics

def main():
    dataset_dir = '/mnt/hgfs/Dataset'
    all_files = glob.glob(f'{dataset_dir}/**/*.csv', recursive=True)
    
    results = []
    for f in all_files:
        # Ignore non-telemetry directories
        if 'GroundTruth' in f or 'FirewallLogs' in f:
            continue
            
        res = analyze_flight_safety(f)
        if res:
            results.append(res)
            
    if not results:
        print("No valid telemetry CSVs found.")
        return

    results_df = pd.DataFrame(results)
    
    # Group by attack condition (extracts prefix before '_seed')
    results_df['Condition'] = results_df['File'].apply(lambda x: x.split('_seed')[0])
    
    print("\n=== Enhanced Physical Safety Metrics ===")
    summary = results_df.groupby('Condition')[['Max_Roll_rad', 'Max_Rollspeed_rad_s', 'RMS_Roll_rad']].mean()
    
    # Sort for cleaner presentation (Baseline first)
    summary = summary.sort_index()
    print(summary.to_markdown())

if __name__ == '__main__':
    main()