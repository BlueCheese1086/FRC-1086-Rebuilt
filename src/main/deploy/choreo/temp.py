import json
import os

def process_trajectories():
    files = [f for f in os.listdir('.') if f.endswith('.traj')]
    
    for filename in files:
        with open(filename, 'r') as f:
            try:
                data = json.load(f)
            except json.JSONDecodeError:
                print(f"Error reading {filename}. Skipping.")
                continue

        is_stop_path = filename.lower().endswith(('_oc.traj', '_dc.traj'))
        
        sections = ['snapshot', 'params']
        
        for section in sections:
            if section not in data:
                continue
                
            new_constraints = [
                c for c in data[section].get('constraints', [])
                if c.get('data', {}).get('type') != 'StopPoint'
            ]
            
            if is_stop_path:
                last_index = len(data[section].get('waypoints', [])) - 1
                
                if last_index >= 0:
                    stop_constraint = {
                        "from": last_index,
                        "to": None,
                        "data": {"type": "StopPoint", "props": {}},
                        "enabled": True
                    }
                    new_constraints.append(stop_constraint)
            
            data[section]['constraints'] = new_constraints

        with open(filename, 'w') as f:
            json.dump(data, f, indent=1)
        
        status = "Added stop at end" if is_stop_path else "Removed all stops"
        print(f"Processed {filename}: {status}")

if __name__ == "__main__":
    process_trajectories()