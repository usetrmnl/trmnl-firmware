#!/usr/bin/env python3
"""
PlatformIO Config Flattener

This script generates a flattened JSON representation of the resolved 
platformio.ini configuration. Useful when working with 'extends' or
variable interpolation across multiple environments.
"""
import json
import subprocess
import sys
import os

def flatten_config(data):
    """Flatten the PlatformIO config by sorting environments and their properties"""
    result = {}
    
    for env_data in data:
        env_name = env_data[0]
        properties = env_data[1]
        
        # Sort properties by key name for consistent ordering
        sorted_props = {}
        for prop in properties:
            key = prop[0]
            value = prop[1]
            
            # Sort list values for consistency
            if isinstance(value, list):
                value = sorted(value)
            
            sorted_props[key] = value
        
        result[env_name] = dict(sorted(sorted_props.items()))
    
    # Sort environments by name
    return dict(sorted(result.items()))

def main():
    # Get the project root (parent of scripts directory)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    
    # Run pio project config from project root
    result = subprocess.run(
        ['pio', 'project', 'config', '--json-output'],
        cwd=project_root,
        capture_output=True,
        text=True
    )
    
    if result.returncode != 0:
        print(f"Error running pio command: {result.stderr}", file=sys.stderr)
        sys.exit(1)
    
    # Parse and flatten the config
    data = json.loads(result.stdout)
    flattened = flatten_config(data)
    
    # Output to stdout
    print(json.dumps(flattened, indent=2, sort_keys=True))

if __name__ == "__main__":
    main()