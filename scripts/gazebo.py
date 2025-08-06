#!/usr/bin/env python3

import os
import subprocess
import sys
import time
from typing import List, Dict, Any

import argparse
import re


def parse_unknown_args(unknown: List[str]) -> Dict[str, Any]:
    """
    Parse the list of unknown arguments from argparse into a dictionary.
    This version specifically handles arguments in the form '--key=value'.
    """
    arg_dict = {}
    pattern = re.compile(r"--([^=]+)=(.*)")
    for arg in unknown:
        match = pattern.match(arg)
        if match:
            key, value = match.groups()
            # Convert value to boolean if it's 'True' or 'False'
            if value == "True":
                value = True
            elif value == "False":
                value = False
            # Check if the key already exists
            if key in arg_dict:
                if isinstance(arg_dict[key], list):
                    arg_dict[key].append(value)
                else:
                    arg_dict[key] = [arg_dict[key], value]
            else:
                arg_dict[key] = value
        else:
            # If the argument doesn't match the pattern, treat it as a flag
            arg_dict[arg.lstrip("-")] = True
    return arg_dict


def main():
    try:
        parser = argparse.ArgumentParser(description="Run the Mujoco Connector")
        parser.add_argument(
            "--file_path", type=str, required=True, help="Path to the Gazebo SDF file"
        )
        parser.add_argument(
            "--headless",
            required=False,
            action="store_true",
            help="Run in headless mode",
        )
        parser.add_argument(
            "--real_time_factor",
            type=float,
            required=False,
            default=1.0,
            help="Real time factor",
        )
        parser.add_argument(
            "--step_size", type=float, required=False, default=0.01, help="Step size"
        )
        parser.add_argument(
            "--max_real_time",
            type=float,
            required=False,
            default=None,
            help="Maximum real time",
        )
        parser.add_argument(
            "--max_number_of_steps",
            type=float,
            required=False,
            default=None,
            help="Maximum number of steps",
        )
        parser.add_argument(
            "--max_simulation_time",
            type=float,
            required=False,
            default=None,
            help="Maximum simulation time",
        )

        args, unknown = parser.parse_known_args()

        # Convert unknown arguments into a dictionary
        unknown_args_dict = parse_unknown_args(unknown)

        assert os.path.exists(args.file_path), f"File {args.file_path} does not exist"

        result = subprocess.run(
            ["gz", "sim", "--versions"], capture_output=True, text=True, check=True
        )
        gz_version = result.stdout.strip()

        gz_plugin_path = os.path.join(
            os.path.dirname(__file__),
            "..",
            "src",
            "gazebo_connector",
            "gazebo_plugin",
            f"gazebo-{gz_version}",
            "libMultiverseConnector.so",
        )
        gz_plugin_path = os.path.normpath(gz_plugin_path)
        assert os.path.exists(gz_plugin_path), f"Gazebo plugin {gz_plugin_path} does not exist"

        env = os.environ.copy()
        env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.path.dirname(gz_plugin_path)

        process_args = ["gz", "sim", args.file_path, "-r"]
        process = subprocess.Popen(process_args, env=env)
        process.wait()

        time.sleep(1)  # Extra time to clean up
    except subprocess.CalledProcessError as e:
        print(f"Error: {e.stderr.strip()}")
    except KeyboardInterrupt:
        time.sleep(1)  # Extra time to clean up
    sys.exit(0)


if __name__ == "__main__":
    main()
